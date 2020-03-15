// Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma de Barcelona (UAB). This work is licensed under the terms of the MIT license. For a copy, see <https://opensource.org/licenses/MIT>.

#include "Carla.h"
#include "DepthLidar.h"
#include "Carla/Actor/ActorBlueprintFunctionLibrary.h"
#include "Carla/Sensor/PixelReader.h"
#include <carla/sensor/s11n/LidarMeasurement.h>

#include "Components/SceneCaptureComponent2D.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/TextureRenderTarget2D.h"

#include "ConstructorHelpers.h"
#include "Materials/MaterialInstanceDynamic.h"

float Wrap2PI(float in){
  return in > carla::geom::Math::Pi2<float>()?
         in - carla::geom::Math::Pi2<float>() * static_cast<int>(in/carla::geom::Math::Pi2<float>()):
         in;
}

FActorDefinition ADepthLidar::GetSensorDefinition()
{
  // sensor.lidar.depth
  return UActorBlueprintFunctionLibrary::MakeLidarDefinition(TEXT("depth"));
}

ADepthLidar::ADepthLidar(const FObjectInitializer &ObjectInitializer) : Super(ObjectInitializer)
{
  // Global settings
  PrimaryActorTick.bCanEverTick = true;

  // Create capture component
  CaptureComponent2D = CreateDefaultSubobject<USceneCaptureComponent2D>(
      FName(*FString::Printf(TEXT("DepthLidar_SceneCaptureComponent2D"))));
  CaptureComponent2D->SetupAttachment(RootComponent); // Attach this component to parent sensor actor

  // Load the depth post process material
  ConstructorHelpers::FObjectFinder<UMaterial> Loader(
    #if PLATFORM_LINUX
      TEXT("Material'/Carla/PostProcessingMaterials/SegmentationDepthEffectMaterial.SegmentationDepthEffectMaterial'")
    #else
      TEXT("Material'/Carla/PostProcessingMaterials/DepthEffectMaterial.DepthEffectMaterial'")
    #endif
  );

  if (Loader.Succeeded())
  {
    DepthMaterial = Loader.Object;
    DepthMaterial->GetScalarParameterValue(TEXT("Far_1"), MaxDepth);
    MaxDepth *=1e-2; // cm to m
  }
}

void ADepthLidar::Set(const FActorDescription &ActorDescription)
{
  Super::Set(ActorDescription);

  UActorBlueprintFunctionLibrary::SetLidar(ActorDescription, Description);
}

void ADepthLidar::BeginPlay()
{
  Super::BeginPlay();

  // Deactivate auto capture, capture manully
  CaptureComponent2D->Deactivate();
  CaptureComponent2D->bAutoActivate = false;

  // Set the post depth material
  CaptureComponent2D->PostProcessSettings.AddBlendable(UMaterialInstanceDynamic::Create(DepthMaterial, this), 1.0);

  // LDR is faster
  CaptureComponent2D->CaptureSource = ESceneCaptureSource::SCS_FinalColorLDR;

  // Calculate parameters
  ApplyConfig();

  // Create the rendering pool and
  // set the texture size of the rendering pool generated
  RenderTargetPool = MakeUnique<FTexturePool>();
  check(RenderTargetPool.IsValid());

  // Start from zero
  LastOrientation = 0.0f;
  Scan = 0;
}

void ADepthLidar::Tick(float DeltaTime)
{
  Super::Tick(DeltaTime);
  //UE_LOG(LogTemp, Log, TEXT("DeltaTime: %f"), DeltaTime);

  // Total scan fov this tick
  float ScanFov = RotationRate * DeltaTime;
  UE_LOG(LogTemp, Log, TEXT("ScanFov: %f"), ScanFov);

  // We should use how many capture to cover this scan
  int CaptureNum = SetScanFov(ScanFov);
  UE_LOG(LogTemp, Log, TEXT("Capture Number: %d"), CaptureNum);

  // Process all the CaptureNum captures
  for (int i = 0; i < CaptureNum; ++i)
  {
    //[CaptureStartOrientation, CaptureEndOrientation) is going to be processed by this capture
    float CaptureStartOrientation = LastOrientation + i * HStep;
    float CaptureEndOrientation = LastOrientation + (i + 1) * HStep;

    //[RayStartID, RayEndId) is covered by this capture
    int RayStartID = std::ceil(CaptureStartOrientation/HReso);
    int RayEndID = std::floor(CaptureEndOrientation/HReso) + 1;

    // Set the Capture orientation
    CaptureComponent2D->SetRelativeRotation(FQuat(FVector(0, 0, 1), (CaptureStartOrientation + CaptureEndOrientation) / 2.0));

    // Construct a CaptureInfo of this capture
    FCaptureInfo CaptureInfo;
    CaptureInfo.CaptureStartOrientation = CaptureStartOrientation;
    CaptureInfo.CaptureEndOrientation = CaptureEndOrientation;
    CaptureInfo.RayStartID = RayStartID;
    CaptureInfo.RayEndID = RayEndID;
    CaptureInfo.Width = TextureSize.X;
    CaptureInfo.Height = TextureSize.Y;
    CaptureInfo.HFov = HFov;
    CaptureInfo.VFov = VFov;
    CaptureInfo.Scan = Scan;
    CaptureInfo.Empty = false;

    // Get a texture target from texture target pool
    auto TextureTarget = RenderTargetPool->Get(TextureSize);

    // Process only if there is available texture, otherwise send a dummy one
    if (TextureTarget) {
      // Bind texture target to capture component
      CaptureComponent2D->TextureTarget = TextureTarget;

      // Capture the scene, this will rendering the texture on rendering thread
      CaptureComponent2D->CaptureScene();

      // Process the rendered target on rendering thread
      ENQUEUE_RENDER_COMMAND(FDepthLidar_WaitForCaptureDone)
      ([Sensor = this, CaptureInfo, TextureTarget, Stream = GetDataStream(*this)](FRHICommandListImmediate &InRHICmdList) mutable {
        auto StreamPtr = std::make_shared<decltype(Stream)>(std::move(Stream));
        auto RenderResource = static_cast<const FTextureRenderTarget2DResource *>(TextureTarget->Resource);
        InRHICmdList.ReadSurfaceDataAsync(
            RenderResource->GetRenderTargetTexture(),
            FIntRect(0, 0, RenderResource->GetSizeXY().X, RenderResource->GetSizeXY().Y),
            FReadSurfaceDataFlags(RCM_UNorm, CubeFace_MAX),
            [Sensor, CaptureInfo, TextureTarget, StreamPtr](TArray<FColor> &&Pixels) mutable {
              Sensor->PutRenderTarget(TextureTarget);
              Sensor->SendPixels(std::move(Pixels), CaptureInfo, StreamPtr);
            });
      });
    } else{
      CaptureInfo.Empty = true; // Set Empty to true to send a dummy one
      SendPixels(TArray<FColor>{}, CaptureInfo, std::make_shared<FAsyncDataStream>(GetDataStream(*this)));
    }

    ++Scan; // Scan always increase
  }

  // Update LastOrientation for next tick, wrap in [0~2*PI)
  LastOrientation = Wrap2PI(LastOrientation + ScanFov);
}

void ADepthLidar::SendPixels(TArray<FColor>&& Pixels, FCaptureInfo CaptureInfo, std::shared_ptr<FAsyncDataStream> StreamPtr) const
{
    // Make a new LidarMeasurement
    carla::sensor::s11n::LidarMeasurement LidarMeasurement(Description.Channels);

    // Set the capture center orientation
    float CaptureCenterOrientation = (CaptureInfo.CaptureStartOrientation + CaptureInfo.CaptureEndOrientation)/2.0;
    
    if (!CaptureInfo.Empty) {
      float RayOrientation, RayPitch, RayYaw;
      for (int Channel = 0; Channel < Description.Channels; ++Channel) {
        for (int Ray = CaptureInfo.RayStartID; Ray < CaptureInfo.RayEndID; ++Ray) {
          // Current ray horizontal orientation
          RayOrientation = Ray * HReso;

          // Current ray yaw relative to capture center
          RayYaw = RayOrientation - CaptureCenterOrientation;
          RayPitch = -Elevations[Channel]; // Elevations up horizon is positive, which is opposite to left-hand coordinate

          // Calc the image coordinates from orientation
          int U = static_cast<int>((std::tan(RayYaw) / std::tan(CaptureInfo.HFov / 2.0) + 1.0) * 0.5 * CaptureInfo.Width);
          int V = static_cast<int>((1.0 + std::tan(RayPitch) / std::tan(CaptureInfo.VFov / 2.0) / std::cos(RayYaw)) * 0.5 * CaptureInfo.Height);

          // Todo make some interpolation
          const auto &Color = Pixels[V * CaptureInfo.Width + U];
          float Depth = (Color.R + Color.G * 256.0f) / static_cast<float>(256 * 256 - 1) * MaxDepth;
          //UE_LOG(LogTemp, Log, TEXT("Alpha: %d"), Color.A);

          // Get point coordinate in sensor frame
          carla::rpc::Location Point;
          Point.x = cos(RayOrientation - PI/2.0) * Depth / cos(RayYaw);
          Point.y = sin(RayOrientation - PI/2.0) * Depth / cos(RayYaw);
          Point.z = tan(RayPitch)                * Depth / cos(RayYaw);

          if (Point.Length() < Description.Range) {
            LidarMeasurement.WritePoint(Channel, Point, Color.B);
          }
        }
      }
    }

    // Send the LidarMeasurement
    LidarMeasurement.SetHorizontalAngle(CaptureInfo.CaptureStartOrientation);
    LidarMeasurement.SetHorizontalEndAngle(CaptureInfo.CaptureEndOrientation);
    LidarMeasurement.SetScan(CaptureInfo.Scan);
    StreamPtr->Send(*this, LidarMeasurement, StreamPtr->PopBufferFromPool());
}

void ADepthLidar::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
  Super::EndPlay(EndPlayReason);
  RenderTargetPool.Release();
}

// Calculate the Fov of the capture based on lidar configuration
void ADepthLidar::ApplyConfig()
{
  // Calculate verticle angular resolution and elevations
  Elevations.SetNum(Description.Channels);
  if (Description.VerticleAngles.Num() != Description.Channels)
  {
    float delta = carla::geom::Math::ToRadians(Description.UpperFovLimit - Description.LowerFovLimit) / (Description.Channels - 1.0);
    for (int i = 0; i < Description.Channels; ++i)
    {
      Elevations[i] = carla::geom::Math::ToRadians(Description.LowerFovLimit) + i * delta;
    }
  }
  else
  {
    Elevations[0] = carla::geom::Math::ToRadians(Description.VerticleAngles[0]);
    Description.UpperFovLimit = Description.VerticleAngles[0];
    Description.LowerFovLimit = Description.VerticleAngles[0];
    for (int i = 1; i < Description.Channels; ++i)
    {
      Description.UpperFovLimit = std::max(Description.UpperFovLimit,  Description.VerticleAngles[i]);
      Description.LowerFovLimit = std::min(Description.LowerFovLimit,  Description.VerticleAngles[i]);

      Elevations[i] = carla::geom::Math::ToRadians(Description.VerticleAngles[i]);
    }
  }
  
  // Original lidar verticle fov, enlarged by 2 degrees
  LidarVFov = carla::geom::Math::ToRadians(std::max(std::abs(Description.UpperFovLimit), std::abs(Description.LowerFovLimit))*2 + 2.0);

  // rotation rate in rad/s
  RotationRate = Description.RotationFrequency * carla::geom::Math::Pi2<float>();

  // Horizon lidar resolution in rad
  HReso = RotationRate / Description.PointsPerSecond;
  VReso = HReso; // force verticle reso to be same
}

// Calculate the camera projection matrix
void ADepthLidar::SetProjectionMatrix()
{
  // Use custom projection matrix
  CaptureComponent2D->bUseCustomProjectionMatrix = true;

  //// Projection matrix based on FOV
  if ((int32)ERHIZBuffer::IsInverted){
  CaptureComponent2D->CustomProjectionMatrix =
    FReversedZPerspectiveMatrix(HFov / 2.0, VFov / 2.0, 1.0, 1.0, GNearClippingPlane, GNearClippingPlane);
  }else{
    CaptureComponent2D->CustomProjectionMatrix =
    FPerspectiveMatrix(HFov / 2.0, VFov / 2.0, 1.0, 1.0, GNearClippingPlane, GNearClippingPlane);
  }
}

int ADepthLidar::SetScanFov(float ScanFov)
{
  int N=1;
  while(MaxHStep*N < ScanFov - HReso ) ++N;

  HStep = ScanFov/N;
  HFov = HStep + carla::geom::Math::ToRadians(2.0);
  UE_LOG(LogTemp, Log, TEXT("HStep: %f"), HStep);

  // Enlarge the verticle fov cause the edge of image has smaller verticle fov
  VFov = 2.0 * atan(tan(LidarVFov/ 2.0) / cos(HFov / 2.0));

  // Set the texture size, 2 time upsample
  TextureSize.X = 2 * static_cast<int>(HFov / HReso);
  TextureSize.Y = 2 *  static_cast<int>(VFov / VReso);
  UE_LOG(LogTemp, Log, TEXT("Texture Size: %d, %d"), TextureSize.X, TextureSize.Y);

  // set the projection matrix based on Fov
  SetProjectionMatrix();

  return N;
}

void ADepthLidar::PutRenderTarget(const FRenderTargetPtr &TextureTarget) const{
  if (RenderTargetPool.IsValid())
  {
    RenderTargetPool->Put(TextureTarget);
  }
}

// Implemention of the RenderTargetPool
FRenderTargetPtr FTexturePool::Get(const FIntPoint& Size)
{
  FRenderTargetPtr Got = nullptr;

  int AvialableSize = 0;
  {
    // Try to get one from the pool
    FScopeLock ScopeLock(&CS);
    if (!Avialables.empty())
    {
      Got = Avialables.top();
      Avialables.pop();
      AvialableSize = Avialables.size();
    }
  }

  // Pool is empty, create a new one
  if (Got==nullptr && Total.size()<10)
  {
    //Todo: make clear what this config means
    Got = NewObject<UTextureRenderTarget2D>();
    Got->AddToRoot();
    Total.push(Got);


    Got->CompressionSettings = TextureCompressionSettings::TC_Default;
    Got->SRGB = false;
    Got->bAutoGenerateMips = false;
    Got->bForceLinearGamma= true;
    Got->AddressX = TextureAddress::TA_Clamp;
    Got->AddressY = TextureAddress::TA_Clamp;
    Got->InitCustomFormat(Size.X, Size.Y, PF_B8G8R8A8, true);
  }

  if(Got!=nullptr){
    Got->ResizeTarget(Size.X, Size.Y); // This function will do nothing if the size do not change
  }

  UE_LOG(LogTemp, Log, TEXT("Texture Used: %d"), Total.size() - AvialableSize);

  return Got;
}

void FTexturePool::Put(const FRenderTargetPtr &RenderTarget)
{
  FScopeLock ScopeLock(&CS);
  Avialables.push(RenderTarget);
}

FTexturePool::~FTexturePool(){
  while(!Total.empty()){
    Total.top()->ReleaseResource();
    Total.top()->RemoveFromRoot();
    Total.pop();
  }
}
