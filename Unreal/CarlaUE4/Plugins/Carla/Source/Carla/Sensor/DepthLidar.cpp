// Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma de Barcelona (UAB). This work is licensed under the terms of the MIT license. For a copy, see <https://opensource.org/licenses/MIT>.

#include "Carla.h"
#include "DepthLidar.h"
#include "Carla/Actor/ActorBlueprintFunctionLibrary.h"
#include "Carla/Sensor/PixelReader.h"
#include <carla/sensor/s11n/LidarMeasurement.h>

#include "Components/SceneCaptureComponent2D.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/TextureRenderTarget2D.h"

FActorDefinition ADepthLidar::GetSensorDefinition()
{
  // sensor.lidar.depth
  return UActorBlueprintFunctionLibrary::MakeLidarDefinition(TEXT("depth"));
}

ADepthLidar::ADepthLidar(const FObjectInitializer &ObjectInitializer) : Super(ObjectInitializer)
{
  // Global settings
  PrimaryActorTick.bCanEverTick = true;
  PrimaryActorTick.TickGroup = TG_PrePhysics;

  // Create capture component
  CaptureComponent2D = CreateDefaultSubobject<USceneCaptureComponent2D>(
      FName(*FString::Printf(TEXT("SceneCaptureComponent2D"))));
  CaptureComponent2D->SetupAttachment(RootComponent);

  // Create the rendering pool
  RenderTargetPool = MakeUnique<FRenderTargetPool>();
}

void ADepthLidar::Set(const FActorDescription &ActorDescription)
{
  Super::Set(ActorDescription);

  UActorBlueprintFunctionLibrary::SetLidar(ActorDescription, Description);

  CalcResolutionAndCaptureFov();

  CalcProjection();

  CalcTextureSize();

  // Set the texture size of the rendering pool generated
  RenderTargetPool->SetSize(TextureSize);
}

void ADepthLidar::BeginPlay()
{
  // Deactivate it, capture manully
  CaptureComponent2D->Deactivate();

  // LDR is faster
  CaptureComponent2D->CaptureSource = ESceneCaptureSource::SCS_FinalColorLDR;

  // Start from zero
  LastOrientation = 0.0f;
  RayStartOrientation = 0.0f;

  Super::BeginPlay();
}

void ADepthLidar::Tick(float DeltaTime)
{
  Super::Tick(DeltaTime);

  // [LastOrientation, CurrentOrientation) is going to be processed,
  // each time HStep at most
  CurrentOrientation = LastOrientation +RotationRate * DeltaTime;

  float CaptureStartOrientation = LastOrientation;
  float CaptureEndOrientation = LastOrientation;
  while (CaptureStartOrientation < CurrentOrientation)
  {
    // Capture end orientation, clamped by CurrentOrientation
    CaptureEndOrientation = std::min(CaptureStartOrientation + HStep, CurrentOrientation);

    // Ray end orientation of this capture
    int N = static_cast<int>((CaptureEndOrientation-RayStartOrientation)/HReso);
    if( std::fmod(CaptureEndOrientation-RayStartOrientation, HReso) > 0 ) ++N;
    RayEndOrientation = RayStartOrientation + N*HReso;

    // Set the Capture orientation
    float CaptureCenterOrientation = (CaptureStartOrientation+CaptureEndOrientation)/2.0;
    CaptureComponent2D->SetRelativeRotation(FQuat(FVector(0, 0, 1), CaptureCenterOrientation));

    // Get a texture target from texture target pool
    auto TextureTarget = RenderTargetPool->Get();

    // Bind texture target to capture component
    CaptureComponent2D->TextureTarget = TextureTarget.Get();

    // Capture the scene, this will rendering the texture on rendering thread
    CaptureComponent2D->CaptureScene();

    // Construct a CaptureInfo of this capture
    FCaptureInfo CaptureInfo;
    CaptureInfo.CaptureCenterOrientation = CaptureCenterOrientation;
    CaptureInfo.RayStartOrientation = RayStartOrientation;
    CaptureInfo.RayNumber = N;
    
    // Process the rendered target on rendering thread
    ENQUEUE_RENDER_COMMAND(FDepthLidar_SendPixelsInRenderThread)
    (
      std::bind(&ADepthLidar::HandleCaptureOnRenderingThread, this, 
                                                              CaptureInfo, 
                                                              TextureTarget, 
                                                              GetDataStream(*this), 
                                                              std::placeholders::_1)
    );

    // Update for next capture
    std::swap(CaptureStartOrientation, CaptureEndOrientation);
    std::swap(RayStartOrientation, RayEndOrientation);
  }

  // Update LastOrientation for next tick, wrap in [0~2*PI)
  LastOrientation = (CurrentOrientation >= carla::geom::Math::Pi2<float>()) ? CurrentOrientation - carla::geom::Math::Pi2<float>() : CurrentOrientation;
}

// This function is called on rendering thread
void ADepthLidar::HandleCaptureOnRenderingThread(FCaptureInfo CaptureInfo,
                                                 FRenderTargetPtr Target,
                                                 FAsyncDataStream& Stream,
                                                 FRHICommandListImmediate &InRHICmdList) const
{
  // Check if self is still alive
  if (IsPendingKill())
    return;

  // Dump texture data into buffer
  auto Buffer = Stream.PopBufferFromPool();
  FPixelReader::WritePixelsToBuffer( *Target, Buffer, 0, InRHICmdList);

  // Make a new LidarMeasurement
  carla::sensor::s11n::LidarMeasurement LidarMeasurement(Description.Channels);
  LidarMeasurement.SetHorizontalAngle(CaptureInfo.CaptureCenterOrientation);

  // Populate the LidarMeasurement
  float RayOrientation, RayPitch, RayYaw;
  for (int Channel = 0; Channel < Description.Channels; ++Channel)
  {
    for(int Ray = 0;Ray<CaptureInfo.RayNumber;++Ray)
    {
      // Current ray horizontal orientation
      RayOrientation = CaptureInfo.RayStartOrientation + Ray*HReso;

      // Current ray yaw relative to capture center
      RayYaw = RayOrientation - CaptureInfo.CaptureCenterOrientation;
      RayPitch = Elevations[Channel];

      // Calc the image coordinates from orientation
      int U = static_cast<int>(std::tan(RayYaw) * static_cast<float>(TextureSize.X) / std::tan(HFov/2.0) / 2.0 + static_cast<float>(TextureSize.X) / 2.0);
      int V = static_cast<int>(std::tan(RayPitch) * static_cast<float>(TextureSize.Y) / std::tan(VFov/2.0) / 2.0 + static_cast<float>(TextureSize.Y) / 2.0);

      // Get the depth of the point
      float Depth = reinterpret_cast<float>(Buffer[4*(y*TextureSize.X + x)]);

      // Polar coordinates to cartisian coordinates
      FVector Point;
      Point.X = std::cos(RayOrientation)*Depth;
      Point.Y = std::sin(RayOrientation)*Depth;
      Point.Z = std::tan(RayPitch)*Depth;

      // Add point into LidarMeasurement
      LidarMeasurement.WritePoint(Channel, Point);
    }
  }

  // Send the LidarMeasurement via stream
  Stream.Send(*this, LidarMeasurement, Stream.PopBufferFromPool());

  // Return rendering target to the pool
  RenderTargetPool->Put(Target);
}

void ADepthLidar::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
  Super::EndPlay(EndPlayReason);
}

// Calculate the Fov of the capture based on lidar configuration
void ADepthLidar::CalcResolutionAndCaptureFov()
{
  // Calculate verticle angular resolution and elevations
  if (Description.VerticleAngles.Num() != Description.Channels)
  {
    VReso = carla::geom::Math::ToRadians(Description.UpperFovLimit - Description.LowerFovLimit) / (Description.Channels - 1);
    for (int i = 0; i < Description.Channels; ++i)
    {
      Elevations[i] = carla::geom::Math::ToRadians(Description.LowerFovLimit) + i * VReso;
    }
  }
  else
  {
    Description.UpperFovLimit = Description.VerticleAngles[0];
    Description.LowerFovLimit = Description.VerticleAngles[0];
    VReso = std::numeric_limits<float>::max();
    for (int i = 1; i < Description.Channels; ++i)
    {
      Description.UpperFovLimit = std::max(Description.UpperFovLimit,  Description.VerticleAngles[i]);
      Description.LowerFovLimit = std::min(Description.LowerFovLimit,  Description.VerticleAngles[i]);

      VReso = std::min(VReso, carla::geom::Math::ToRadians(std::abs(Description.VerticleAngles[i] - Description.VerticleAngles[i - 1])));
    }
   }

  // Original lidar verticle fov
  float lidar_vfov = carla::geom::Math::ToRadians(Description.UpperFovLimit - Description.LowerFovLimit + 1.0);

  // Enlarge the verticle fov cause the edge of image has smaller verticle fov
  VFov = 2.0 * atan(1.0 / (cos(HFov / 2.0) * tan(lidar_vfov / 2.0)));

  // rotation rate in rad/s
  RotationRate = Description.RotationFrequency * carla::geom::Math::Pi2<float>();

  // Horizon lidar resolution in rad
  HReso = RotationRate / Description.PointsPerSecond;
}

// Calculate the camera projection matrix
void ADepthLidar::CalcProjection()
{
  // Use custom projection matrix
  CaptureComponent2D->bUseCustomProjectionMatrix = true;

  // Projection matrix based on FOV
  CaptureComponent2D->CustomProjectionMatrix =
    FPerspectiveMatrix(HFov / 2.0, VFov / 2.0, 1.0, 1.0, GNearClippingPlane, GNearClippingPlane);
}

void ADepthLidar::CalcTextureSize()
{
  // Upsample 4 times to mitigate aliasing
  TextureSize.X = 4 * static_cast<int>(HFov / HReso);
  TextureSize.Y = 4 * static_cast<int>(VFov / VReso);
}

// Implemention of the RenderTargetPool
FRenderTargetPtr FRenderTargetPool::Get()
{
  FRenderTargetPtr Got = nullptr;

  // Try to get one from the pool
  Lock.lock();
  if (!Avialables.empty())
  {
    Got = Avialables.top();
    Avialables.pop();
  }
  Lock.unlock();

  // Pool is empty, create a new one
  if (!Got.IsValid())
  {
    //Todo: make clear what this config means
    Got = MakeShared<UTextureRenderTarget2D>();
    Got->CompressionSettings = TextureCompressionSettings::TC_Default;
    Got->SRGB = false;
    Got->bAutoGenerateMips = false;
    Got->AddressX = TextureAddress::TA_Clamp;
    Got->AddressY = TextureAddress::TA_Clamp;
    Got->InitCustomFormat(Width, Height, PF_B8G8R8A8, true);
  }

  return Got;
}

void FRenderTargetPool::Put(const FRenderTargetPtr &RenderTarget)
{
  Lock.lock();
  Avialables.push(RenderTarget);
  Lock.unlock();
}


void FRenderTargetPool::SetSize(const FIntPoint& Size){
  Width = Size.X;
  Height = Size.Y;
}