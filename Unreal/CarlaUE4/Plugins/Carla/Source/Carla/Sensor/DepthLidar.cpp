// Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma de Barcelona (UAB). This work is licensed under the terms of the MIT license. For a copy, see <https://opensource.org/licenses/MIT>.

#include "Carla.h"
#include "DepthLidar.h"
#include "Carla/Actor/ActorBlueprintFunctionLibrary.h"

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

  RenderTargetPool->SetSize(TextureWidth, TextureHeight);
}

void ADepthLidar::BeginPlay()
{
  // Deactivate it, capture manully
  CaptureComponent2D->Deactivate();

  // LDR is faster
  CaptureComponent2D->CaptureSource = ESceneCaptureSource::SCS_FinalColorLDR;

  LastOrientation = 0.0f;

  Super::BeginPlay();
}

void ADepthLidar::Tick(float DeltaTime)
{
  Super::Tick(DeltaTime);

  float DeltaAngle = RotationRate*DeltaTime;
  CurrentOrientation = LastOrientation + DeltaAngle;

  float CenterAngle = LastOrientation + HFov/2.0;
  while(DeltaAngle>0){
    // Set the Capture orientation
    CaptureComponent2D->SetRelativeRotation(FQuat(FVector(0,0,1), CenterAngle));

    // Get a texture target from texture target pool
    auto TextureTarget = RenderTargetPool->Get();

    // Bind texture target to capture component
    CaptureComponent2D->TextureTarget = TextureTarget;

    // Capture the scene, this will rendering the target on rendering thread
    CaptureComponent2D->CaptureScene();

    ENQUEUE_RENDER_COMMAND(FDepthLidar_SendPixelsInRenderThread)(
      std::bind(&ADepthLidar::HandleCaptureOnRenderingThread, this, TextureTarget, GetDataStream(), _1);
    );

    // Prepare for next iteration
    DeltaAngle -= HFov;
    CenterAngle+= HFov;
  }
}

// This function is called on rendering thread
void ADepthLidar::HandleCaptureOnRenderingThread(SharedPtr<UTextureRenderTarget2D> Target, FAsyncDataStream Stream, FRHICommandListImmediate &InRHICmdList){
  /// @todo Can we make sure the sensor is not going to be destroyed?
  if (!IsPendingKill())
  {
    auto Buffer = Stream.PopBufferFromPool();
    FPixelReader::WritePixelsToBuffer(
        *Target,
        Buffer,
        carla::sensor::SensorRegistry::get<TSensor *>::type::header_offset,
        InRHICmdList);
    
    // make lidar measurement fromt buffer and send it
  }

  // Return rendering target to the pool
  RenderTargetPool->Put(Target);
}

void ADepthLidar::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
  Super::EndPlay(EndPlayReason);
}

// Calculate the Fov of the capture based on lidar configuration
void ADepthLidar::CalcResolutionAndCaptureFov(){
  // Calculate verticle angular resolution and elevations
  if(Description.VerticleAngles.Num() != Description.Channels){
    VReso = carla::geom::Math::ToRadians(Description.UpperFovLimit - Description.LowerFovLimit)/(Description.Channels-1);
    for(int i=0;i<Description.Channels;++i){
      Elevations[i] = carla::geom::Math::ToRadians(Description.LowerFovLimit) + i*VReso;
    }
  }else{
    VReso =std::numeric_limits<float>::max();
    for(int i=1;i<Description.Channels;++i){
      VReso = std::min(VReso, carla::geom::Math::ToRadians(std::abs(Description.VerticleAngles[i]-Description.VerticleAngles[i-1])));
    }
    Description.UpperFovLimit = Description.VerticleAngles.Max();
    Description.LowerFovLimit = Description.VerticleAngles.Min();
  }

  // Original lidar verticle fov
  float lidar_vfov = carla::geom::Math::ToRadians(Description.UpperFovLimit - Description.LowerFovLimit + 1.0);

  // Enlarge the verticle fov cause the edge of image has smaller verticle fov
  VFov = 2.0 * atan( 1.0/(cos(HFov/2.0)*tan(lidar_vfov/2.0)));

  // rotation rate in rad/s
  RotationRate = Description.RotationFrequency*carla::geom::Math::Pi2;

  // Horizon lidar resolution in rad
  HReso = RotationRate/Description.PointsPerSecond;

}

// Calculate the camera projection matrix
void ADepthLidar::CalcProjection(){
  // Use custom projection matrix
  CaptureComponent2D->bUseCustomProjectionMatrix = true;

  // Projection matrix based on FOV
  CaptureComponent2D->CustomProjectionMatrix = 
    FPerspectiveMatrix::FPerspectiveMatrix(HFov/2.0, VFov/2.0, 1.0, 1.0, GNearClippingPlane, GNearClippingPlane);
}

void ADepthLidar::CalcTextureSize(){
  // Up sample 4 times to mitigate aliasing
  TextureWidth = 4 * static_cast<int>(HFov/HReso);
  TextureHeight= 4 * static_cast<int>(VFov/VReso);
}

// Implemention of the RenderTargetPool
FRenderTargetPool::FRenderTargetPtr FRenderTargetPool::Get()
{
  FRenderTargetPtr got = nullptr;

  lock_.lock();
  if (!avialables_.empty())
  {
    got = avialables_.top();
    avialables_.pop();
  }
  lock_.unlock();

  if (!got.IsValid())
  {
    got = MakeShared<UTextureRenderTarget2D>();
    got->CompressionSettings = TextureCompressionSettings::TC_Default;
    got->SRGB = false;
    got->bAutoGenerateMips = false;
    got->AddressX = TextureAddress::TA_Clamp;
    got->AddressY = TextureAddress::TA_Clamp;
    got->InitCustomFormat(width_, height_, PF_B8G8R8A8, bInForceLinearGamma);
  }

  return got;
}

void FRenderTargetPool::Put(const FRenderTargetPool::FRenderTargetPtr &RenderTarget)
{
  lock_.lock();
  avialables_.push(RenderTarget);
  lock_.unlock();
}