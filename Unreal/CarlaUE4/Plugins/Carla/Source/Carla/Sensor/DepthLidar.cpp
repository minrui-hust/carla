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

#include <chrono>

float Wrap2PI(float in){
  return in > carla::geom::Math::Pi2<float>()?
         in - carla::geom::Math::Pi2<float>():
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
      TEXT("Material'/Carla/PostProcessingMaterials/DepthEffectMaterial_GLSL.DepthEffectMaterial_GLSL'")
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
  //CaptureComponent2D->bCaptureEveryFrame = false;
  //CaptureComponent2D->bCaptureOnMovement = false;
  CaptureComponent2D->Deactivate();

  // Set the post depth material
  CaptureComponent2D->PostProcessSettings.AddBlendable(UMaterialInstanceDynamic::Create(DepthMaterial, this), 1.0);

  // LDR is faster
  CaptureComponent2D->CaptureSource = ESceneCaptureSource::SCS_FinalColorLDR;

  // Remove other post process effect
  RemoveOtherPostProcessingEffect(CaptureComponent2D->ShowFlags);

  // Calculate parameters
  CalcResolutionAndCaptureFov();
  CalcProjection();
  CalcTextureSize();

  // Create the rendering pool and
  // set the texture size of the rendering pool generated
  RenderTargetPool = MakeUnique<FRenderTargetPool>(TextureSize);

  // Start from zero
  LastOrientation = 0.0f;
  RayStartOrientation = 0.0f;
}

namespace
{

class FVoidTask
{
  TFunction<void(void)> Task;

public:
  static ESubsequentsMode::Type GetSubsequentsMode()
  {
    // Don't support tasks having dependencies on us, reduces task graph overhead tracking and dealing with subsequents
    return ESubsequentsMode::FireAndForget;
  }

  template<typename TTask>
  FVoidTask(TTask&& task)
      : Task(std::forward<TTask>(task))
  {
  }

  FORCEINLINE TStatId GetStatId() const
  {
    RETURN_QUICK_DECLARE_CYCLE_STAT(FVoidTask, STATGROUP_TaskGraphTasks);
  }

  ENamedThreads::Type GetDesiredThread()
  {
    return ENamedThreads::AnyThread;
  }

  void DoTask(ENamedThreads::Type CurrentThread, const FGraphEventRef &MyCompletionGraphEvent)
  {
    Task(); // Call the Task
  }
};

} // namespace

void ADepthLidar::Tick(float DeltaTime)
{
  Super::Tick(DeltaTime);
  UE_LOG(LogTemp, Log, TEXT("Delta: %f"), DeltaTime);

  // [LastOrientation, CurrentOrientation) is going to be processed,
  // each time HStep at most
  CurrentOrientation = LastOrientation + RotationRate * DeltaTime;

  int slice = 0;
  float CaptureStartOrientation = LastOrientation;
  float CaptureEndOrientation = LastOrientation;
  while (CaptureStartOrientation < CurrentOrientation && (CaptureStartOrientation-LastOrientation) <= carla::geom::Math::Pi2<float>())
  {
    // Capture end orientation, clamped by CurrentOrientation and 2pi
    CaptureEndOrientation = std::min(CaptureStartOrientation + HStep, std::min(CurrentOrientation, LastOrientation+carla::geom::Math::Pi2<float>()));

    // Ray end orientation of this capture
    int N = static_cast<int>((CaptureEndOrientation-RayStartOrientation)/HReso);
    if( std::fmod(CaptureEndOrientation-RayStartOrientation, HReso) > 0 ) ++N;
    RayEndOrientation = RayStartOrientation + N*HReso;

    // Set the Capture orientation
    float CaptureCenterOrientation = (CaptureStartOrientation+CaptureEndOrientation)/2.0;
    CaptureComponent2D->SetRelativeRotation(FQuat(FVector(0,0,1), CaptureCenterOrientation));

    // Get a texture target from texture target pool
    auto TextureTarget = RenderTargetPool->Get();

    // Bind texture target to capture component
    CaptureComponent2D->TextureTarget = TextureTarget;

    // Capture the scene, this will rendering the texture on rendering thread
    CaptureComponent2D->CaptureScene();

    // Construct a CaptureInfo of this capture
    FCaptureInfo CaptureInfo;
    CaptureInfo.CaptureCenterOrientation = CaptureCenterOrientation;
    CaptureInfo.RayStartOrientation = RayStartOrientation;
    CaptureInfo.RayNumber = N;

    // Process the rendered target on rendering thread
    ENQUEUE_RENDER_COMMAND(FDepthLidar_WaitForCaptureDone)
    ([Sensor = this, CaptureInfo, TextureTarget, Stream = GetDataStream(*this)](FRHICommandListImmediate &InRHICmdList) mutable {

      // Get the fence event on render thread
      // This fence make sure the capture task has been executed by RHI thread
      auto ev = InRHICmdList.RHIThreadFence();

      // Wait the fence event on other thread, so not blocking the render thread
      FGraphEventArray PreRequests = {ev};
      auto StreamPtr = std::make_shared<decltype(Stream)>(std::move(Stream));
      TGraphTask<FVoidTask>::CreateTask(&PreRequests).ConstructAndDispatchWhenReady([Sensor, CaptureInfo, TextureTarget, StreamPtr]() mutable {
        // When the event fired,
        // Call the Async Read to read the texture, with a callback sending the data
        auto RenderResource = static_cast<const FTextureRenderTarget2DResource *>(TextureTarget->Resource);
        GDynamicRHI->RHIReadSurfaceDataAsync(RenderResource->GetRenderTargetTexture(),
                                             FIntRect(0, 0, RenderResource->GetSizeXY().X, RenderResource->GetSizeXY().Y),
                                             FReadSurfaceDataFlags(RCM_UNorm, CubeFace_MAX),
                                             [Sensor, CaptureInfo, TextureTarget, StreamPtr](TArray<FColor> &&Pixels) mutable {
                                               // Todo check if Sensor is still valid
                                               {
                                                 Sensor->PutRenderTarget(TextureTarget);
                                                 Sensor->SendPixelsOnOtherThread(std::move(Pixels), CaptureInfo, StreamPtr);
                                               }
                                             });
      });
    });

    // Update for next capture
    std::swap(CaptureStartOrientation, CaptureEndOrientation);
    std::swap(RayStartOrientation, RayEndOrientation);
    ++slice;
  }
  UE_LOG(LogTemp, Log, TEXT("Slice: %d"), slice);

  // Update LastOrientation for next tick, wrap in [0~2*PI)
  LastOrientation = Wrap2PI(CurrentOrientation);
}

void ADepthLidar::SendPixelsOnOtherThread(TArray<FColor> Pixels, FCaptureInfo CaptureInfo, std::shared_ptr<FAsyncDataStream> StreamPtr) const
{
    // Make a new LidarMeasurement
    carla::sensor::s11n::LidarMeasurement LidarMeasurement(Description.Channels);
    LidarMeasurement.SetHorizontalAngle(CaptureInfo.CaptureCenterOrientation);

    // Populate the LidarMeasurement
    float RayOrientation, RayPitch, RayYaw;
    for (int Channel = 0; Channel < Description.Channels; ++Channel)
    {
      for (int Ray = 0; Ray < CaptureInfo.RayNumber; ++Ray)
      {
        // Current ray horizontal orientation
        RayOrientation = CaptureInfo.RayStartOrientation + Ray * HReso;

        // Current ray yaw relative to capture center
        RayYaw = RayOrientation - CaptureInfo.CaptureCenterOrientation;
        RayPitch = Elevations[Channel];

        // Calc the image coordinates from orientation
        int U = static_cast<int>((std::tan(RayYaw) / std::tan(HFov / 2.0) + 1.0) * (0.5 * static_cast<float>(TextureSize.X)));
        int V = static_cast<int>((1.0 - std::tan(RayPitch) / std::tan(VFov / 2.0) / std::cos(RayYaw)) * (0.5 * static_cast<float>(TextureSize.Y)));

        // Get the depth
        const auto& Color = Pixels[V*TextureSize.X + U];
        float Depth = (Color.R + Color.G * 255.0f + Color.B * 255.0f * 255.0f) / static_cast<float>(255 * 255 * 255 - 1) * MaxDepth;

        // Get point coordinate in Capture frame
        carla::rpc::Location Point;
        Point.x = cos(RayOrientation) * Depth / cos(RayYaw);
        Point.y = sin(RayOrientation) * Depth / cos(RayYaw);
        Point.z = std::tan(RayPitch) * Depth;

        if (Point.Length() < Description.Range)
        {
          LidarMeasurement.WritePoint(Channel, Point);
        }
      }
  }

  // Send the LidarMeasurement via stream
  StreamPtr->Send(*this, LidarMeasurement, StreamPtr->PopBufferFromPool());
}

void ADepthLidar::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
  Super::EndPlay(EndPlayReason);
  RenderTargetPool.Release();
}

// Calculate the Fov of the capture based on lidar configuration
void ADepthLidar::CalcResolutionAndCaptureFov()
{
  // Calculate verticle angular resolution and elevations
  Elevations.SetNum(Description.Channels);
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
    Elevations[0] = carla::geom::Math::ToRadians(Description.VerticleAngles[0]);
    Description.UpperFovLimit = Description.VerticleAngles[0];
    Description.LowerFovLimit = Description.VerticleAngles[0];
    VReso = std::numeric_limits<float>::max();
    for (int i = 1; i < Description.Channels; ++i)
    {
      Description.UpperFovLimit = std::max(Description.UpperFovLimit,  Description.VerticleAngles[i]);
      Description.LowerFovLimit = std::min(Description.LowerFovLimit,  Description.VerticleAngles[i]);

      VReso = std::min(VReso, carla::geom::Math::ToRadians(std::abs(Description.VerticleAngles[i] - Description.VerticleAngles[i - 1])));

      Elevations[i] = carla::geom::Math::ToRadians(Description.VerticleAngles[i]);
    }
  }
  
  // Original lidar verticle fov
  float lidar_vfov = carla::geom::Math::ToRadians(std::max(std::abs(Description.UpperFovLimit), std::abs(Description.LowerFovLimit))*2 + 2.0);

  // Enlarge the verticle fov cause the edge of image has smaller verticle fov
  VFov = 2.0 * atan( tan(lidar_vfov / 2.0) / cos(HFov / 2.0) );

  // rotation rate in rad/s
  RotationRate = Description.RotationFrequency * carla::geom::Math::Pi2<float>();

  // Horizon lidar resolution in rad
  HReso = RotationRate / Description.PointsPerSecond;
  VReso = HReso; // force verticle reso to be same
}

// Calculate the camera projection matrix
void ADepthLidar::CalcProjection()
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

void ADepthLidar::CalcTextureSize()
{
  // Upsample 4 times to mitigate aliasing
  TextureSize.X = 2  * static_cast<int>(HFov / HReso);
  TextureSize.Y = 2 *  static_cast<int>(VFov / VReso);
}

void ADepthLidar::RemoveOtherPostProcessingEffect(FEngineShowFlags &ShowFlags)
{
  ShowFlags.SetAmbientOcclusion(false);
  ShowFlags.SetAntiAliasing(false);
  ShowFlags.SetAtmosphericFog(false);
  // ShowFlags.SetAudioRadius(false);
  // ShowFlags.SetBillboardSprites(false);
  ShowFlags.SetBloom(false);
  // ShowFlags.SetBounds(false);
  // ShowFlags.SetBrushes(false);
  // ShowFlags.SetBSP(false);
  // ShowFlags.SetBSPSplit(false);
  // ShowFlags.SetBSPTriangles(false);
  // ShowFlags.SetBuilderBrush(false);
  // ShowFlags.SetCameraAspectRatioBars(false);
  // ShowFlags.SetCameraFrustums(false);
  ShowFlags.SetCameraImperfections(false);
  ShowFlags.SetCameraInterpolation(false);
  // ShowFlags.SetCameraSafeFrames(false);
  // ShowFlags.SetCollision(false);
  // ShowFlags.SetCollisionPawn(false);
  // ShowFlags.SetCollisionVisibility(false);
  ShowFlags.SetColorGrading(false);
  // ShowFlags.SetCompositeEditorPrimitives(false);
  // ShowFlags.SetConstraints(false);
  // ShowFlags.SetCover(false);
  // ShowFlags.SetDebugAI(false);
  // ShowFlags.SetDecals(false);
  // ShowFlags.SetDeferredLighting(false);
  ShowFlags.SetDepthOfField(false);
  ShowFlags.SetDiffuse(false);
  ShowFlags.SetDirectionalLights(false);
  ShowFlags.SetDirectLighting(false);
  // ShowFlags.SetDistanceCulledPrimitives(false);
  // ShowFlags.SetDistanceFieldAO(false);
  // ShowFlags.SetDistanceFieldGI(false);
  ShowFlags.SetDynamicShadows(false);
  // ShowFlags.SetEditor(false);
  ShowFlags.SetEyeAdaptation(false);
  ShowFlags.SetFog(false);
  // ShowFlags.SetGame(false);
  // ShowFlags.SetGameplayDebug(false);
  // ShowFlags.SetGBufferHints(false);
  ShowFlags.SetGlobalIllumination(false);
  ShowFlags.SetGrain(false);
  // ShowFlags.SetGrid(false);
  // ShowFlags.SetHighResScreenshotMask(false);
  // ShowFlags.SetHitProxies(false);
  ShowFlags.SetHLODColoration(false);
  ShowFlags.SetHMDDistortion(false);
  // ShowFlags.SetIndirectLightingCache(false);
  // ShowFlags.SetInstancedFoliage(false);
  // ShowFlags.SetInstancedGrass(false);
  // ShowFlags.SetInstancedStaticMeshes(false);
  // ShowFlags.SetLandscape(false);
  // ShowFlags.SetLargeVertices(false);
  ShowFlags.SetLensFlares(false);
  ShowFlags.SetLevelColoration(false);
  ShowFlags.SetLightComplexity(false);
  ShowFlags.SetLightFunctions(false);
  ShowFlags.SetLightInfluences(false);
  ShowFlags.SetLighting(false);
  ShowFlags.SetLightMapDensity(false);
  ShowFlags.SetLightRadius(false);
  ShowFlags.SetLightShafts(false);
  // ShowFlags.SetLOD(false);
  ShowFlags.SetLODColoration(false);
  // ShowFlags.SetMaterials(false);
  // ShowFlags.SetMaterialTextureScaleAccuracy(false);
  // ShowFlags.SetMeshEdges(false);
  // ShowFlags.SetMeshUVDensityAccuracy(false);
  // ShowFlags.SetModeWidgets(false);
  ShowFlags.SetMotionBlur(false);
  // ShowFlags.SetNavigation(false);
  ShowFlags.SetOnScreenDebug(false);
  // ShowFlags.SetOutputMaterialTextureScales(false);
  // ShowFlags.SetOverrideDiffuseAndSpecular(false);
  // ShowFlags.SetPaper2DSprites(false);
  ShowFlags.SetParticles(false);
  // ShowFlags.SetPivot(false);
  ShowFlags.SetPointLights(false);
  // ShowFlags.SetPostProcessing(false);
  // ShowFlags.SetPostProcessMaterial(false);
  // ShowFlags.SetPrecomputedVisibility(false);
  // ShowFlags.SetPrecomputedVisibilityCells(false);
  // ShowFlags.SetPreviewShadowsIndicator(false);
  // ShowFlags.SetPrimitiveDistanceAccuracy(false);
  ShowFlags.SetPropertyColoration(false);
  // ShowFlags.SetQuadOverdraw(false);
  // ShowFlags.SetReflectionEnvironment(false);
  // ShowFlags.SetReflectionOverride(false);
  ShowFlags.SetRefraction(false);
  // ShowFlags.SetRendering(false);
  ShowFlags.SetSceneColorFringe(false);
  // ShowFlags.SetScreenPercentage(false);
  ShowFlags.SetScreenSpaceAO(false);
  ShowFlags.SetScreenSpaceReflections(false);
  // ShowFlags.SetSelection(false);
  // ShowFlags.SetSelectionOutline(false);
  // ShowFlags.SetSeparateTranslucency(false);
  // ShowFlags.SetShaderComplexity(false);
  // ShowFlags.SetShaderComplexityWithQuadOverdraw(false);
  // ShowFlags.SetShadowFrustums(false);
  // ShowFlags.SetSkeletalMeshes(false);
  // ShowFlags.SetSkinCache(false);
  ShowFlags.SetSkyLighting(false);
  // ShowFlags.SetSnap(false);
  // ShowFlags.SetSpecular(false);
  // ShowFlags.SetSplines(false);
  ShowFlags.SetSpotLights(false);
  // ShowFlags.SetStaticMeshes(false);
  ShowFlags.SetStationaryLightOverlap(false);
  // ShowFlags.SetStereoRendering(false);
  // ShowFlags.SetStreamingBounds(false);
  ShowFlags.SetSubsurfaceScattering(false);
  // ShowFlags.SetTemporalAA(false);
  // ShowFlags.SetTessellation(false);
  // ShowFlags.SetTestImage(false);
  // ShowFlags.SetTextRender(false);
  // ShowFlags.SetTexturedLightProfiles(false);
  ShowFlags.SetTonemapper(false);
  // ShowFlags.SetTranslucency(false);
  // ShowFlags.SetVectorFields(false);
  // ShowFlags.SetVertexColors(false);
  // ShowFlags.SetVignette(false);
  // ShowFlags.SetVisLog(false);
  ShowFlags.SetVisualizeAdaptiveDOF(false);
  ShowFlags.SetVisualizeBloom(false);
  ShowFlags.SetVisualizeBuffer(false);
  ShowFlags.SetVisualizeDistanceFieldAO(false);
  ShowFlags.SetVisualizeDistanceFieldGI(false);
  ShowFlags.SetVisualizeDOF(false);
  ShowFlags.SetVisualizeHDR(false);
  ShowFlags.SetVisualizeLightCulling(false);
  ShowFlags.SetVisualizeLPV(false);
  ShowFlags.SetVisualizeMeshDistanceFields(false);
  ShowFlags.SetVisualizeMotionBlur(false);
  ShowFlags.SetVisualizeOutOfBoundsPixels(false);
  ShowFlags.SetVisualizeSenses(false);
  ShowFlags.SetVisualizeShadingModels(false);
  ShowFlags.SetVisualizeSSR(false);
  ShowFlags.SetVisualizeSSS(false);
  // ShowFlags.SetVolumeLightingSamples(false);
  // ShowFlags.SetVolumes(false);
  // ShowFlags.SetWidgetComponents(false);
  // ShowFlags.SetWireframe(false);
}

void ADepthLidar::PutRenderTarget(const FRenderTargetPtr &TextureTarget) const{
  if (RenderTargetPool.IsValid())
  {
    RenderTargetPool->Put(TextureTarget);
  }
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
  if (Got==nullptr)
  {
    //Todo: make clear what this config means
    Got = NewObject<UTextureRenderTarget2D>();
    Got->AddToRoot();
    Total.push(Got);

    Got->CompressionSettings = TextureCompressionSettings::TC_Default;
    Got->SRGB = false;
    Got->bHDR_DEPRECATED = false;
    Got->bAutoGenerateMips = false;
    Got->bGPUSharedFlag = true;
    Got->AddressX = TextureAddress::TA_Clamp;
    Got->AddressY = TextureAddress::TA_Clamp;
    Got->InitCustomFormat(Size.X, Size.Y, PF_B8G8R8A8, true);
  }

  return Got;
}

void FRenderTargetPool::Put(const FRenderTargetPtr &RenderTarget)
{
  Lock.lock();
  Avialables.push(RenderTarget);
  Lock.unlock();
}

FRenderTargetPool::~FRenderTargetPool(){
  while(!Total.empty()){
    Total.top()->ReleaseResource();
    Total.top()->RemoveFromRoot();
    Total.pop();
  }
}
