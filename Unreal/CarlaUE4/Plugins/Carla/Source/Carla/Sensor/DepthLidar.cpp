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

  // Bind the depth post process material with capture component
  if (Loader.Succeeded())
  {
    CaptureComponent2D->PostProcessSettings.AddBlendable(UMaterialInstanceDynamic::Create(Loader.Object, this), 1.0);
  }

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
  // Deactivate capture component, capture manully
  CaptureComponent2D->Deactivate();

  // LDR is faster
  CaptureComponent2D->CaptureSource = ESceneCaptureSource::SCS_FinalColorLDR;

  // Remove other post process effect
  RemoveOtherPostProcessingEffect(CaptureComponent2D->ShowFlags);

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
    CaptureComponent2D->TextureTarget = TextureTarget;

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
  LastOrientation = (CurrentOrientation >= carla::geom::Math::Pi2<float>()) 
                    ? CurrentOrientation - carla::geom::Math::Pi2<float>() 
                    : CurrentOrientation;
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
      float Depth = *(reinterpret_cast<float*>(Buffer.data() + 4*(V*TextureSize.X + U)));

      // Polar coordinates to cartisian coordinates
      carla::rpc::Location Point;
      Point.x = std::cos(RayOrientation)*Depth;
      Point.y = std::sin(RayOrientation)*Depth;
      Point.z = std::tan(RayPitch)*Depth;

      Stream.Send(*this, LidarMeasurement, Stream.PopBufferFromPool());
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