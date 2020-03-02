// Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma de Barcelona (UAB). 
// This work is licensed under the terms of the MIT license. 
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include <stack>

#include <carla/sensor/s11n/LidarMeasurement.h>

#include "Carla/Sensor/LidarDescription.h"
#include "CoreMinimal.h"
#include "Sensor/Sensor.h"

#include "DepthLidar.generated.h"

class USceneCaptureComponent2D;
class UTextureRenderTarget2D;
class FRenderTargetPool;

/**
 * 
 */
UCLASS()
class CARLA_API ADepthLidar : public ASensor
{
  GENERATED_BODY()

  public:

  ADepthLidar(const FObjectInitializer &ObjectInitializer);

  static FActorDefinition GetSensorDefinition(); 

  void Set(const FActorDescription &ActorDescription) override;

  protected:
  virtual void BeginPlay() override;

  virtual void Tick(float DeltaTime) override;

  virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

  private:
  void CalcResolutionAndCaptureFov();

  void CalcProjection();

  void CalcTextureSize();

  void HandleCaptureOnRenderingThread(SharedPtr<UTextureRenderTarget2D> Target, FAsyncDataStream Stream, FRHICommandListImmediate &InRHICmdList);

  UPROPERTY(EditAnywhere)
  FLidarDescription Description;

  // Store the horizon fov of the capture in rad
  // Fix it at 18 degree at this point
  constexpr float HFov = carla::geom::Math::ToRadians(18.0f);

  // Store the vertical fov of the capture in rad
  float VFov;

  // horizon angular resolution in rad
  float HReso;

  // verticle angular resolution in rad
  float VReso;

  // verticle elevation angle of laser ray
  TArray<float> Elevations;

  // Texture Size in pixels
  int TextureWidth;
  int TextureHeight;
  
  // Orientation of the lidar
  float CurrentOrientation = 0.0f;
  float LastOrientation = 0.0f;

  // Lidar rotation rate in rad/second
  float RotationRate = 0.0f;

  // Scene capture component to capture the depth of the scene
  USceneCaptureComponent2D *CaptureComponent2D = nullptr;

  // Rendering target pool for CaptureComponent to output
  TUniquePtr<FRenderTargetPool> RenderTargetPool = nullptr;
};

/**
 * Manage a pool of avialable TextureRenderTarget
 */
class FRenderTargetPool{
  using  FRenderTargetPtr = TSharedPtr<UTextureRenderTarget2D>;

  public:
  FRenderTargetPtr Get();
  void Put(const FRenderTargetPtr& RenderTarget);

  void SetSize(int width, int height){width_=width;height_=height;}

  private:
  int width_;
  int height_;
  std::mutex lock_;
  std::stack<FRenderTargetPtr> avialables_;
};