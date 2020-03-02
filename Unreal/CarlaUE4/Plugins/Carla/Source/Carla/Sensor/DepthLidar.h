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

using  FRenderTargetPtr = TSharedPtr<UTextureRenderTarget2D>;

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

  void HandleCaptureOnRenderingThread(FRenderTargetPtr Target, FAsyncDataStream& Stream, FRHICommandListImmediate &InRHICmdList) const;

  private:
  void CalcResolutionAndCaptureFov();

  void CalcProjection();

  void CalcTextureSize();

  // This function called on rendering thread, pass arguments by value.
  // Make this function const, so that it would not change the state of itself.  RenderTargetPool is an exception

  UPROPERTY(EditAnywhere)
  FLidarDescription Description;

  // Store the horizon fov of the capture in rad
  // Fix it at 18 degree at this point
  const float HFov = carla::geom::Math::ToRadians(18.0f);

  // Store the vertical fov of the capture in rad
  float VFov;

  // Horizon angular resolution in rad
  float HReso;

  // Verticle angular resolution in rad
  float VReso;

  // Verticle elevation angle of laser ray
  TArray<float> Elevations;

  // Texture Size in pixels
	FIntPoint TextureSize;
  
  // Orientation of the lidar
  float CurrentOrientation = 0.0f;
  float LastOrientation = 0.0f;

  // Lidar rotation rate in rad/second
  float RotationRate = 0.0f;

  // Scene capture component to capture the depth of the scene
  USceneCaptureComponent2D *CaptureComponent2D = nullptr;

  // Rendering target pool for CaptureComponent to output
  // Declare it to be mutable, so can be changed even in const member function
  mutable TUniquePtr<FRenderTargetPool> RenderTargetPool = nullptr;
};

/**
 * Manage a pool of avialable TextureRenderTarget
 */
class FRenderTargetPool{
  public:
  FRenderTargetPtr Get();
  void Put(const FRenderTargetPtr& RenderTarget);
  void SetSize(const FIntPoint& size);

  private:
  int Width;
  int Height;
  std::mutex Lock;
  std::stack<FRenderTargetPtr> Avialables;
};