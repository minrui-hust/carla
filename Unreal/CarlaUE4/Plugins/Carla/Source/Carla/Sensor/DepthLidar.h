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


  private:
  void CalcResolutionAndCaptureFov();

  void CalcProjection();

  void CalcTextureSize();

  void RemoveOtherPostProcessingEffect(FEngineShowFlags &ShowFlags);

  struct FCaptureInfo{
    float CaptureCenterOrientation = 0.0f;
    float RayStartOrientation = 0.0f;
    int RayNumber = 0;
  };

  // This function called on rendering thread, pass arguments by value.
  // Make this function const, so that it would not change the state of itself.  RenderTargetPool is an exception
  void HandleCaptureOnRenderingThread(FCaptureInfo CaptureInfo,
                                      FRenderTargetPtr Target, 
                                      FAsyncDataStream& Stream, 
                                      FRHICommandListImmediate &InRHICmdList) const;

  // Capture every 18deg in horizon, with Fov 20deg
  const float HFov = carla::geom::Math::ToRadians(20.0f);
  const float HStep = carla::geom::Math::ToRadians(18.0f);

  // Store the vertical fov of the capture in rad.
  float VFov;

  // Horizon angular resolution in rad.
  float HReso;

  // Verticle angular resolution in rad.
  float VReso;

  // Verticle elevation angle of laser ray.
  TArray<float> Elevations;

  // Lidar rotation rate in rad/second
  float RotationRate = 0.0f;

  // Texture Size in pixels, calculated based on lidar angular resolution
	FIntPoint TextureSize;
  
  // Orientation of the lidar, erea between LastOrientation and CurrentOrientation
  // is going to be captured on this tick
  float LastOrientation = 0.0f;
  float CurrentOrientation = 0.0f;

  // The laser ray going to be processed in one capture
  float RayStartOrientation = 0.0f;
  float RayEndOrientation = 0.0f;

  UPROPERTY(EditAnywhere)
  FLidarDescription Description;
  
  UPROPERTY()
  UMaterial* DepthMaterial = nullptr;

  // Scene capture component to capture the depth of the scene
  UPROPERTY()
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
  FRenderTargetPool(const FIntPoint& Size):Size(Size){
    Lock.unlock();
  }

  FRenderTargetPtr Get();
  void Put(const FRenderTargetPtr& RenderTarget);

  private:
  FIntPoint Size;
  std::mutex Lock;
  std::stack<FRenderTargetPtr> Avialables;
};