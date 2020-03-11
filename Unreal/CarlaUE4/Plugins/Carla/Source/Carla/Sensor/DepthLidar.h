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

using  FRenderTargetPtr = UTextureRenderTarget2D*;

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

  void PutRenderTarget(const FRenderTargetPtr& TextureTarget) const;

  struct FCaptureInfo
  {
    float CaptureCenterOrientation = 0.0f;
    float RayStartOrientation = 0.0f;
    int RayNumber = 0;
    bool Empty = true;
  };
  void SendPixelsOnOtherThread(TArray<FColor> Pixels, FCaptureInfo CaptureInfo, std::shared_ptr<FAsyncDataStream> Stream) const;

  protected:
  virtual void BeginPlay() override;

  virtual void Tick(float DeltaTime) override;

  virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;


  private:
  void CalcResolutionAndCaptureFov();

  void SetProjectionMatrix();

  int SetHFov(float ScanFov);

  void SetTextureSize();

  // The max allowed capture fov, default to be 90 degree
  const float MaxHStep = carla::geom::Math::ToRadians(90.0);

  // Capture fov in horizon
  float HStep;

  // Store the horizontal fov of the capture in rad.
  float HFov;

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

  float MaxDepth = 0.0f;

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
class FRenderTargetPool
{
public:
  FRenderTargetPool()
  {
  }

  ~FRenderTargetPool();

  FRenderTargetPtr Get(const FIntPoint &Size);
  void Put(const FRenderTargetPtr &RenderTarget);

private:
  FCriticalSection CS;
  std::stack<FRenderTargetPtr> Avialables;
  std::stack<FRenderTargetPtr> Total;
};