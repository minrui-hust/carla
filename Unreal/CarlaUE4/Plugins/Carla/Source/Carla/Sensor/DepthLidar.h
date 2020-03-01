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

  UPROPERTY(EditAnywhere)
  FLidarDescription Description;

  /// Scene capture component to capture the depth of the scene
  USceneCaptureComponent2D *CaptureComponent2D = nullptr;

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

  private:
  std::mutex lock_;
  std::stack<FRenderTargetPtr> avialables_;
};