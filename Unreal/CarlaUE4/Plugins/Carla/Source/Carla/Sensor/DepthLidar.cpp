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
  PrimaryActorTick.bCanEverTick = true;
  PrimaryActorTick.TickGroup = TG_PrePhysics;

  CaptureComponent2D = CreateDefaultSubobject<USceneCaptureComponent2D>(
      FName(*FString::Printf(TEXT("SceneCaptureComponent2D"))));
  CaptureComponent2D->SetupAttachment(RootComponent);
}

void ADepthLidar::Set(const FActorDescription &ActorDescription)
{
  Super::Set(ActorDescription);
  UActorBlueprintFunctionLibrary::SetLidar(ActorDescription, Description);
}

void ADepthLidar::BeginPlay()
{
}

void ADepthLidar::Tick(float DeltaTime)
{
  Super::Tick(DeltaTime);
}

void EndPlay(const EEndPlayReason::Type EndPlayReason)
{
}

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
  }

  return got;
}

void FRenderTargetPool::Put(const FRenderTargetPool::FRenderTargetPtr &RenderTarget)
{
  lock_.lock();
  avialables_.push(RenderTarget);
  lock_.unlock();
}