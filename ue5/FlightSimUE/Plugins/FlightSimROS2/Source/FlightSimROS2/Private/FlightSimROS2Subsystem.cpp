// FlightSimROS2Subsystem.cpp
// UE5 side only uses the plain C API from fsros2_bridge.h.
// No rclcpp headers, no typeid, no dynamic_cast — no RTTI required.

#include "FlightSimROS2Subsystem.h"
#include "FlightSimLondonMapActor.h"
#include "Ros2Bridge.h"

#include "Engine/World.h"
#include "Engine/DirectionalLight.h"
#include "Engine/SkyLight.h"
#include "Camera/CameraActor.h"
#include "Components/DirectionalLightComponent.h"
#include "Components/SkyLightComponent.h"
#include "GameFramework/PlayerController.h"
#include "TextureResource.h"

// ---------------------------------------------------------------------------
// Coordinate helpers  (NED → UE5)
// NED : X=North Y=East  Z=Down  (right-handed, metres)
// UE5 : X=Forward Y=Right Z=Up  (left-handed, centimetres)
// ---------------------------------------------------------------------------

FVector UFlightSimROS2Subsystem::NedToUE5Pos(const float (&NedM)[3])
{
    return FVector(
         static_cast<double>(NedM[0]) * 100.0,
         static_cast<double>(NedM[1]) * 100.0,
        -static_cast<double>(NedM[2]) * 100.0);
}

FQuat UFlightSimROS2Subsystem::NedToUE5Rot(const float (&WxyzNed)[4])
{
    return FQuat(WxyzNed[1], WxyzNed[2], -WxyzNed[3], WxyzNed[0]);
}

// ---------------------------------------------------------------------------
// Callbacks — all called from the wrapper's spin thread.
// Only write to TQueue or TAtomic here; game-thread reads happen in Tick().
// ---------------------------------------------------------------------------

static void OnSceneState(const FsRos2Entity* entities,
                         int                 count,
                         uint64_t            sim_step,
                         void*               userdata)
{
    UFlightSimROS2Subsystem* self = static_cast<UFlightSimROS2Subsystem*>(userdata);
    if (!self) return;

    self->Telemetry.SceneFeedActive = true;
    self->Telemetry.SimStep         = static_cast<int64>(sim_step);

    for (int i = 0; i < count; ++i)
    {
        // Populate target position from scene entity so the HUD can read it
        // without needing a separate TargetState subscription.
        if (FCStringAnsi::Strnicmp(entities[i].type, "target", 6) == 0)
        {
            self->Telemetry.TargetPosNED = FVector(
                entities[i].position_ned_m[0],
                entities[i].position_ned_m[1],
                entities[i].position_ned_m[2]);
        }

        FEntitySnapshot Snap;
        Snap.Id         = UTF8_TO_TCHAR(entities[i].id);
        Snap.Type       = UTF8_TO_TCHAR(entities[i].type);
        Snap.LocationCm = UFlightSimROS2Subsystem::NedToUE5Pos(entities[i].position_ned_m);
        Snap.Rotation   = UFlightSimROS2Subsystem::NedToUE5Rot(entities[i].attitude_wxyz);
        Snap.Active     = true;
        self->EntityQueue.Enqueue(Snap);
    }
}

static void OnMissileState(const FsRos2MissileState* s, void* userdata)
{
    UFlightSimROS2Subsystem* self = static_cast<UFlightSimROS2Subsystem*>(userdata);
    if (!self || !s) return;

    FFlightSimTelemetry& T = self->Telemetry;
    T.MissilePosNED   = FVector(s->position_ned_m[0], s->position_ned_m[1], s->position_ned_m[2]);
    T.MissileVelNED   = FVector(s->velocity_ned_mps[0], s->velocity_ned_mps[1], s->velocity_ned_mps[2]);
    T.MissileSpeedMps = T.MissileVelNED.Size();
    T.MissileThrustN  = s->thrust_n;
    T.MissileLengthM  = s->length_m;
    T.MissileFinPitchRad = s->fin_pitch_rad;
    T.MissileFinYawRad   = s->fin_yaw_rad;
    T.SeekerRangeM    = s->seeker_range_m;
    T.MissileActive   = (s->active != 0);
    T.MissileHit      = (s->hit    != 0);
    T.SeekerLocked    = (s->seeker_locked != 0);
    T.RangeM          = s->seeker_range_m; // best proxy until EngagementStatus arrives
}

static void OnEngagementStatus(const FsRos2EngagementStatus* s, void* userdata)
{
    UFlightSimROS2Subsystem* self = static_cast<UFlightSimROS2Subsystem*>(userdata);
    if (!self || !s) return;

    FFlightSimTelemetry& T = self->Telemetry;
    T.RangeM        = s->range_m;
    T.MissDistanceM = s->miss_distance_m;
    T.StepCount     = static_cast<int64>(s->step_count);
    T.Intercept     = (s->intercept != 0);
    T.Complete      = (s->complete  != 0);
}

// ---------------------------------------------------------------------------
// Initialize
// ---------------------------------------------------------------------------

void UFlightSimROS2Subsystem::Initialize(FSubsystemCollectionBase& Collection)
{
    Super::Initialize(Collection);

    Bridge = fsros2_create_bridge("flightsim_ue5_node");
    if (!Bridge)
    {
        UE_LOG(LogTemp, Error,
               TEXT("FlightSimROS2Subsystem: fsros2_create_bridge failed. "
                    "Is ROS 2 running? Check LD_LIBRARY_PATH."));
        return;
    }

    fsros2_set_scene_state_cb(Bridge, OnSceneState, this);
    fsros2_set_missile_state_cb(Bridge, OnMissileState, this);
    fsros2_set_engagement_status_cb(Bridge, OnEngagementStatus, this);

    EnsureMonitoringScene();

    UE_LOG(LogTemp, Log,
           TEXT("FlightSimROS2Subsystem: node 'flightsim_ue5_node' started"));
}

// ---------------------------------------------------------------------------
// Deinitialize
// ---------------------------------------------------------------------------

void UFlightSimROS2Subsystem::Deinitialize()
{
    fsros2_destroy_bridge(Bridge);
    Bridge = nullptr;
    Super::Deinitialize();
}

// ---------------------------------------------------------------------------
// Tick (game thread)
// ---------------------------------------------------------------------------

void UFlightSimROS2Subsystem::Tick(float /*DeltaTime*/)
{
    EnsureMonitoringScene();

    FEntitySnapshot Snap;
    while (EntityQueue.Dequeue(Snap))
    {
        SpawnOrUpdateActor(Snap);
    }

    UpdateMonitoringCamera();

    if (MissileActor && MissileActor->SeekerRenderTarget)
    {
        PublishSeekerImage();
    }
}

TStatId UFlightSimROS2Subsystem::GetStatId() const
{
    RETURN_QUICK_DECLARE_CYCLE_STAT(UFlightSimROS2Subsystem, STATGROUP_Tickables);
}

void UFlightSimROS2Subsystem::EnsureMonitoringScene()
{
    if (bMonitoringSceneReady)
    {
        return;
    }

    UWorld* World = GetWorld();
    if (!World)
    {
        return;
    }

    FActorSpawnParameters Params;
    Params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;

    if (!LondonMap)
    {
        LondonMap = World->SpawnActor<AFlightSimLondonMapActor>(
            AFlightSimLondonMapActor::StaticClass(),
            FTransform(FRotator::ZeroRotator, FVector::ZeroVector),
            Params);
        if (LondonMap)
        {
            LondonMap->SetActorLabel(TEXT("FlightSim London Map"));
            LondonMap->bLoadOnBeginPlay = false;
            LondonMap->LoadLondonObj();
        }
    }

    if (!SunLight)
    {
        SunLight = World->SpawnActor<ADirectionalLight>(
            ADirectionalLight::StaticClass(),
            FVector(-30000.0, -30000.0, 80000.0),
            FRotator(-45.0, -35.0, 0.0),
            Params);
        if (SunLight)
        {
            SunLight->SetActorLabel(TEXT("FlightSim Sun Light"));
            SunLight->GetLightComponent()->SetIntensity(8.0F);
        }
    }

    if (!SkyLight)
    {
        SkyLight = World->SpawnActor<ASkyLight>(
            ASkyLight::StaticClass(),
            FVector::ZeroVector,
            FRotator::ZeroRotator,
            Params);
        if (SkyLight)
        {
            SkyLight->SetActorLabel(TEXT("FlightSim Sky Light"));
            SkyLight->GetLightComponent()->SetIntensity(1.5F);
        }
    }

    if (!MonitoringCamera)
    {
        MonitoringCamera = World->SpawnActor<ACameraActor>(
            ACameraActor::StaticClass(),
            FVector(-60000.0, -60000.0, 45000.0),
            FRotator(-35.0, 45.0, 0.0),
            Params);
        if (MonitoringCamera)
        {
            MonitoringCamera->SetActorLabel(TEXT("FlightSim Monitoring Camera"));
        }
    }

    if (MonitoringCamera)
    {
        for (FConstPlayerControllerIterator It = World->GetPlayerControllerIterator(); It; ++It)
        {
            if (APlayerController* PC = It->Get())
            {
                PC->SetViewTarget(MonitoringCamera);
            }
        }
    }

    bMonitoringSceneReady = true;
    UE_LOG(LogTemp, Log, TEXT("FlightSimROS2: London map monitoring scene ready"));
}

void UFlightSimROS2Subsystem::UpdateMonitoringCamera()
{
    if (!MonitoringCamera)
    {
        return;
    }

    FVector Center = FVector::ZeroVector;
    double Radius = 40000.0;

    if (EntityActors.Num() > 0)
    {
        FBox Bounds(ForceInit);
        for (const TPair<FString, AFlightSimSceneActor*>& Pair : EntityActors)
        {
            if (Pair.Value)
            {
                Bounds += Pair.Value->GetActorLocation();
            }
        }

        if (Bounds.IsValid)
        {
            Center = Bounds.GetCenter();
            Radius = FMath::Max(40000.0, Bounds.GetExtent().Size() * 2.5);
        }
    }

    const FVector CameraLocation = Center + FVector(-Radius, -Radius, Radius * 0.75);
    const FRotator CameraRotation = FRotationMatrix::MakeFromX(Center - CameraLocation).Rotator();
    MonitoringCamera->SetActorLocationAndRotation(CameraLocation, CameraRotation);

    if (UWorld* World = GetWorld())
    {
        for (FConstPlayerControllerIterator It = World->GetPlayerControllerIterator(); It; ++It)
        {
            if (APlayerController* PC = It->Get())
            {
                if (PC->GetViewTarget() != MonitoringCamera)
                {
                    PC->SetViewTarget(MonitoringCamera);
                }
            }
        }
    }
}

// ---------------------------------------------------------------------------
// SpawnOrUpdateActor
// ---------------------------------------------------------------------------

void UFlightSimROS2Subsystem::SpawnOrUpdateActor(const FEntitySnapshot& Snap)
{
    UWorld* World = GetWorld();
    if (!World) return;

    AFlightSimSceneActor* Actor = nullptr;

    if (AFlightSimSceneActor** Found = EntityActors.Find(Snap.Id))
    {
        Actor = *Found;
    }
    else
    {
        FActorSpawnParameters Params;
        Params.SpawnCollisionHandlingOverride =
            ESpawnActorCollisionHandlingMethod::AlwaysSpawn;

        Actor = World->SpawnActor<AFlightSimSceneActor>(
            AFlightSimSceneActor::StaticClass(),
            FTransform(Snap.Rotation, Snap.LocationCm),
            Params);

        if (!Actor) return;

        Actor->EntityId = Snap.Id;
        EntityActors.Add(Snap.Id, Actor);

        if (Snap.Type.Equals(TEXT("missile"), ESearchCase::IgnoreCase))
        {
            Actor->InitAsMissile(SeekerWidthPx, SeekerHeightPx, SeekerHFovDeg);
            MissileActor = Actor;
            fsros2_publish_camera_info(Bridge, SeekerWidthPx, SeekerHeightPx, SeekerHFovDeg);
            UE_LOG(LogTemp, Log,
                   TEXT("FlightSimROS2: Missile '%s' spawned, seeker %dx%d %.1f° HFoV"),
                   *Snap.Id, SeekerWidthPx, SeekerHeightPx, SeekerHFovDeg);
        }
        else
        {
            const bool bIsTarget = Snap.Type.Equals(TEXT("target"), ESearchCase::IgnoreCase)
                                || Snap.Type.Equals(TEXT("shahed"), ESearchCase::IgnoreCase)
                                || Snap.Type.Equals(TEXT("drone"), ESearchCase::IgnoreCase);
            Actor->SetEntityType(bIsTarget ? EFlightSimEntityType::Target : EFlightSimEntityType::Depot);
            UE_LOG(LogTemp, Log,
                   TEXT("FlightSimROS2: Entity '%s' type='%s' spawned"),
                   *Snap.Id, *Snap.Type);
        }
    }

    Actor->UpdateTransform(Snap.LocationCm, Snap.Rotation);
}

// ---------------------------------------------------------------------------
// PublishSeekerImage
// ---------------------------------------------------------------------------

void UFlightSimROS2Subsystem::PublishSeekerImage()
{
    if (!Bridge) return;

    FTextureRenderTargetResource* RTRes =
        MissileActor->SeekerRenderTarget->GameThread_GetRenderTargetResource();
    if (!RTRes) return;

    TArray<FColor> Pixels;
    Pixels.SetNumUninitialized(SeekerWidthPx * SeekerHeightPx);
    if (!RTRes->ReadPixels(Pixels)) return;

    // Convert BGRA → mono8 (BT.601) in a stack-allocated buffer.
    TArray<uint8> Mono;
    Mono.SetNumUninitialized(Pixels.Num());
    for (int32 i = 0; i < Pixels.Num(); ++i)
    {
        const FColor& C = Pixels[i];
        Mono[i] = static_cast<uint8>((77u*C.R + 150u*C.G + 29u*C.B) >> 8u);
    }

    fsros2_publish_image(Bridge, SeekerWidthPx, SeekerHeightPx, Mono.GetData());
    Telemetry.SeekerCamActive = true;
}
