#pragma once

#include "CoreMinimal.h"
#include "Subsystems/WorldSubsystem.h"
#include "Containers/Queue.h"
#include "FlightSimSceneActor.h"
#include "FlightSimROS2Subsystem.generated.h"

class ACameraActor;
class ADirectionalLight;
class AFlightSimLondonMapActor;
class ASkyLight;

// ---------------------------------------------------------------------------
// Telemetry snapshot — written by the ROS2 spin thread, read by the game
// thread (HUD, camera director, Blueprint).  Uses TAtomic where single-field
// reads need to be race-free; compound reads should happen only on game thread
// after EntityQueue is drained.
// ---------------------------------------------------------------------------
USTRUCT(BlueprintType)
struct FFlightSimTelemetry
{
    GENERATED_BODY()

    // ── Missile ──────────────────────────────────────────────────────────
    UPROPERTY(BlueprintReadOnly, Category = "FlightSim|Missile")
    FVector MissilePosNED  = FVector::ZeroVector;   // metres

    UPROPERTY(BlueprintReadOnly, Category = "FlightSim|Missile")
    FVector MissileVelNED  = FVector::ZeroVector;   // m/s

    UPROPERTY(BlueprintReadOnly, Category = "FlightSim|Missile")
    float   MissileSpeedMps = 0.f;

    UPROPERTY(BlueprintReadOnly, Category = "FlightSim|Missile")
    float   MissileThrustN  = 0.f;

    UPROPERTY(BlueprintReadOnly, Category = "FlightSim|Missile")
    float   MissileLengthM  = 0.f;

    UPROPERTY(BlueprintReadOnly, Category = "FlightSim|Missile")
    float   MissileFinPitchRad = 0.f;

    UPROPERTY(BlueprintReadOnly, Category = "FlightSim|Missile")
    float   MissileFinYawRad   = 0.f;

    UPROPERTY(BlueprintReadOnly, Category = "FlightSim|Missile")
    bool    MissileActive   = false;

    UPROPERTY(BlueprintReadOnly, Category = "FlightSim|Missile")
    bool    MissileHit      = false;

    UPROPERTY(BlueprintReadOnly, Category = "FlightSim|Missile")
    bool    SeekerLocked    = false;

    UPROPERTY(BlueprintReadOnly, Category = "FlightSim|Missile")
    float   SeekerRangeM    = 0.f;

    // ── Target (Shahed) ───────────────────────────────────────────────────
    UPROPERTY(BlueprintReadOnly, Category = "FlightSim|Target")
    FVector TargetPosNED   = FVector::ZeroVector;

    UPROPERTY(BlueprintReadOnly, Category = "FlightSim|Target")
    FVector TargetVelNED   = FVector::ZeroVector;

    UPROPERTY(BlueprintReadOnly, Category = "FlightSim|Target")
    float   TargetSpeedMps = 0.f;

    // ── Engagement ────────────────────────────────────────────────────────
    UPROPERTY(BlueprintReadOnly, Category = "FlightSim|Engagement")
    float    RangeM         = 0.f;

    UPROPERTY(BlueprintReadOnly, Category = "FlightSim|Engagement")
    float    MissDistanceM  = 0.f;

    UPROPERTY(BlueprintReadOnly, Category = "FlightSim|Engagement")
    bool     Intercept      = false;

    UPROPERTY(BlueprintReadOnly, Category = "FlightSim|Engagement")
    bool     Complete       = false;

    UPROPERTY(BlueprintReadOnly, Category = "FlightSim|Engagement")
    int64    StepCount      = 0;

    // ── Bridge health ─────────────────────────────────────────────────────
    UPROPERTY(BlueprintReadOnly, Category = "FlightSim|Bridge")
    bool     SceneFeedActive  = false;

    UPROPERTY(BlueprintReadOnly, Category = "FlightSim|Bridge")
    bool     SeekerCamActive  = false;

    UPROPERTY(BlueprintReadOnly, Category = "FlightSim|Bridge")
    int64    SimStep          = 0;
};

// ---------------------------------------------------------------------------
// Plain-old-data entity snapshot — pushed from ROS 2 spin thread to game thread.
// ---------------------------------------------------------------------------
struct FEntitySnapshot
{
    FString Id;
    FString Type;       // "missile" | "target" | "depot"
    FVector LocationCm;
    FQuat   Rotation;
    bool    Active = true;
};

// ---------------------------------------------------------------------------
// UWorldSubsystem — alive for the duration of the world.
// The HUD, camera director, and Blueprint nodes can call
//   GetWorld()->GetSubsystem<UFlightSimROS2Subsystem>()
// to read Telemetry or access the missile actor directly.
// ---------------------------------------------------------------------------
UCLASS()
class UFlightSimROS2Subsystem : public UTickableWorldSubsystem
{
    GENERATED_BODY()

public:
    virtual void    Initialize(FSubsystemCollectionBase& Collection) override;
    virtual void    Deinitialize() override;
    virtual void    Tick(float DeltaTime) override;
    virtual TStatId GetStatId() const override;

    // ── Live telemetry (game-thread safe after Tick drains the queue) ─────
    UPROPERTY(BlueprintReadOnly, Category = "FlightSim")
    FFlightSimTelemetry Telemetry;

    // ── Scene actors (set when first entity message arrives) ─────────────
    UPROPERTY(BlueprintReadOnly, Category = "FlightSim")
    AFlightSimSceneActor* MissileActor = nullptr;

    // ── Seeker camera parameters ──────────────────────────────────────────
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FlightSim|Seeker")
    int32 SeekerWidthPx  = 320;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FlightSim|Seeker")
    int32 SeekerHeightPx = 240;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FlightSim|Seeker")
    float SeekerHFovDeg  = 6.0f;

    // Called by the static C callback — must be public.
    TQueue<FEntitySnapshot, EQueueMode::Spsc> EntityQueue;

    static FVector NedToUE5Pos(const float (&NedM)[3]);
    static FQuat   NedToUE5Rot(const float (&WxyzNed)[4]);

private:
    // Opaque handle returned by the C wrapper library.
    void* Bridge = nullptr;

    UPROPERTY()
    TMap<FString, AFlightSimSceneActor*> EntityActors;

    UPROPERTY()
    ACameraActor* MonitoringCamera = nullptr;

    UPROPERTY()
    AFlightSimLondonMapActor* LondonMap = nullptr;

    UPROPERTY()
    ADirectionalLight* SunLight = nullptr;

    UPROPERTY()
    ASkyLight* SkyLight = nullptr;

    bool bMonitoringSceneReady = false;

    void SpawnOrUpdateActor(const FEntitySnapshot& Snap);
    void PublishSeekerImage();
    void EnsureMonitoringScene();
    void UpdateMonitoringCamera();
};
