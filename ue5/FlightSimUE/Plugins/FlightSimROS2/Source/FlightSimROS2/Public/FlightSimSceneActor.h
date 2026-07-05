#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Components/SceneCaptureComponent2D.h"
#include "Engine/TextureRenderTarget2D.h"
#include "FlightSimSceneActor.generated.h"

UENUM(BlueprintType)
enum class EFlightSimEntityType : uint8
{
    Unknown  UMETA(DisplayName = "Unknown"),
    Missile  UMETA(DisplayName = "Missile"),
    Target   UMETA(DisplayName = "Target"),
    Depot    UMETA(DisplayName = "Depot"),
};

/**
 * Generic actor that represents a FlightSim entity (missile, target, or depot).
 * For the missile entity a SceneCaptureComponent2D is attached to produce the
 * seeker-camera render target that is read back and published as a ROS 2 image.
 */
UCLASS()
class AFlightSimSceneActor : public AActor
{
    GENERATED_BODY()

public:
    AFlightSimSceneActor();

    // Called by the subsystem each tick after the scene-state queue is drained.
    void UpdateTransform(const FVector& LocationCm, const FQuat& Rotation);

    // Seeker camera render target (valid only on Missile entities).
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "FlightSim|Seeker")
    UTextureRenderTarget2D* SeekerRenderTarget = nullptr;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "FlightSim")
    EFlightSimEntityType EntityType = EFlightSimEntityType::Unknown;

    // The entity id string from the ROS 2 message.
    FString EntityId;

    // Called once after spawning so the seeker camera is properly configured.
    void InitAsMissile(int32 CamWidthPx, int32 CamHeightPx, float HFovDeg);

    void SetEntityType(EFlightSimEntityType NewType);

    UPROPERTY(VisibleAnywhere, Category = "FlightSim")
    UStaticMeshComponent* MeshComponent = nullptr;

    UPROPERTY(VisibleAnywhere, Category = "FlightSim|Seeker")
    USceneCaptureComponent2D* SeekerCapture = nullptr;

protected:
    virtual void BeginPlay() override;
};
