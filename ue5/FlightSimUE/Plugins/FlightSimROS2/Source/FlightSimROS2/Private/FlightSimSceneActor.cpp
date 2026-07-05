#include "FlightSimSceneActor.h"
#include "Engine/StaticMesh.h"
#include "UObject/ConstructorHelpers.h"

AFlightSimSceneActor::AFlightSimSceneActor()
{
    PrimaryActorTick.bCanEverTick = false;

    MeshComponent = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("Mesh"));
    SetRootComponent(MeshComponent);
    MeshComponent->SetCollisionEnabled(ECollisionEnabled::NoCollision);
    MeshComponent->SetMobility(EComponentMobility::Movable);
}

void AFlightSimSceneActor::BeginPlay()
{
    Super::BeginPlay();
}

void AFlightSimSceneActor::InitAsMissile(int32 CamWidthPx, int32 CamHeightPx, float HFovDeg)
{
    SetEntityType(EFlightSimEntityType::Missile);

    // Render target: BGRA8 so ReadPixels works without format conversion.
    SeekerRenderTarget = NewObject<UTextureRenderTarget2D>(this, TEXT("SeekerRT"));
    SeekerRenderTarget->InitCustomFormat(CamWidthPx, CamHeightPx,
                                         PF_B8G8R8A8, /*bInForceLinearGamma=*/false);
    SeekerRenderTarget->RenderTargetFormat = RTF_RGBA8;
    SeekerRenderTarget->UpdateResourceImmediate(true);

    // Capture component — points along actor's +X axis (forward in UE5).
    SeekerCapture = NewObject<USceneCaptureComponent2D>(this, TEXT("SeekerCapture"));
    SeekerCapture->SetupAttachment(GetRootComponent());
    SeekerCapture->RegisterComponent();
    SeekerCapture->TextureTarget          = SeekerRenderTarget;
    SeekerCapture->CaptureSource         = SCS_FinalColorLDR;
    SeekerCapture->FOVAngle              = HFovDeg;
    SeekerCapture->bCaptureEveryFrame    = true;
    SeekerCapture->bCaptureOnMovement    = false;
    SeekerCapture->ShowFlags.SetAtmosphere(false);
    SeekerCapture->ShowFlags.SetFog(false);
    SeekerCapture->ShowFlags.SetBloom(false);
    SeekerCapture->ShowFlags.SetAntiAliasing(false);
}

void AFlightSimSceneActor::SetEntityType(EFlightSimEntityType NewType)
{
    EntityType = NewType;

    const TCHAR* MeshPath = TEXT("/Engine/BasicShapes/Sphere.Sphere");
    FVector Scale(2.0, 2.0, 2.0);

    if (EntityType == EFlightSimEntityType::Missile)
    {
        MeshPath = TEXT("/Engine/BasicShapes/Cone.Cone");
        Scale = FVector(2.5, 2.5, 7.0);
    }
    else if (EntityType == EFlightSimEntityType::Target)
    {
        MeshPath = TEXT("/Engine/BasicShapes/Sphere.Sphere");
        Scale = FVector(4.0, 4.0, 2.0);
    }
    else if (EntityType == EFlightSimEntityType::Depot)
    {
        MeshPath = TEXT("/Engine/BasicShapes/Cube.Cube");
        Scale = FVector(8.0, 8.0, 2.0);
    }

    if (UStaticMesh* Mesh = LoadObject<UStaticMesh>(nullptr, MeshPath))
    {
        MeshComponent->SetStaticMesh(Mesh);
        MeshComponent->SetRelativeScale3D(Scale);
        MeshComponent->SetVisibility(true, true);
    }
}

void AFlightSimSceneActor::UpdateTransform(const FVector& LocationCm, const FQuat& Rotation)
{
    SetActorLocationAndRotation(LocationCm, Rotation, false, nullptr, ETeleportType::TeleportPhysics);
}
