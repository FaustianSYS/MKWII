#include "FlightSimLondonMapActor.h"

#include "ProceduralMeshComponent.h"
#include "HAL/FileManager.h"
#include "Misc/FileHelper.h"

AFlightSimLondonMapActor::AFlightSimLondonMapActor()
{
    PrimaryActorTick.bCanEverTick = false;

    MeshComponent = CreateDefaultSubobject<UProceduralMeshComponent>(TEXT("LondonMesh"));
    SetRootComponent(MeshComponent);
    MeshComponent->SetCollisionEnabled(ECollisionEnabled::NoCollision);
    MeshComponent->SetCastShadow(false);
    MeshComponent->bUseAsyncCooking = true;
}

void AFlightSimLondonMapActor::BeginPlay()
{
    Super::BeginPlay();

    if (bLoadOnBeginPlay)
    {
        LoadLondonObj();
    }
}

bool AFlightSimLondonMapActor::LoadLondonObj()
{
    if (!MeshComponent)
    {
        return false;
    }

    if (!IFileManager::Get().FileExists(*ObjPath))
    {
        UE_LOG(LogTemp, Warning, TEXT("FlightSimLondonMap: OBJ not found: %s"), *ObjPath);
        return false;
    }

    FString ObjText;
    if (!FFileHelper::LoadFileToString(ObjText, *ObjPath))
    {
        UE_LOG(LogTemp, Warning, TEXT("FlightSimLondonMap: failed to read OBJ: %s"), *ObjPath);
        return false;
    }

    TArray<FVector> Vertices;
    TArray<int32> Triangles;
    TArray<FVector> Normals;
    TArray<FVector2D> UV0;
    TArray<FProcMeshTangent> Tangents;
    TArray<FLinearColor> Colors;

    TArray<FString> Lines;
    ObjText.ParseIntoArrayLines(Lines, false);

    Vertices.Reserve(100000);
    Triangles.Reserve(300000);

    for (const FString& RawLine : Lines)
    {
        FString Line = RawLine.TrimStartAndEnd();
        if (Line.IsEmpty() || Line.StartsWith(TEXT("#")))
        {
            continue;
        }

        if (Line.StartsWith(TEXT("v ")))
        {
            TArray<FString> Parts;
            Line.ParseIntoArrayWS(Parts);
            if (Parts.Num() >= 4)
            {
                const double X = FCString::Atod(*Parts[1]);
                const double Y = FCString::Atod(*Parts[2]);
                const double Z = FCString::Atod(*Parts[3]);

                // The preprocessed AccuCities mesh is already Z-up with X/Y
                // on the ground plane. Convert metres-ish source units to UE cm.
                Vertices.Add(FVector(
                    X * SourceMetersToCentimeters,
                    Y * SourceMetersToCentimeters,
                    Z * SourceMetersToCentimeters));
            }
        }
        else if (Line.StartsWith(TEXT("f ")))
        {
            TArray<FString> Parts;
            Line.ParseIntoArrayWS(Parts);
            if (Parts.Num() < 4)
            {
                continue;
            }

            TArray<int32> FaceIndices;
            FaceIndices.Reserve(Parts.Num() - 1);

            for (int32 PartIdx = 1; PartIdx < Parts.Num(); ++PartIdx)
            {
                FString IndexToken = Parts[PartIdx];
                FString VertexIndexText;
                if (!IndexToken.Split(TEXT("/"), &VertexIndexText, nullptr))
                {
                    VertexIndexText = IndexToken;
                }

                const int32 ObjIndex = FCString::Atoi(*VertexIndexText);
                if (ObjIndex == 0)
                {
                    continue;
                }

                const int32 VertexIndex = ObjIndex > 0 ? ObjIndex - 1 : Vertices.Num() + ObjIndex;
                if (Vertices.IsValidIndex(VertexIndex))
                {
                    FaceIndices.Add(VertexIndex);
                }
            }

            // Fan-triangulate polygons; the processed OBJ is already mostly
            // triangles, but this keeps the loader robust.
            for (int32 I = 1; I + 1 < FaceIndices.Num(); ++I)
            {
                Triangles.Add(FaceIndices[0]);
                Triangles.Add(FaceIndices[I]);
                Triangles.Add(FaceIndices[I + 1]);
            }
        }
    }

    if (Vertices.Num() == 0 || Triangles.Num() == 0)
    {
        UE_LOG(LogTemp, Warning, TEXT("FlightSimLondonMap: OBJ contained no renderable geometry: %s"), *ObjPath);
        return false;
    }

    Normals.Init(FVector::UpVector, Vertices.Num());
    UV0.Init(FVector2D::ZeroVector, Vertices.Num());
    Tangents.Init(FProcMeshTangent(1.0F, 0.0F, 0.0F), Vertices.Num());
    Colors.Init(FLinearColor(0.45F, 0.47F, 0.42F, 1.0F), Vertices.Num());

    MeshComponent->ClearAllMeshSections();
    MeshComponent->CreateMeshSection_LinearColor(
        0,
        Vertices,
        Triangles,
        Normals,
        UV0,
        Colors,
        Tangents,
        false);

    UE_LOG(LogTemp, Log, TEXT("FlightSimLondonMap: loaded %d vertices, %d triangles from %s"),
           Vertices.Num(), Triangles.Num() / 3, *ObjPath);
    return true;
}
