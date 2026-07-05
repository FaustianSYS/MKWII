#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "FlightSimLondonMapActor.generated.h"

class UProceduralMeshComponent;

UCLASS()
class AFlightSimLondonMapActor : public AActor
{
    GENERATED_BODY()

public:
    AFlightSimLondonMapActor();

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "FlightSim|London")
    UProceduralMeshComponent* MeshComponent = nullptr;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FlightSim|London")
    FString ObjPath = TEXT("/home/j1p7/FlightSim/assets/gui/maps/accucities_tq3280/london_tq3280_lod2.obj");

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FlightSim|London")
    float SourceMetersToCentimeters = 100.0F;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FlightSim|London")
    bool bLoadOnBeginPlay = true;

    UFUNCTION(BlueprintCallable, Category = "FlightSim|London")
    bool LoadLondonObj();

protected:
    virtual void BeginPlay() override;
};
