#pragma once

#include "CoreMinimal.h"
#include "Modules/ModuleInterface.h"
#include "Modules/ModuleManager.h"

class FFlightSimROS2Module : public IModuleInterface
{
public:
    virtual void StartupModule() override;
    virtual void ShutdownModule() override;

    static FFlightSimROS2Module& Get()
    {
        return FModuleManager::LoadModuleChecked<FFlightSimROS2Module>("FlightSimROS2");
    }
};
