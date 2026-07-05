#include "FlightSimROS2Module.h"
#include "Modules/ModuleManager.h"
#include "Ros2Bridge.h"   // plain C API — no rclcpp headers

IMPLEMENT_MODULE(FFlightSimROS2Module, FlightSimROS2)

void FFlightSimROS2Module::StartupModule()
{
    fsros2_global_init();
    UE_LOG(LogTemp, Log, TEXT("FlightSimROS2: fsros2 bridge initialised"));
}

void FFlightSimROS2Module::ShutdownModule()
{
    fsros2_global_shutdown();
    UE_LOG(LogTemp, Log, TEXT("FlightSimROS2: fsros2 bridge shut down"));
}
