using UnrealBuildTool;
using System.IO;

public class FlightSimROS2 : ModuleRules
{
    public FlightSimROS2(ReadOnlyTargetRules Target) : base(Target)
    {
        PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;

        PublicDependencyModuleNames.AddRange(new string[]
        {
            "Core", "CoreUObject", "Engine",
            "RenderCore", "RHI",
            "ProceduralMeshComponent",
        });

        PrivateDependencyModuleNames.Add("Projects");

        // ----------------------------------------------------------------
        // The UE5 plugin talks to ROS 2 exclusively through the C wrapper
        // library (libflightsim_ue5_ros2_bridge.so).
        // No rclcpp headers, no STL templates, no RTTI — just a plain C API.
        // ----------------------------------------------------------------
        string BridgeInstall =
            "/home/j1p7/FlightSim/ros2/install/debug/flightsim_ue5_ros2_bridge";

        // C header only — no ROS 2 compiler flags needed.
        PublicIncludePaths.Add(Path.Combine(BridgeInstall, "include"));

        PublicAdditionalLibraries.Add(
            Path.Combine(BridgeInstall, "lib",
                         "libflightsim_ue5_ros2_bridge.so"));

        PublicRuntimeLibraryPaths.Add(Path.Combine(BridgeInstall, "lib"));

        // Ensure the .so is found at runtime via RPATH.
        PublicDelayLoadDLLs.Add("libflightsim_ue5_ros2_bridge.so");
    }
}
