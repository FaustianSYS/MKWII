using UnrealBuildTool;
public class FlightSimUEEditorTarget : TargetRules {
    public FlightSimUEEditorTarget(TargetInfo Target) : base(Target) {
        Type = TargetType.Editor;
        DefaultBuildSettings = BuildSettingsVersion.V5;
        IncludeOrderVersion = EngineIncludeOrderVersion.Unreal5_5;
        ExtraModuleNames.Add("FlightSimUE");
    }
}
