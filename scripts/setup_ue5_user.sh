#!/usr/bin/env bash
# Run as your normal user (NO sudo).
# Clones UE5 5.5 source, builds it, and creates the FlightSim project scaffold.
#
# Prerequisites:
#   1. sudo ./scripts/setup_ue5_linux.sh   (already done - NVIDIA + deps)
#   2. sudo reboot                         (required once for the NVIDIA driver)
#   3. Epic GitHub access:  https://www.unrealengine.com/en-US/ue-on-github
# ===========================================================================
set -euo pipefail

UE5_INSTALL_DIR="${HOME}/UnrealEngine"
UE5_BRANCH="5.5"
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/ue5/FlightSimUE"

# ---------------------------------------------------------------------------
# Step 3: Clone UE5 source from Epic GitHub
# ---------------------------------------------------------------------------
clone_ue5() {
    if [[ -d "${UE5_INSTALL_DIR}/.git" ]]; then
        echo "[Step 3] Already cloned at ${UE5_INSTALL_DIR}. Fetching latest..."
        git -C "${UE5_INSTALL_DIR}" fetch --depth=1 origin "${UE5_BRANCH}" || true
        return
    fi

    echo "==> [Step 3] Cloning Unreal Engine ${UE5_BRANCH} (~30 GB, may take 20-40 min)..."
    echo ""
    echo "  Using SSH (recommended). Make sure your GitHub SSH key is set up:"
    echo "  https://docs.github.com/en/authentication/connecting-to-your-github-account/adding-a-new-ssh-key-to-your-github-account"
    echo ""

    git clone --depth=1 --branch "${UE5_BRANCH}" \
        git@github.com:EpicGames/UnrealEngine.git \
        "${UE5_INSTALL_DIR}"

    git -C "${UE5_INSTALL_DIR}" lfs install
    echo "[Step 3] ✓ Clone complete"
}

# ---------------------------------------------------------------------------
# Step 4: Setup and build UnrealEditor
# ---------------------------------------------------------------------------
build_ue5() {
    echo "==> [Step 4] Running Setup.sh (downloads binary dependencies ~10 GB)..."
    cd "${UE5_INSTALL_DIR}"
    ./Setup.sh

    echo "==> [Step 4] Generating project files..."
    ./GenerateProjectFiles.sh -cmake

    JOBS=$(nproc)
    echo "==> [Step 4] Building UnrealEditor with ${JOBS} jobs (2-4 hours)..."
    make UnrealEditor -j"${JOBS}" 2>&1 | tee /tmp/ue5_build.log

    echo "[Step 4] ✓ Build complete"
}

# ---------------------------------------------------------------------------
# Step 5: Create FlightSim UE5 project scaffold
# ---------------------------------------------------------------------------
create_project() {
    if [[ -f "${PROJECT_DIR}/FlightSimUE.uproject" ]]; then
        echo "[Step 5] Project already exists at ${PROJECT_DIR}"
        return
    fi

    echo "==> [Step 5] Creating FlightSimUE project scaffold..."
    mkdir -p "${PROJECT_DIR}/Source/FlightSimUE"
    mkdir -p "${PROJECT_DIR}/Content"
    mkdir -p "${PROJECT_DIR}/Config"

    cat > "${PROJECT_DIR}/FlightSimUE.uproject" <<'JSON'
{
    "FileVersion": 3,
    "EngineAssociation": "",
    "Category": "Simulation",
    "Description": "FlightSim air-defense co-simulation environment with ROS 2 bridge",
    "Modules": [
        {
            "Name": "FlightSimUE",
            "Type": "Runtime",
            "LoadingPhase": "Default"
        }
    ]
}
JSON

    cat > "${PROJECT_DIR}/Source/FlightSimUE/FlightSimUE.Build.cs" <<'CS'
using UnrealBuildTool;

public class FlightSimUE : ModuleRules
{
    public FlightSimUE(ReadOnlyTargetRules Target) : base(Target)
    {
        PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
        PublicDependencyModuleNames.AddRange(new string[] {
            "Core", "CoreUObject", "Engine", "InputCore", "RenderCore", "Renderer"
        });
        PrivateDependencyModuleNames.AddRange(new string[] {
            "SceneCapture"
        });
    }
}
CS

    cat > "${PROJECT_DIR}/Source/FlightSimUE/FlightSimUE.h" <<'H'
#pragma once
#include "CoreMinimal.h"
H

    cat > "${PROJECT_DIR}/Source/FlightSimUE/FlightSimUE.cpp" <<'CPP'
#include "FlightSimUE.h"
#include "Modules/ModuleManager.h"
IMPLEMENT_PRIMARY_GAME_MODULE(FDefaultGameModuleImpl, FlightSimUE, "FlightSimUE");
CPP

    cat > "${PROJECT_DIR}/Source/FlightSimUETarget.cs" <<'CS'
using UnrealBuildTool;
using System.Collections.Generic;

public class FlightSimUETarget : TargetRules
{
    public FlightSimUETarget(TargetInfo Target) : base(Target)
    {
        Type = TargetType.Game;
        DefaultBuildSettings = BuildSettingsVersion.V5;
        IncludeOrderVersion = EngineIncludeOrderVersion.Unreal5_5;
        ExtraModuleNames.Add("FlightSimUE");
    }
}
CS

    cat > "${PROJECT_DIR}/Source/FlightSimUEEditor.Target.cs" <<'CS'
using UnrealBuildTool;
using System.Collections.Generic;

public class FlightSimUEEditorTarget : TargetRules
{
    public FlightSimUEEditorTarget(TargetInfo Target) : base(Target)
    {
        Type = TargetType.Editor;
        DefaultBuildSettings = BuildSettingsVersion.V5;
        IncludeOrderVersion = EngineIncludeOrderVersion.Unreal5_5;
        ExtraModuleNames.Add("FlightSimUE");
    }
}
CS

    cat > "${PROJECT_DIR}/Config/DefaultEngine.ini" <<'INI'
[/Script/EngineSettings.GameMapsSettings]
GameDefaultMap=/Game/Maps/AirDefenseMap
EditorStartupMap=/Game/Maps/AirDefenseMap
INI

    echo "[Step 5] ✓ Project scaffold at ${PROJECT_DIR}"
}

# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
if [[ "${EUID}" -eq 0 ]]; then
    echo "ERROR: Do NOT run this script as root. Run as your normal user:"
    echo "  ./scripts/setup_ue5_user.sh"
    exit 1
fi

echo "========================================"
echo " FlightSim UE5 User Setup"
echo " UE5 dir:  ${UE5_INSTALL_DIR}"
echo " Project:  ${PROJECT_DIR}"
echo "========================================"
echo ""
echo "Before continuing, make sure you have:"
echo "  1. Rebooted (required for NVIDIA driver from previous step)"
echo "  2. Linked your GitHub account to Epic Games:"
echo "     https://www.unrealengine.com/en-US/ue-on-github"
echo "  3. SSH key set up for GitHub (or you'll be prompted for credentials)"
echo ""
read -rp "Press Enter to start cloning UE5, or Ctrl+C to abort..."

clone_ue5
build_ue5
create_project

echo ""
echo "========================================"
echo " All done!"
echo ""
echo " Launch UE5 editor:"
echo "   ${UE5_INSTALL_DIR}/Engine/Binaries/Linux/UnrealEditor \\"
echo "     ${PROJECT_DIR}/FlightSimUE.uproject"
echo ""
echo " Start FlightSim ROS bridge (separate terminal):"
echo "   ./scripts/launch_ue5_bridge.sh"
echo ""
echo " See ue5/README.md for integration steps."
echo "========================================"
