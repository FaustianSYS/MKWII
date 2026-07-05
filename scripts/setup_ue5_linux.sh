#!/usr/bin/env bash
# ===========================================================================
# FlightSim – UE5 Linux Setup Script (Ubuntu 24.04, kernel 6.17+)
#
# Run this script ONCE with sudo. It:
#   1. Rebuilds the NVIDIA kernel module via DKMS (nvidia-smi will work)
#   2. Installs UE5 build prerequisites
#   3. Clones UE5 5.5 source (Epic GitHub – you must have access first)
#   4. Generates project files and kicks off the build
#
# PREREQUISITES BEFORE RUNNING:
#   a) Link your Epic Games account to GitHub:
#      https://www.unrealengine.com/en-US/ue-on-github
#   b) Accept the UE source agreement on GitHub so you can access:
#      https://github.com/EpicGames/UnrealEngine
#   c) Create a GitHub Personal Access Token with repo scope, or use SSH key.
#
# USAGE:
#   sudo ./scripts/setup_ue5_linux.sh
#   (or run the STEP sections manually in your own terminal)
# ===========================================================================
set -euo pipefail

KERNEL="$(uname -r)"
UE5_VERSION="5.5"
UE5_BRANCH="5.5"
UE5_INSTALL_DIR="${HOME}/UnrealEngine"
GITHUB_USER="${GITHUB_USER:-}"   # set to your GitHub username, or leave blank

# ---------------------------------------------------------------------------
# STEP 1: Fix NVIDIA driver (open kernel module for Blackwell RTX 5000+)
# ---------------------------------------------------------------------------
step1_fix_nvidia() {
    echo "==> [Step 1] Checking NVIDIA driver for kernel ${KERNEL}"

    # If nvidia-smi already works, nothing to do
    if nvidia-smi &>/dev/null; then
        echo "[Step 1] ✓ NVIDIA driver already working:"
        nvidia-smi --query-gpu=name,memory.total --format=csv,noheader
        return
    fi

    echo "[Step 1] nvidia-smi not responding, installing open kernel driver..."

    # RTX 5000+ (Blackwell) and many RTX 4000+ require the open kernel module.
    # Always install the -open variant to avoid the proprietary module being
    # pulled in by a plain 'nvidia-driver-NNN' package.
    OPEN_PKG=""
    for ver in 595 590 580; do
        if apt-cache show "nvidia-driver-${ver}-open" &>/dev/null; then
            OPEN_PKG="nvidia-driver-${ver}-open"
            break
        fi
    done

    if [[ -z "${OPEN_PKG}" ]]; then
        echo "[Step 1] No -open driver found in apt. Trying ubuntu-drivers..."
        apt-get install -y ubuntu-drivers-common
        ubuntu-drivers install --gpgpu || ubuntu-drivers install
    else
        echo "[Step 1] Installing ${OPEN_PKG}..."
        apt-get install -y "${OPEN_PKG}"
    fi

    echo "[Step 1] Loading NVIDIA modules..."
    modprobe nvidia nvidia-uvm nvidia-modeset 2>/dev/null || true

    if nvidia-smi &>/dev/null; then
        echo "[Step 1] ✓ NVIDIA driver OK"
        nvidia-smi --query-gpu=name,memory.total --format=csv,noheader
    else
        echo "[Step 1] ✗ NVIDIA module not responding — a REBOOT is required."
        echo "          After reboot, re-run this script (steps 1+2 will be fast)."
    fi
}

# ---------------------------------------------------------------------------
# STEP 2: Install UE5 build dependencies
# ---------------------------------------------------------------------------
step2_install_deps() {
    echo "==> [Step 2] Installing UE5 build prerequisites..."
    apt-get update -qq
    apt-get install -y \
        git git-lfs \
        build-essential \
        clang-18 lld-18 libc++-18-dev libc++abi-18-dev \
        libxrandr-dev libxrender-dev libxcursor-dev libxi-dev libxinerama-dev \
        libxss-dev libglu1-mesa-dev \
        libsdl2-dev \
        libasound2-dev \
        libudev-dev \
        python3 python3-pip \
        vulkan-tools libvulkan-dev \
        pkg-config \
        cmake ninja-build

    # clang-18 as default (UE5 requires clang, not GCC for linking)
    update-alternatives --install /usr/bin/clang   clang   /usr/bin/clang-18   100 2>/dev/null || true
    update-alternatives --install /usr/bin/clang++ clang++ /usr/bin/clang++-18 100 2>/dev/null || true

    echo "[Step 2] ✓ Dependencies installed"
}

# ---------------------------------------------------------------------------
# STEP 3: Clone UE5 source
# ---------------------------------------------------------------------------
step3_clone_ue5() {
    if [[ -d "${UE5_INSTALL_DIR}/.git" ]]; then
        echo "[Step 3] UE5 source already cloned at ${UE5_INSTALL_DIR}, pulling..."
        git -C "${UE5_INSTALL_DIR}" fetch --depth=1 origin "${UE5_BRANCH}"
        return
    fi

    echo "==> [Step 3] Cloning UE5 ${UE5_VERSION} source (this is ~30 GB)..."
    echo ""
    echo "  You must have Epic GitHub access. Join at:"
    echo "  https://www.unrealengine.com/en-US/ue-on-github"
    echo ""

    if [[ -n "${GITHUB_USER}" ]]; then
        REPO="https://github.com/EpicGames/UnrealEngine"
    else
        REPO="git@github.com:EpicGames/UnrealEngine.git"
    fi

    git clone --depth=1 --branch "${UE5_BRANCH}" "${REPO}" "${UE5_INSTALL_DIR}"
    git -C "${UE5_INSTALL_DIR}" lfs install
    echo "[Step 3] ✓ Clone complete"
}

# ---------------------------------------------------------------------------
# STEP 4: Setup and build UE5
# ---------------------------------------------------------------------------
step4_build_ue5() {
    echo "==> [Step 4] Setting up UE5 build (this takes 2–4 hours)..."
    cd "${UE5_INSTALL_DIR}"

    ./Setup.sh
    ./GenerateProjectFiles.sh -cmake

    # Build UnrealEditor only (skip everything else to save time/disk)
    JOBS="${NPROC:-$(nproc)}"
    echo "[Step 4] Building with ${JOBS} parallel jobs..."
    make UnrealEditor -j"${JOBS}" 2>&1 | tee /tmp/ue5_build.log

    echo "[Step 4] ✓ UE5 build complete"
    echo "  Launch: ${UE5_INSTALL_DIR}/Engine/Binaries/Linux/UnrealEditor"
}

# ---------------------------------------------------------------------------
# STEP 5: Create FlightSim UE5 project
# ---------------------------------------------------------------------------
step5_create_project() {
    local PROJECT_DIR="/home/j1p7/FlightSim/ue5/FlightSimUE"
    if [[ -d "${PROJECT_DIR}" ]]; then
        echo "[Step 5] Project already exists at ${PROJECT_DIR}"
        return
    fi

    echo "==> [Step 5] Creating FlightSim UE5 project..."
    mkdir -p "${PROJECT_DIR}"
    cat > "${PROJECT_DIR}/FlightSimUE.uproject" <<'JSON'
{
    "FileVersion": 3,
    "EngineAssociation": "",
    "Category": "",
    "Description": "FlightSim UE5 co-simulation environment",
    "Modules": [
        {
            "Name": "FlightSimUE",
            "Type": "Runtime",
            "LoadingPhase": "Default"
        }
    ],
    "Plugins": [
        {
            "Name": "ROS2Plugin",
            "Enabled": true
        }
    ]
}
JSON

    # Source module scaffold
    mkdir -p "${PROJECT_DIR}/Source/FlightSimUE"
    cat > "${PROJECT_DIR}/Source/FlightSimUE/FlightSimUE.Build.cs" <<'CS'
using UnrealBuildTool;

public class FlightSimUE : ModuleRules
{
    public FlightSimUE(ReadOnlyTargetRules Target) : base(Target)
    {
        PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
        PublicDependencyModuleNames.AddRange(new string[] {
            "Core", "CoreUObject", "Engine", "InputCore", "RenderCore"
        });
    }
}
CS

    echo "[Step 5] ✓ Project scaffold created at ${PROJECT_DIR}"
    echo "  Open with: ${UE5_INSTALL_DIR}/Engine/Binaries/Linux/UnrealEditor ${PROJECT_DIR}/FlightSimUE.uproject"
}

# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
if [[ "${EUID}" -ne 0 ]]; then
    echo "ERROR: Run as root: sudo $0"
    echo ""
    echo "You can also run individual steps manually:"
    echo "  sudo bash -c 'source $0 && step1_fix_nvidia'"
    echo "  sudo bash -c 'source $0 && step2_install_deps'"
    echo "  bash -c 'source $0 && step3_clone_ue5'   # no sudo needed"
    echo "  bash -c 'source $0 && step4_build_ue5'"
    echo "  bash -c 'source $0 && step5_create_project'"
    exit 1
fi

echo "========================================"
echo " FlightSim UE5 Linux Setup"
echo " Kernel: ${KERNEL}"
echo " Install: ${UE5_INSTALL_DIR}"
echo "========================================"

step1_fix_nvidia
step2_install_deps

echo ""
echo "========================================"
echo " Sudo steps complete."
echo " Run the following as your normal user to clone + build UE5:"
echo "   ./scripts/setup_ue5_user.sh"
echo "========================================"

echo ""
echo "========================================"
echo " Setup complete!"
echo " 1. Fix NVIDIA if needed: reboot if step1 failed"
echo " 2. Launch UE5:"
echo "    ${UE5_INSTALL_DIR}/Engine/Binaries/Linux/UnrealEditor"
echo " 3. Start FlightSim bridge:"
echo "    ./scripts/launch_ue5_bridge.sh"
echo " 4. See ue5/README.md for integration steps"
echo "========================================"
