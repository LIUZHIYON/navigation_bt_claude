#!/bin/bash
# ================================================================
# Navigation Behavior Tree Demo - One-click Run Script
# ================================================================
# This script:
#   1. Configures CMake (if build directory doesn't exist)
#   2. Builds the project
#   3. Runs the navigation demo
#
# After running, open Groot2 and connect to tcp://localhost:1667
# to monitor the behavior tree execution live.
# ================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
BUILD_DIR="$PROJECT_DIR/build"

echo "============================================"
echo " Navigation Behavior Tree - A* Pathfinding"
echo "============================================"
echo ""
echo "Project: $PROJECT_DIR"
echo "Build:   $BUILD_DIR"
echo ""

# --- Step 1: Configure ---
if [ ! -f "$BUILD_DIR/CMakeCache.txt" ]; then
    echo "[1/3] Configuring CMake..."
    mkdir -p "$BUILD_DIR"
    cd "$BUILD_DIR"
    cmake .. -DCMAKE_BUILD_TYPE=Release
else
    echo "[1/3] CMake already configured."
    cd "$BUILD_DIR"
fi

# --- Step 2: Build ---
echo "[2/3] Building project..."
cmake --build . -j"$(nproc 2>/dev/null || echo 4)"

# --- Step 3: Run ---
echo "[3/3] Running demo..."
echo ""
echo "============================================"
echo " Groot2 Live Monitor: tcp://localhost:1667"
echo " Open Groot2 -> Monitor -> Connect"
echo "============================================"
echo ""

./navigation_demo "$@"

echo ""
echo "============================================"
echo " Demo finished!"
echo "============================================"
