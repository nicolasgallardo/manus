#!/bin/bash

# Linux build script for Hand IK
# Assumes Pinocchio and Eigen are already installed

echo "========================================"
echo "Hand IK Linux Build Script"
echo "========================================"

# Check if we're in the right directory
if [ ! -f "CMakeLists.txt" ]; then
    echo "Error: CMakeLists.txt not found!"
    echo "Please run this script from the hand_ik root directory."
    exit 1
fi

# Function to check if a command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# Check dependencies
echo "Checking dependencies..."

if ! command_exists cmake; then
    echo "Error: CMake not found!"
    echo "Install with: sudo apt install cmake"
    exit 1
fi

if ! command_exists g++; then
    echo "Error: g++ not found!"
    echo "Install with: sudo apt install build-essential"
    exit 1
fi

if ! command_exists pkg-config; then
    echo "Error: pkg-config not found!"
    echo "Install with: sudo apt install pkg-config"
    exit 1
fi

# Check for Eigen3
if ! pkg-config --exists eigen3; then
    echo "Warning: Eigen3 not found via pkg-config"
    echo "Install with: sudo apt install libeigen3-dev"
fi

echo "Dependencies check completed."

# Create build directory if it doesn't exist
if [ ! -d "build" ]; then
    echo "Creating build directory..."
    mkdir build
fi

cd build

# Configure with CMake
echo "========================================"
echo "Configuring with CMake..."
echo "========================================"

cmake ..

if [ $? -ne 0 ]; then
    echo "CMake configuration failed!"
    echo ""
    echo "Possible issues:"
    echo "- Pinocchio not found (install with: sudo apt install robotpkg-py3*-pinocchio)"
    echo "- Eigen3 not found (install with: sudo apt install libeigen3-dev)"
    echo "- Missing build tools (install with: sudo apt install build-essential)"
    echo ""
    exit 1
fi

# Build with make
echo "========================================"
echo "Building with make..."
echo "========================================"

# Use all available cores for faster building
CORES=$(nproc)
echo "Building with $CORES cores..."

make -j$CORES

if [ $? -ne 0 ]; then
    echo "Build failed!"
    exit 1
fi

echo "========================================"
echo "Build completed successfully!"
echo "========================================"
echo ""
echo "Executables are located in:"
echo "  build/hand_ik_example"
echo "  build/test_jacobian"
echo "  build/test_reachability"
echo "  build/validate_coupling"
echo ""

# Check if URDF file exists
if [ ! -f "../surge_v13_hand_right_pybullet.urdf" ]; then
    echo "WARNING: URDF file not found!"
    echo "Please place 'surge_v13_hand_right_pybullet.urdf' in the root directory."
    echo ""
fi

# Ask if user wants to run tests
read -p "Run tests now? (y/n): " run_tests

if [[ $run_tests =~ ^[Yy]$ ]]; then
    echo ""
    echo "========================================"
    echo "Running Tests..."
    echo "========================================"
    
    echo "Running Passive Coupling Validation..."
    ./validate_coupling
    
    echo ""
    echo "Running Jacobian test..."
    ./test_jacobian
    
    echo ""
    echo "Running Reachability test..."
    ./test_reachability
    
    echo ""
    echo "Running Example..."
    ./hand_ik_example
fi

echo ""
echo "Build script completed!"

# Make the script executable when created
chmod +x build_linux.sh