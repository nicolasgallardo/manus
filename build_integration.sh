#!/bin/bash

echo "=========================================="
echo "Building Manus Hand IK Integration"
echo "=========================================="

# Check if we're in the right directory
if [ ! -f "CMakeLists.txt" ]; then
    echo "Error: CMakeLists.txt not found!"
    echo "Please run this script from the project root directory."
    exit 1
fi

# Check for URDF file
URDF_FILE="surge_v13_hand_right_pybullet.urdf"
if [ ! -f "$URDF_FILE" ] && [ ! -f "hand_ik/$URDF_FILE" ]; then
    echo "Warning: URDF file not found!"
    echo "Please ensure $URDF_FILE is in the root or hand_ik/ directory."
fi

# Check that hand_ik subdirectory exists
if [ ! -d "hand_ik" ]; then
    echo "Error: hand_ik directory not found!"
    echo "Please ensure your Hand IK library is in the hand_ik/ subdirectory."
    exit 1
fi

# Create build directory
mkdir -p build
cd build

# Configure with CMake
echo "Configuring with CMake..."
cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON

if [ $? -ne 0 ]; then
    echo "CMake configuration failed!"
    echo ""
    echo "Common issues:"
    echo "- Missing Manus SDK libraries (update CMakeLists.txt with correct library names)"
    echo "- Missing hand_ik library (ensure it builds successfully first)"
    echo "- Missing dependencies (Eigen3, etc.)"
    exit 1
fi

# Build
echo "Building..."
make -j$(nproc)

if [ $? -ne 0 ]; then
    echo "Build failed!"
    echo ""
    echo "Common issues:"
    echo "- Missing Manus SDK headers (replace placeholder includes in source files)"
    echo "- Linking errors (check library names in CMakeLists.txt)"
    echo "- C++17 compiler issues"
    exit 1
fi

echo "=========================================="
echo "Build completed successfully!"
echo "=========================================="

# Check executables
echo "Built executables:"
if [ -f "SDKClient" ]; then
    echo "  ✓ SDKClient"
else
    echo "  ✗ SDKClient (not found)"
fi

if [ -f "tests/test_manus_integration" ]; then
    echo "  ✓ test_manus_integration"
else
    echo "  ✗ test_manus_integration (not found)"
fi

# Copy URDF to build directory if found
if [ -f "../$URDF_FILE" ]; then
    cp "../$URDF_FILE" .
    echo "  ✓ Copied $URDF_FILE to build directory"
elif [ -f "../hand_ik/$URDF_FILE" ]; then
    cp "../hand_ik/$URDF_FILE" .
    echo "  ✓ Copied $URDF_FILE from hand_ik/ to build directory"
fi

echo ""
echo "Next steps:"
echo "=========================================="
echo "1. Replace Manus SDK placeholder includes:"
echo "   - Edit src/ManusSkeletonSetup.cpp"
echo "   - Edit src/SDKClientIntegration.cpp" 
echo "   - Replace placeholder includes with your actual Manus SDK headers"
echo ""
echo "2. Update CMakeLists.txt with your actual Manus SDK library names"
echo ""
echo "3. Test the integration:"
echo "   ./tests/test_manus_integration"
echo ""
echo "4. Run the integrated client:"
echo "   ./SDKClient"
echo ""
echo "For detailed setup instructions, see the integration documentation."