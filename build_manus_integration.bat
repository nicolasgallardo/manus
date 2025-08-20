@echo off
setlocal EnableExtensions EnableDelayedExpansion

echo ========================================
echo Manus Hand IK Integration Build Script
echo ========================================

rem Check if we're in the right directory
if not exist "CMakeLists.txt" (
    echo Error: CMakeLists.txt not found!
    echo Please run this script from the manus repository root directory.
    pause
    exit /b 1
)

rem Check for required directories
if not exist "hand_ik" (
    echo Error: hand_ik directory not found!
    echo Please ensure the hand_ik library is present in the repository.
    pause
    exit /b 1
)

if not exist "MANUS_Core_3.0.0_SDK" (
    echo Warning: MANUS_Core_3.0.0_SDK directory not found!
    echo The build will continue but may not find Manus SDK libraries.
    echo.
)

rem Create src directory and stub if they don't exist
if not exist "src" (
    echo Creating src directory...
    mkdir src
)

if not exist "src\SDKClientIntegration.cpp" (
    echo Creating SDKClientIntegration.cpp stub...
    echo This will create a minimal stub that allows the build to succeed.
    echo You can replace it with the actual integration code later.
    echo.
)

rem Set vcpkg path (adjust if your vcpkg is in a different location)
set VCPKG_ROOT=C:\Users\nicol\vcpkg
if not exist "%VCPKG_ROOT%" (
    echo Warning: vcpkg not found at %VCPKG_ROOT%
    echo Please adjust the VCPKG_ROOT path in this script if vcpkg is elsewhere.
    echo.
)

rem Clean previous build (optional)
set /p clean_build="Clean previous build? (y/n): "
if /i "%clean_build%"=="y" (
    echo Cleaning previous build...
    if exist "build" rmdir /s /q build
)

rem Create build directory
if not exist "build" mkdir build

echo ========================================
echo Configuring with CMake...
echo ========================================

rem Configure with CMake
cmake -S . -B build -G "Visual Studio 17 2022" ^
  -DCMAKE_TOOLCHAIN_FILE=%VCPKG_ROOT%\scripts\buildsystems\vcpkg.cmake ^
  -DVCPKG_TARGET_TRIPLET=x64-windows ^
  -DMANUS_SDK_DIR=%CD%\MANUS_Core_3.0.0_SDK ^
  -DBUILD_WITH_PINOCCHIO=ON ^
  -DBUILD_MANUS_INTEGRATION=ON ^
  -DBUILD_TESTING=ON

if errorlevel 1 (
    echo.
    echo ========================================
    echo CMake Configuration FAILED!
    echo ========================================
    echo.
    echo Common issues:
    echo 1. vcpkg not found or missing packages
    echo    - Install: vcpkg install boost-filesystem boost-system boost-serialization eigen3 --triplet x64-windows
    echo 2. Visual Studio 2022 not installed
    echo 3. Missing dependencies ^(Pinocchio, Eigen3^)
    echo.
    echo Try running with different options:
    echo   -DBUILD_WITH_PINOCCHIO=OFF  ^(if Pinocchio issues^)
    echo   -DBUILD_TESTING=OFF         ^(if Google Test issues^)
    echo.
    pause
    exit /b 1
)

echo.
echo ========================================
echo Building with Visual Studio...
echo ========================================

rem Build the project
cmake --build build --config Release -j

if errorlevel 1 (
    echo.
    echo ========================================
    echo Build FAILED!
    echo ========================================
    echo.
    echo Check the error messages above for specific issues.
    echo Common problems:
    echo 1. Missing include files - check Manus SDK headers
    echo 2. Linking errors - check library paths
    echo 3. Compiler errors - check source code compatibility
    echo.
    pause
    exit /b 1
)

echo.
echo ========================================
echo Build COMPLETED successfully!
echo ========================================

rem Check what was built
echo.
echo Built executables:
if exist "build\bin\Release\SDKClient.exe" (
    echo   ✓ SDKClient.exe
) else (
    echo   ✗ SDKClient.exe ^(not found^)
)

if exist "build\bin\Release\test_manus_integration.exe" (
    echo   ✓ test_manus_integration.exe
) else (
    echo   ✗ test_manus_integration.exe ^(not found - GTest may be missing^)
)

echo.
echo Built libraries:
if exist "build\lib\Release\ManusHandIK.lib" (
    echo   ✓ ManusHandIK.lib
) else (
    echo   ✗ ManusHandIK.lib ^(not found^)
)

if exist "build\lib\Release\hand_ik.lib" (
    echo   ✓ hand_ik.lib
) else (
    echo   ✗ hand_ik.lib ^(not found^)
)

rem Copy URDF file if it exists
if exist "hand_ik\surge_v13_hand_right_pybullet.urdf" (
    echo.
    echo Copying URDF file to build directory...
    copy "hand_ik\surge_v13_hand_right_pybullet.urdf" "build\bin\Release\" >nul 2>&1
    if exist "build\bin\Release\surge_v13_hand_right_pybullet.urdf" (
        echo   ✓ URDF file copied successfully
    )
)

echo.
echo ========================================
echo Next Steps
echo ========================================
echo.
echo 1. Test the integration:
echo    cd build\bin\Release
echo    .\test_manus_integration.exe
echo.
echo 2. Run the SDK client:
echo    .\SDKClient.exe
echo.
echo 3. To implement the full integration:
echo    - Replace src\SDKClientIntegration.cpp with the complete integration code
echo    - Include the ManusHandIKBridge.h/cpp files
echo    - Include the ManusSkeletonSetup.h/cpp files
echo    - Update the Manus SDK header includes
echo.
echo 4. Build configuration summary:
echo    - Manus SDK: %CD%\MANUS_Core_3.0.0_SDK
echo    - vcpkg: %VCPKG_ROOT%
echo    - Pinocchio: %BUILD_WITH_PINOCCHIO%
echo    - Testing: %BUILD_TESTING%
echo.

rem Ask if user wants to run tests
set /p run_tests="Run tests now? (y/n): "
if /i "%run_tests%"=="y" (
    echo.
    echo Running tests...
    cd build\bin\Release
    if exist "test_manus_integration.exe" (
        echo.
        echo === Running Integration Tests ===
        .\test_manus_integration.exe
    ) else (
        echo Tests not available ^(GTest not found during build^)
    )
    
    if exist "SDKClient.exe" (
        echo.
        echo === Running SDK Client Stub ===
        .\SDKClient.exe
    )
    cd ..\..\..
)

echo.
echo Build script completed!
pause