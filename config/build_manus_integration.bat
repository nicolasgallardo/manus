@echo off
setlocal EnableExtensions EnableDelayedExpansion

rem === Configurable ===
set "GEN=Visual Studio 17 2022"
set "ARCH=x64"
set "CFG=Release"
set "BUILD_DIR=build"
set "VCPKG_ROOT=C:\Users\nicol\vcpkg"

echo ========================================
echo Building Manus Core + Hand IK Integration
echo ========================================

rem Always run from repo root
pushd "%~dp0"

rem Check if we're in the right directory
if not exist "CMakeLists.txt" (
    echo Error: CMakeLists.txt not found!
    echo Please run this script from the manus repository root directory.
    goto :fail
)

rem 1) Check for Manus SDK
echo ========================================
echo [check] Verifying Manus SDK...
echo ========================================

set "MANUS_SDK_DIR=%CD%\MANUS_Core_3.0.0_SDK"
if not exist "%MANUS_SDK_DIR%" (
    echo ERROR: Manus SDK not found at: %MANUS_SDK_DIR%
    echo.
    echo Please ensure the Manus SDK is extracted to:
    echo   %MANUS_SDK_DIR%
    echo.
    echo Download from: https://developer.manus-meta.com/
    goto :fail
)

echo Found Manus SDK at: %MANUS_SDK_DIR%

rem Check key SDK files
if not exist "%MANUS_SDK_DIR%\include\ManusSDK.h" (
    echo ERROR: ManusSDK.h not found in %MANUS_SDK_DIR%\include
    goto :fail
)

if not exist "%MANUS_SDK_DIR%\lib\win64\ManusSDK.lib" (
    echo ERROR: ManusSDK.lib not found in %MANUS_SDK_DIR%\lib\win64
    goto :fail
)

if not exist "%MANUS_SDK_DIR%\bin\win64\ManusSDK.dll" (
    echo ERROR: ManusSDK.dll not found in %MANUS_SDK_DIR%\bin\win64
    goto :fail
)

echo ✓ Manus SDK structure validated

rem 2) Check for source files
echo ========================================
echo [check] Verifying integration source files...
echo ========================================

set "MISSING_FILES=0"

if not exist "src\SDKClientIntegration.cpp" (
    echo ERROR: src\SDKClientIntegration.cpp not found
    set "MISSING_FILES=1"
) else (
    echo ✓ src\SDKClientIntegration.cpp
)

if not exist "src\ManusHandIKBridge.cpp" (
    echo ERROR: src\ManusHandIKBridge.cpp not found
    set "MISSING_FILES=1"
) else (
    echo ✓ src\ManusHandIKBridge.cpp
)

if not exist "src\ManusSkeletonSetup.cpp" (
    echo ERROR: src\ManusSkeletonSetup.cpp not found
    set "MISSING_FILES=1"
) else (
    echo ✓ src\ManusSkeletonSetup.cpp
)

if not exist "include\ManusHandIKBridge.h" (
    echo ERROR: include\ManusHandIKBridge.h not found
    set "MISSING_FILES=1"
) else (
    echo ✓ include\ManusHandIKBridge.h
)

if not exist "include\ManusSkeletonSetup.h" (
    echo ERROR: include\ManusSkeletonSetup.h not found
    set "MISSING_FILES=1"
) else (
    echo ✓ include\ManusSkeletonSetup.h
)

if "%MISSING_FILES%"=="1" (
    echo.
    echo ERROR: Missing required integration source files!
    echo Please ensure all Manus integration files are present.
    goto :fail
)

echo ✓ All integration source files found

rem 3) Configure with Manus integration enabled
echo ========================================
echo [configure] Configuring with Manus integration...
echo ========================================

if exist "%BUILD_DIR%\CMakeCache.txt" (
    echo Cleaning existing build configuration...
    del "%BUILD_DIR%\CMakeCache.txt" 2>nul
)

cmake -S . -B "%BUILD_DIR%" -G "%GEN%" -A %ARCH% ^
  -DCMAKE_TOOLCHAIN_FILE="%VCPKG_ROOT%\scripts\buildsystems\vcpkg.cmake" ^
  -DCMAKE_BUILD_TYPE=%CFG% ^
  -DBUILD_MANUS_INTEGRATION=ON ^
  -DMANUS_SDK_DIR="%MANUS_SDK_DIR%" ^
  -DHAND_IK_BUILD_EXAMPLES=ON ^
  -DHAND_IK_BUILD_TESTS=ON

if errorlevel 1 goto :fail_configure

echo ✓ Configuration successful

rem 4) Build external dependencies first
echo ========================================
echo [externals] Building external dependencies...
echo ========================================

echo Building staged dependencies (this may take several minutes)...

cmake --build "%BUILD_DIR%" --config %CFG% --target tinyxml2_ext urdfdom_headers_ext urdfdom_ext pinocchio_ext
if errorlevel 1 goto :fail_externals

echo ✓ External dependencies built successfully

rem 5) Build Hand IK library
echo ========================================
echo [hand_ik] Building Hand IK library...
echo ========================================

cmake --build "%BUILD_DIR%" --config %CFG% --target hand_ik
if errorlevel 1 goto :fail_hand_ik

echo ✓ Hand IK library built successfully

rem 6) Build Manus integration components
echo ========================================
echo [manus] Building Manus integration...
echo ========================================

cmake --build "%BUILD_DIR%" --config %CFG% --target manus_hand_ik_bridge SDKClient
if errorlevel 1 goto :fail_manus

echo ✓ Manus integration built successfully

rem 7) Build tests if enabled
if exist "%BUILD_DIR%\test_manus_integration.vcxproj" (
    echo ========================================
    echo [tests] Building integration tests...
    echo ========================================
    
    cmake --build "%BUILD_DIR%" --config %CFG% --target test_manus_integration
    if errorlevel 1 (
        echo Warning: Integration test build failed, but continuing...
    ) else (
        echo ✓ Integration tests built successfully
    )
)

rem 8) Verify output files
echo ========================================
echo [verify] Verifying build outputs...
echo ========================================

set "OUTPUT_DIR=%BUILD_DIR%\bin\%CFG%"
set "MISSING_OUTPUTS=0"

if not exist "%OUTPUT_DIR%\SDKClient.exe" (
    echo ERROR: SDKClient.exe not found
    set "MISSING_OUTPUTS=1"
) else (
    echo ✓ SDKClient.exe
)

if not exist "%OUTPUT_DIR%\ManusSDK.dll" (
    echo ERROR: ManusSDK.dll not auto-copied
    set "MISSING_OUTPUTS=1"
) else (
    echo ✓ ManusSDK.dll (auto-copied)
)

if not exist "%OUTPUT_DIR%\surge_v13_hand_right_pybullet.urdf" (
    echo WARNING: URDF file not auto-copied
) else (
    echo ✓ surge_v13_hand_right_pybullet.urdf (auto-copied)
)

if "%MISSING_OUTPUTS%"=="1" (
    echo.
    echo ERROR: Missing critical output files!
    goto :fail
)

rem 9) Copy configuration file
echo ========================================
echo [config] Setting up configuration...
echo ========================================

if exist "config\manus_integration.json" (
    copy "config\manus_integration.json" "%OUTPUT_DIR%\" >nul
    echo ✓ Configuration file copied
) else (
    echo Warning: config\manus_integration.json not found
)

rem 10) Success summary
echo ========================================
echo ✓ BUILD SUCCESSFUL!
echo ========================================

echo.
echo Integration components built:
echo   • SDKClient.exe            - Main integration executable
echo   • manus_hand_ik_bridge.lib - Bridge library
echo   • ManusSDK.dll             - Manus SDK runtime (auto-copied)
echo   • URDF file                - Hand model (auto-copied)
echo.

echo Output directory: %CD%\%OUTPUT_DIR%
echo.

echo Next steps:
echo   1. Start Manus Core Dashboard
echo   2. Pair your Manus gloves
echo   3. Run: %OUTPUT_DIR%\SDKClient.exe --side=right --debug
echo.

echo Command line options:
echo   --side=left^|right^|both    - Which hand(s) to track
echo   --host=IP                  - Manus Core host (default: 127.0.0.1)
echo   --port=PORT                - Manus Core port (default: 9004)
echo   --simulate                 - Use simulated motion instead of gloves
echo   --debug                    - Enable debug logging
echo   --urdf=PATH                - Custom URDF file path
echo.

popd
exit /b 0

:fail_configure
echo.
echo ❌ Configuration failed!
echo.
echo Common issues:
echo   1. CMake not found in PATH
echo   2. Visual Studio 2022 not installed
echo   3. vcpkg toolchain file not found
echo   4. Missing vcpkg packages (boost, eigen3)
echo.
echo Try:
echo   vcpkg install boost-filesystem boost-system boost-serialization eigen3 --triplet x64-windows
goto :fail

:fail_externals
echo.
echo ❌ External dependencies build failed!
echo.
echo This usually indicates:
echo   1. Network issues downloading dependencies
echo   2. Corrupted CMake cache - try: rmdir /s /q %BUILD_DIR%
echo   3. Missing compiler toolchain
echo   4. vcpkg package issues
goto :fail

:fail_hand_ik
echo.
echo ❌ Hand IK library build failed!
echo.
echo Check for:
echo   1. C++ compilation errors in hand_ik sources
echo   2. Missing Pinocchio/Eigen dependencies
echo   3. MSVC-specific issues (M_PI, constexpr, etc.)
goto :fail

:fail_manus
echo.
echo ❌ Manus integration build failed!
echo.
echo Check for:
echo   1. Missing Manus SDK headers/libraries
echo   2. Incorrect ManusSDK.h API usage
echo   3. Linker errors with ManusSDK.lib
echo   4. Missing include paths for Manus SDK
echo.
echo Verify Manus SDK structure:
echo   %MANUS_SDK_DIR%\include\ManusSDK.h
echo   %MANUS_SDK_DIR%\lib\win64\ManusSDK.lib
echo   %MANUS_SDK_DIR%\bin\win64\ManusSDK.dll
goto :fail

:fail
echo.
echo ❌ BUILD FAILED
echo Check the error messages above for specific issues.
popd
exit /b 1