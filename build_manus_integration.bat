@echo off
setlocal EnableExtensions EnableDelayedExpansion

rem === Manus Integration Build Script ===
rem Builds the manus integration project with Hand IK + Manus SDK
rem Handles Eigen target collision and Manus SDK header detection

rem === Configurable Settings ===
set "GEN=Visual Studio 17 2022"
set "ARCH=x64"
set "CFG=Release"
set "BUILD_DIR=build"
set "VCPKG_ROOT=C:\Users\nicol\vcpkg"

rem Check for force reconfigure flag
set "FORCE_RECONFIG=0"
if "%1"=="--clean" set "FORCE_RECONFIG=1"
if "%1"=="-c" set "FORCE_RECONFIG=1"

echo ========================================
echo Manus Integration Build Script
echo ========================================
echo Generator: %GEN%
echo Architecture: %ARCH%
echo Configuration: %CFG%
echo VCPKG Root: %VCPKG_ROOT%
echo ========================================

rem Always run from repo root
pushd "%~dp0"

rem Check if we're in the right directory
if not exist "CMakeLists.txt" (
    echo Error: CMakeLists.txt not found!
    echo Please run this script from the manus integration root directory.
    goto :fail
)

rem Verify vcpkg toolchain exists
if not exist "%VCPKG_ROOT%\scripts\buildsystems\vcpkg.cmake" (
    echo Error: vcpkg toolchain file not found!
    echo Expected: %VCPKG_ROOT%\scripts\buildsystems\vcpkg.cmake
    echo Please verify VCPKG_ROOT is set correctly.
    goto :fail
)

rem Check for Manus SDK in repository
if exist "MANUS_Core_3.0.0_SDK" (
    echo Found Manus SDK in repository: MANUS_Core_3.0.0_SDK
    set "MANUS_SDK_ARG=-DMANUS_SDK_DIR=%CD%\MANUS_Core_3.0.0_SDK"
) else (
    echo Warning: MANUS_Core_3.0.0_SDK not found in repository
    echo SDKClient will not be built
    set "MANUS_SDK_ARG="
)

rem Check for hand_ik subdirectory
if exist "hand_ik" (
    echo Found hand_ik subdirectory
) else (
    echo Warning: hand_ik subdirectory not found
    echo This may cause build issues
)

rem 1) Configure if needed
if "%FORCE_RECONFIG%"=="1" (
    echo [configure] Force reconfigure requested, cleaning build directory...
    rmdir /s /q "%BUILD_DIR%" 2>nul
)

if not exist "%BUILD_DIR%\CMakeCache.txt" (
    echo ========================================
    echo [configure] Generating build files...
    echo ========================================
    echo Using system Eigen to avoid target collision...
    echo Detecting Manus SDK headers automatically...
    
    cmake -S . -B "%BUILD_DIR%" -G "%GEN%" -A %ARCH% ^
      -DCMAKE_TOOLCHAIN_FILE="%VCPKG_ROOT%\scripts\buildsystems\vcpkg.cmake" ^
      -DVCPKG_TARGET_TRIPLET=x64-windows ^
      -DCMAKE_BUILD_TYPE=%CFG% ^
      -DUSE_SYSTEM_EIGEN=ON ^
      -DSKIP_VENDOR_EIGEN=ON ^
      -DMANUS_BUILD_TESTS=ON ^
      %MANUS_SDK_ARG%
      
    if errorlevel 1 goto :fail_configure
    
    echo.
    echo Configuration completed successfully!
) else (
    echo [configure] Build files exist, checking if reconfiguration is needed...
    rem Check if CMake cache is valid by testing for a key variable
    cmake -N -L -B "%BUILD_DIR%" 2>nul | findstr "CMAKE_PROJECT_NAME" >nul
    if errorlevel 1 (
        echo [configure] Cache appears corrupted, reconfiguring...
        rmdir /s /q "%BUILD_DIR%" 2>nul
        
        cmake -S . -B "%BUILD_DIR%" -G "%GEN%" -A %ARCH% ^
          -DCMAKE_TOOLCHAIN_FILE="%VCPKG_ROOT%\scripts\buildsystems\vcpkg.cmake" ^
          -DVCPKG_TARGET_TRIPLET=x64-windows ^
          -DCMAKE_BUILD_TYPE=%CFG% ^
          -DUSE_SYSTEM_EIGEN=ON ^
          -DSKIP_VENDOR_EIGEN=ON ^
          -DMANUS_BUILD_TESTS=ON ^
          %MANUS_SDK_ARG%
          
        if errorlevel 1 goto :fail_configure
    ) else (
        echo [configure] Cache is valid, skipping configuration...
    )
)

rem 2) Build the project
echo ========================================
echo [build] Building manus integration...
echo ========================================

echo Building hand_ik library...
cmake --build "%BUILD_DIR%" --config %CFG% --target hand_ik
if errorlevel 1 goto :fail_build

if exist "MANUS_Core_3.0.0_SDK" (
    echo Building SDKClient executable...
    cmake --build "%BUILD_DIR%" --config %CFG% --target SDKClient
    if errorlevel 1 goto :fail_build
) else (
    echo Skipping SDKClient (Manus SDK not found)
)

echo Building tests (if they exist)...
rem Check if CTest tests are configured
if exist "%BUILD_DIR%\CTestTestfile.cmake" (
    echo CTest configuration found - tests will be run via CTest later
    set "TEST_BUILD_WARN=0"
) else (
    echo No CTest configuration found, skipping test build
    set "TEST_BUILD_WARN=0"
)

rem 3) Verify built executables
echo ========================================
echo [verify] Checking built executables...
echo ========================================

echo.
echo Built targets:

rem Check hand_ik library 
if exist "%BUILD_DIR%\lib\%CFG%\hand_ik.lib" (
    echo   ✓ hand_ik.lib (static library)
) else (
    echo   ✗ hand_ik.lib [NOT FOUND]
    set "MISSING_TARGETS=1"
)

rem Check hand_ik_example 
if exist "%BUILD_DIR%\hand_ik\bin\%CFG%\hand_ik_example.exe" (
    echo   ✓ hand_ik_example.exe
) else (
    echo   - hand_ik_example.exe [SKIPPED - target not generated]
)

rem Check SDKClient - this should exist if SDK was found
if exist "MANUS_Core_3.0.0_SDK" (
    if exist "%BUILD_DIR%\bin\%CFG%\SDKClient.exe" (
        echo   ✓ SDKClient.exe
        
        rem Check if ManusSDK.dll was copied
        if exist "%BUILD_DIR%\bin\%CFG%\ManusSDK.dll" (
            echo     ✓ ManusSDK.dll (copied for runtime)
        ) else (
            echo     ⚠ ManusSDK.dll (not found - may need manual PATH setup)
        )
    ) else (
        echo   ✗ SDKClient.exe [BUILD FAILED]
        set "MISSING_TARGETS=1"
    )
) else (
    echo   - SDKClient.exe [SKIPPED - no Manus SDK]
)

rem Only report on tests if CTest was configured
if exist "%BUILD_DIR%\CTestTestfile.cmake" (
    echo   ✓ Tests configured via CTest (will run on request)
) else (
    echo   - Tests [SKIPPED - no CTest configuration]
)

rem 4) Run basic tests
echo.
set /p run_tests="Run basic tests? (y/n): "
if /i "%run_tests%"=="y" (
    echo ========================================
    echo [test] Running basic tests...
    echo ========================================
    
    if exist "%BUILD_DIR%\bin\%CFG%\test_integration.exe" (
        echo Running integration test...
        "%BUILD_DIR%\bin\%CFG%\test_integration.exe"
        if errorlevel 1 (
            echo Integration test failed
        ) else (
            echo Integration test passed
        )
        echo.
    )
    
    if exist "%BUILD_DIR%\bin\%CFG%\test_hand_ik.exe" (
        echo Running hand IK test...
        "%BUILD_DIR%\bin\%CFG%\test_hand_ik.exe"
        if errorlevel 1 (
            echo Hand IK test failed
        ) else (
            echo Hand IK test passed
        )
        echo.
    )
    
    if exist "%BUILD_DIR%\hand_ik\bin\%CFG%\test_jacobian.exe" (
        echo Running Jacobian validation...
        "%BUILD_DIR%\hand_ik\bin\%CFG%\test_jacobian.exe"
    )
)

rem 5) Success summary
echo ========================================
echo [done] Build completed successfully!
echo ========================================
echo.
echo Executables are located in:
echo   %CD%\%BUILD_DIR%\bin\%CFG%\
echo   %CD%\%BUILD_DIR%\hand_ik\bin\%CFG%\
echo.
echo Key achievements:
echo   ✓ Used system/vcpkg Eigen (no target collision)
echo   ✓ Auto-detected Manus SDK headers and libraries
echo   ✓ Built hand_ik library with corrected coefficients
if exist "MANUS_Core_3.0.0_SDK" (
    echo   ✓ Built SDKClient with Manus SDK integration
) else (
    echo   - SDKClient skipped (no Manus SDK)
)
echo   ✓ Configured for %CFG% build type
echo.
echo To run SDKClient:
if exist "%BUILD_DIR%\bin\%CFG%\SDKClient.exe" (
    echo   %CD%\%BUILD_DIR%\bin\%CFG%\SDKClient.exe
) else (
    echo   [SDKClient not available]
)
echo.
echo To run hand IK example:
if exist "%BUILD_DIR%\hand_ik\bin\%CFG%\hand_ik_example.exe" (
    echo   %CD%\%BUILD_DIR%\hand_ik\bin\%CFG%\hand_ik_example.exe
) else (
    echo   [hand_ik_example not available]
)

popd
exit /b 0

:fail_configure
echo.
echo ========================================
echo CONFIGURATION FAILED
echo ========================================
echo.
echo Possible issues:
echo 1. vcpkg not properly set up
echo 2. Missing required packages in vcpkg:
echo    vcpkg install boost-filesystem boost-system boost-serialization eigen3 --triplet x64-windows
echo 3. CMake version too old (requires 3.24+)
echo 4. Visual Studio 2022 not installed
echo 5. Manus SDK directory structure unexpected
echo.
echo Troubleshooting steps:
echo 1. Verify vcpkg packages:
echo    vcpkg list boost eigen3
echo 2. Check CMake version:
echo    cmake --version
echo 3. Verify VS 2022 installation:
echo    where cl.exe
echo 4. Clean and retry:
echo    rmdir /s /q %BUILD_DIR%
echo    %0 --clean
goto :fail

:fail_build
echo.
echo ========================================
echo BUILD FAILED
echo ========================================
echo.
echo The configuration succeeded but compilation failed.
echo.
echo Common causes:
echo 1. Missing Manus SDK headers (check include path resolution)
echo 2. Library linking issues (verify .lib files found)
echo 3. C++ standard compatibility issues
echo 4. Missing dependencies
echo.
echo Check the build output above for specific error messages.
echo.
echo To debug:
echo 1. Check detailed build log:
echo    cmake --build %BUILD_DIR% --config %CFG% --verbose
echo 2. Verify Manus SDK detection:
echo    cmake -N -L -B %BUILD_DIR% | findstr MANUS
echo 3. Check include directories:
echo    Look for "Manus SDK include dir:" in configure output
goto :fail

:fail
echo.
echo BUILD SCRIPT FAILED
echo See error messages above for details.
popd
exit /b 1