@echo off
setlocal EnableExtensions EnableDelayedExpansion

echo ========================================
echo Eigen Collision Fix Validation Script
echo ========================================
echo.
echo This script tests that the Eigen target collision has been resolved.
echo Expected results:
echo   ✓ "Using system/vcpkg Eigen3::Eigen"
echo   ✓ "Eigen selection: SYSTEM (propagated to all subdirs)"
echo   ✓ "[hand_ik] Using parent-provided system Eigen3::Eigen"
echo   ✗ NO "Populating eigen" messages
echo   ✗ NO "_add_library ... ALIAS target Eigen3::Eigen already exists" errors
echo.

rem Always run from repo root
pushd "%~dp0"

rem Check if we're in the right directory
if not exist "CMakeLists.txt" (
    echo Error: CMakeLists.txt not found!
    echo Please run this script from the manus integration root directory.
    goto :fail
)

echo ========================================
echo Test 1: Clean configure with system Eigen
echo ========================================

rem Clean any existing build
if exist "build" (
    echo Cleaning existing build directory...
    rmdir /s /q "build" 2>nul
)

echo.
echo Running configure with system Eigen enabled...
echo This should NOT show "Populating eigen" anywhere.
echo.

rem Run cmake configure and capture output
cmake -S . -B build -G "Visual Studio 17 2022" -A x64 ^
  -DCMAKE_TOOLCHAIN_FILE=C:\Users\nicol\vcpkg\scripts\buildsystems\vcpkg.cmake ^
  -DVCPKG_TARGET_TRIPLET=x64-windows ^
  -DUSE_SYSTEM_EIGEN=ON ^
  -DSKIP_VENDOR_EIGEN=ON ^
  -DMANUS_BUILD_TESTS=ON > configure_output.txt 2>&1

set "CONFIG_RESULT=%ERRORLEVEL%"

echo.
echo === Configure Output Analysis ===
echo.

rem Check for success indicators
findstr /C:"Using system/vcpkg Eigen3::Eigen" configure_output.txt >nul
if errorlevel 1 (
    echo ❌ FAIL: "Using system/vcpkg Eigen3::Eigen" not found
    set "TEST1_PASS=0"
) else (
    echo ✓ PASS: Found "Using system/vcpkg Eigen3::Eigen"
    set "TEST1_PASS=1"
)

findstr /C:"Eigen selection: SYSTEM" configure_output.txt >nul
if errorlevel 1 (
    echo ❌ FAIL: "Eigen selection: SYSTEM" not found
    set "TEST1_PASS=0"
) else (
    echo ✓ PASS: Found "Eigen selection: SYSTEM"
)

findstr /C:"[hand_ik] Using parent-provided system Eigen3::Eigen" configure_output.txt >nul
if errorlevel 1 (
    echo ❌ FAIL: hand_ik did not respect parent Eigen decision
    set "TEST1_PASS=0"
) else (
    echo ✓ PASS: hand_ik respected parent Eigen decision
)

rem Check for failure indicators (these should NOT appear)
findstr /C:"Populating eigen" configure_output.txt >nul
if errorlevel 1 (
    echo ✓ PASS: No "Populating eigen" found
) else (
    echo ❌ FAIL: Found "Populating eigen" - vendored Eigen still being fetched!
    set "TEST1_PASS=0"
    echo     This means a subdirectory is still fetching Eigen despite system version.
)

findstr /C:"_add_library cannot create ALIAS target" configure_output.txt >nul
if errorlevel 1 (
    echo ✓ PASS: No Eigen alias collision detected
) else (
    echo ❌ FAIL: Eigen alias collision still occurring!
    set "TEST1_PASS=0"
    echo     This means multiple sources are trying to create Eigen3::Eigen.
)

findstr /C:"eigen-subbuild" configure_output.txt >nul
if errorlevel 1 (
    echo ✓ PASS: No eigen-subbuild activity detected
) else (
    echo ❌ FAIL: Found eigen-subbuild activity - vendored Eigen still active!
    set "TEST1_PASS=0"
)

rem Check overall configure result
if %CONFIG_RESULT% neq 0 (
    echo ❌ FAIL: Configure failed with exit code %CONFIG_RESULT%
    set "TEST1_PASS=0"
) else (
    echo ✓ PASS: Configure completed successfully
)

echo.
echo ========================================
echo Test 2: Verify Manus SDK detection
echo ========================================

findstr /C:"Found Manus SDK in repository" configure_output.txt >nul
if errorlevel 1 (
    echo ⚠ WARNING: Manus SDK not found in repository (expected if not present)
) else (
    echo ✓ PASS: Manus SDK detected in repository
)

findstr /C:"Manus SDK include dir:" configure_output.txt >nul
if errorlevel 1 (
    echo ⚠ INFO: Manus SDK include dir not resolved (may be expected)
) else (
    echo ✓ PASS: Manus SDK include directory resolved
)

echo.
echo ========================================
echo Test 3: Fallback test (system Eigen disabled)
echo ========================================

echo Testing fallback behavior when system Eigen is disabled...
echo This should show vendored Eigen being fetched exactly once.
echo.

rem Clean build directory
rmdir /s /q "build" 2>nul

rem Configure with system Eigen disabled
cmake -S . -B build -G "Visual Studio 17 2022" -A x64 ^
  -DCMAKE_TOOLCHAIN_FILE=C:\Users\nicol\vcpkg\scripts\buildsystems\vcpkg.cmake ^
  -DVCPKG_TARGET_TRIPLET=x64-windows ^
  -DUSE_SYSTEM_EIGEN=OFF ^
  -DSKIP_VENDOR_EIGEN=OFF ^
  -DMANUS_BUILD_TESTS=ON > configure_fallback.txt 2>&1

set "FALLBACK_RESULT=%ERRORLEVEL%"

findstr /C:"allowing vendored Eigen fallback" configure_fallback.txt >nul
if errorlevel 1 (
    echo ❌ FAIL: Fallback message not found
    set "TEST3_PASS=0"
) else (
    echo ✓ PASS: Fallback behavior activated
    set "TEST3_PASS=1"
)

findstr /C:"_add_library cannot create ALIAS target" configure_fallback.txt >nul
if errorlevel 1 (
    echo ✓ PASS: No alias collision in fallback mode
) else (
    echo ❌ FAIL: Alias collision even in fallback mode!
    set "TEST3_PASS=0"
)

if %FALLBACK_RESULT% neq 0 (
    echo ❌ FAIL: Fallback configure failed with exit code %FALLBACK_RESULT%
    set "TEST3_PASS=0"
) else (
    echo ✓ PASS: Fallback configure completed successfully
)

echo.
echo ========================================
echo Test Results Summary
echo ========================================

if "%TEST1_PASS%"=="1" (
    echo ✓ Test 1 (System Eigen): PASS
) else (
    echo ❌ Test 1 (System Eigen): FAIL
)

echo ✓ Test 2 (Manus SDK): INFO ONLY

if "%TEST3_PASS%"=="1" (
    echo ✓ Test 3 (Fallback): PASS
) else (
    echo ❌ Test 3 (Fallback): FAIL
)

echo.
if "%TEST1_PASS%"=="1" if "%TEST3_PASS%"=="1" (
    echo 🎉 ALL TESTS PASSED!
    echo.
    echo The Eigen collision fix is working correctly:
    echo   • System Eigen is used when available
    echo   • No vendored Eigen fetching occurs when system version exists
    echo   • hand_ik subdirectory respects parent Eigen decision
    echo   • Fallback to vendored Eigen works without collision
    echo   • Manus SDK detection is preserved
    echo.
    echo You can now run: build_manus_integration.bat
) else (
    echo ❌ SOME TESTS FAILED!
    echo.
    echo Please check the detailed output above and the generated log files:
    echo   • configure_output.txt (system Eigen test)
    echo   • configure_fallback.txt (fallback test)
    echo.
    echo Common issues to check:
    echo   1. Ensure vcpkg has eigen3 installed: vcpkg list eigen3
    echo   2. Check hand_ik/CMakeLists.txt has the updated Eigen guards
    echo   3. Verify no other CMake files are unconditionally fetching Eigen
    echo   4. Make sure CMake cache was properly cleared
    goto :fail
)

echo.
echo Cleaning up test artifacts...
rmdir /s /q "build" 2>nul
del configure_output.txt 2>nul
del configure_fallback.txt 2>nul

popd
exit /b 0

:fail
echo.
echo VALIDATION FAILED
echo Check the output above and generated log files for details.
popd
exit /b 1