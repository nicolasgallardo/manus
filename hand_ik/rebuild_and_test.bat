@echo off
setlocal EnableExtensions EnableDelayedExpansion

echo ========================================
echo Rebuilding Hand IK Tests and Running
echo ========================================

rem Always run from repo root
pushd "%~dp0"

rem Check if we're in the right directory
if not exist "CMakeLists.txt" (
    echo Error: CMakeLists.txt not found!
    echo Please run this script from the hand_ik root directory.
    goto :fail
)

rem 0) Quick check for remaining M_PI issues
echo ========================================
echo [check] Checking for M_PI issues...
echo ========================================

findstr /r /n "M_PI[^a-zA-Z]" src\*.cpp include\*.hpp tests\*.cpp 2>nul
if not errorlevel 1 (
    echo WARNING: Found M_PI usage that may cause MSVC build errors!
    echo Please replace M_PI with hand_ik::kPi or use degToRad/radToDeg functions.
    echo Continuing build anyway...
    echo.
) else (
    echo No direct M_PI usage found ✓
)

rem 1) Rebuild just the core library and test executables
echo ========================================
echo [rebuild] Building library and tests...
echo ========================================

cmake --build build --config Release --target hand_ik test_jacobian test_reachability validate_coupling
if errorlevel 1 (
    echo Build failed! Check the error messages above.
    echo.
    echo Common MSVC issues:
    echo - M_PI not defined: Use hand_ik::kPi instead
    echo - constexpr initialization failed: Make sure all constexpr variables use compile-time constants
    echo.
    goto :fail
)

echo Build completed successfully!

rem 2) Check that executables exist
echo ========================================
echo [check] Verifying executables...
echo ========================================

set "MISSING_EXES=0"

if not exist "build\bin\Release\test_jacobian.exe" (
    echo ERROR: test_jacobian.exe not found
    set "MISSING_EXES=1"
) else (
    echo OK: test_jacobian.exe
)

if not exist "build\bin\Release\test_reachability.exe" (
    echo ERROR: test_reachability.exe not found
    set "MISSING_EXES=1"
) else (
    echo OK: test_reachability.exe
)

if not exist "build\bin\Release\validate_coupling.exe" (
    echo ERROR: validate_coupling.exe not found
    set "MISSING_EXES=1"
) else (
    echo OK: validate_coupling.exe
)

if "%MISSING_EXES%"=="1" (
    echo Some executables are missing! Build may have failed.
    goto :fail
)

rem 3) Run tests in sequence
echo ========================================
echo [test] Running tests in sequence...
echo ========================================

echo.
echo === Running Passive Coupling Validation ===
build\bin\Release\validate_coupling.exe
set "VALIDATE_RESULT=%ERRORLEVEL%"

echo.
echo === Running Jacobian Test ===
build\bin\Release\test_jacobian.exe
set "JACOBIAN_RESULT=%ERRORLEVEL%"

echo.
echo === Running Reachability Test ===
build\bin\Release\test_reachability.exe
set "REACHABILITY_RESULT=%ERRORLEVEL%"

rem 4) Summary
echo.
echo ========================================
echo [summary] Test Results Summary
echo ========================================

if "%VALIDATE_RESULT%"=="0" (
    echo Passive Coupling Validation: PASS
) else (
    echo Passive Coupling@echo off
setlocal EnableExtensions EnableDelayedExpansion

echo ========================================
echo Rebuilding Hand IK Tests and Running
echo ========================================

rem Always run from repo root
pushd "%~dp0"

rem Check if we're in the right directory
if not exist "CMakeLists.txt" (
    echo Error: CMakeLists.txt not found!
    echo Please run this script from the hand_ik root directory.
    goto :fail
)

rem 1) Rebuild just the core library and test executables
echo ========================================
echo [rebuild] Building library and tests...
echo ========================================

cmake --build build --config Release --target hand_ik test_jacobian test_reachability validate_coupling
if errorlevel 1 (
    echo Build failed! Check the error messages above.
    goto :fail
)

echo Build completed successfully!

rem 2) Check that executables exist
echo ========================================
echo [check] Verifying executables...
echo ========================================

set "MISSING_EXES=0"

if not exist "build\bin\Release\test_jacobian.exe" (
    echo ERROR: test_jacobian.exe not found
    set "MISSING_EXES=1"
) else (
    echo OK: test_jacobian.exe
)

if not exist "build\bin\Release\test_reachability.exe" (
    echo ERROR: test_reachability.exe not found
    set "MISSING_EXES=1"
) else (
    echo OK: test_reachability.exe
)

if not exist "build\bin\Release\validate_coupling.exe" (
    echo ERROR: validate_coupling.exe not found
    set "MISSING_EXES=1"
) else (
    echo OK: validate_coupling.exe
)

if "%MISSING_EXES%"=="1" (
    echo Some executables are missing! Build may have failed.
    goto :fail
)

rem 3) Run tests in sequence
echo ========================================
echo [test] Running tests in sequence...
echo ========================================

echo.
echo === Running Passive Coupling Validation ===
build\bin\Release\validate_coupling.exe
set "VALIDATE_RESULT=%ERRORLEVEL%"

echo.
echo === Running Jacobian Test ===
build\bin\Release\test_jacobian.exe
set "JACOBIAN_RESULT=%ERRORLEVEL%"

echo.
echo === Running Reachability Test ===
build\bin\Release\test_reachability.exe
set "REACHABILITY_RESULT=%ERRORLEVEL%"

rem 4) Summary
echo.
echo ========================================
echo [summary] Test Results Summary
echo ========================================

if "%VALIDATE_RESULT%"=="0" (
    echo Passive Coupling Validation: PASS
) else (
    echo Passive Coupling Validation: FAIL ^(exit code %VALIDATE_RESULT%^)
)

if "%JACOBIAN_RESULT%"=="0" (
    echo Jacobian Test: PASS
) else (
    echo Jacobian Test: FAIL ^(exit code %JACOBIAN_RESULT%^)
)

if "%REACHABILITY_RESULT%"=="0" (
    echo Reachability Test: PASS
) else (
    echo Reachability Test: FAIL ^(exit code %REACHABILITY_RESULT%^)
)

echo.
if "%VALIDATE_RESULT%"=="0" if "%JACOBIAN_RESULT%"=="0" if "%REACHABILITY_RESULT%"=="0" (
    echo 🎉 ALL TESTS PASSED! 🎉
    echo The solver fixes are working correctly.
) else (
    echo ❌ Some tests failed. Check the output above for details.
    echo.
    echo Expected results after fixes:
    echo - Jacobian Test: "Internal Jacobian FD check: PASS" for all configurations
    echo - Reachability Test: Success rate ≥80%% with minimal out-of-plane skipping
    echo - Validate Coupling: Coefficients verification should continue to pass
)

echo.
echo Run individual tests with:
echo   build\bin\Release\test_jacobian.exe
echo   build\bin\Release\test_reachability.exe
echo   build\bin\Release\validate_coupling.exe

popd
exit /b 0

:fail
echo.
echo BUILD OR TEST FAILED
popd
exit /b 1