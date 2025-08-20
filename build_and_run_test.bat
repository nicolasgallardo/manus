@echo off
echo ========================================
echo Build and Run Hand IK Tests
echo ========================================
echo.

echo Step 1: Building test executables...
echo.

rem Build the specific test targets
cmake --build build --config Release --target test_jacobian test_reachability validate_coupling

if %ERRORLEVEL% neq 0 (
    echo ❌ Test build failed!
    echo.
    echo The tests couldn't be compiled. Check for:
    echo   1. Missing source files in hand_ik/tests/
    echo   2. Compilation errors
    echo   3. Missing dependencies
    echo.
    goto :end
)

echo ✅ Test build completed!
echo.

echo Step 2: Locating test executables...
echo.

rem Find where the tests were actually built
set "TESTS_FOUND=0"

if exist "build\hand_ik\bin\Release\test_jacobian.exe" (
    echo   ✓ test_jacobian.exe found in hand_ik subdirectory
    set "TESTS_FOUND=1"
    set "TEST_DIR=build\hand_ik\bin\Release"
)

if exist "build\bin\Release\test_jacobian.exe" (
    echo   ✓ test_jacobian.exe found in root directory
    set "TESTS_FOUND=1" 
    set "TEST_DIR=build\bin\Release"
)

if "%TESTS_FOUND%"=="0" (
    echo ❌ Test executables not found!
    echo.
    echo Searched in:
    echo   - build\bin\Release\
    echo   - build\hand_ik\bin\Release\
    echo.
    echo The tests may not have compiled successfully.
    goto :end
)

echo.
echo Step 3: Running tests...
echo.

if "%TEST_DIR%"=="build\hand_ik\bin\Release" (
    echo Tests are in hand_ik subdirectory - running directly...
    echo.
    
    pushd "%TEST_DIR%"
    
    echo === Running Jacobian Test ===
    if exist "test_jacobian.exe" (
        test_jacobian.exe
        echo Jacobian test exit code: %ERRORLEVEL%
    )
    
    echo.
    echo === Running Reachability Test ===
    if exist "test_reachability.exe" (
        test_reachability.exe
        echo Reachability test exit code: %ERRORLEVEL%
    )
    
    echo.
    echo === Running Coupling Validation ===
    if exist "validate_coupling.exe" (
        validate_coupling.exe
        echo Coupling validation exit code: %ERRORLEVEL%
    )
    
    popd
    
) else (
    echo Tests are in root directory - using CTest...
    echo.
    
    pushd build
    ctest -C Release --output-on-failure --verbose
    popd
)

echo.
echo ========================================
echo Test Summary
echo ========================================
echo.

echo All hand_ik tests have been executed.
echo.
echo These tests validate:
echo   ✓ Jacobian finite difference accuracy
echo   ✓ Inverse kinematics reachability  
echo   ✓ Passive coupling coefficient validation
echo.

echo For detailed test results, check the output above.
echo.

echo If tests fail, common causes:
echo   1. Missing URDF file (should be auto-copied)
echo   2. Incorrect passive coupling coefficients
echo   3. Numerical precision issues
echo   4. Joint limit violations

:end
echo.
pause