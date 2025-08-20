@echo off
setlocal EnableExtensions EnableDelayedExpansion

echo ========================================
echo Testing Solver Fixes - Quick Check
echo ========================================

rem Always run from repo root
pushd "%~dp0"

rem Check if we're in the right directory
if not exist "CMakeLists.txt" (
    echo Error: CMakeLists.txt not found!
    echo Please run this script from the hand_ik root directory.
    goto :fail
)

@echo off
setlocal EnableExtensions EnableDelayedExpansion

echo ========================================
echo Testing Solver Fixes - Quick Check
echo ========================================

rem Always run from repo root
pushd "%~dp0"

rem Check if we're in the right directory
if not exist "CMakeLists.txt" (
    echo Error: CMakeLists.txt not found!
    echo Please run this script from the hand_ik root directory.
    goto :fail
)

rem 1) Rebuild just the targets we need
echo ========================================
echo [rebuild] Building with unified residual/Jacobian...
echo ========================================

cmake --build build --config Release --target hand_ik test_jacobian test_reachability
if errorlevel 1 (
    echo Build failed! The unified residual/Jacobian changes may have syntax errors.
    goto :fail
)

echo Build completed successfully!

rem 2) Test Jacobian alignment first (most critical)
echo ========================================
echo [test] Testing Jacobian Finite Difference Alignment...
echo ========================================

echo Running Jacobian test to check for frozen-plane consistency...
build\bin\Release\test_jacobian.exe
set "JACOBIAN_RESULT=%ERRORLEVEL%"

if "%JACOBIAN_RESULT%"=="0" (
    echo ✓ Jacobian Test: PASS - Frozen plane residual/Jacobian alignment successful!
) else (
    echo ❌ Jacobian Test: FAIL - Still seeing residual/Jacobian mismatch
    echo.
    echo This indicates the solver and tests are still using different residual definitions.
    echo Check that:
    echo   1. solve^(^) uses buildPlaneProjectedJacobian with frozen planes
    echo   2. checkJacobianFiniteDiff uses the same frozen planes
    echo   3. No calls to old projectToFlexionPlane in solver path
    goto :fail
)

rem 3) Test reachability improvements
echo.
echo ========================================
echo [test] Testing Reachability with Unified Planes...
echo ========================================

echo Running reachability test to check for convergence improvements...
build\bin\Release\test_reachability.exe
set "REACHABILITY_RESULT=%ERRORLEVEL%"

if "%REACHABILITY_RESULT%"=="0" (
    echo ✓ Reachability Test: PASS - Should see ≥80%% success rate now!
) else (
    echo ❌ Reachability Test: FAIL - May need solver parameter tuning
    echo.
    echo If Jacobian test passed but reachability failed, try:
    echo   1. Check damping parameters ^(damping_init, damping_factor^)
    echo   2. Verify line search is working with frozen planes
    echo   3. Check that plane tolerance is reasonable ^(~5mm^)
)

rem 4) Quick summary
echo.
echo ========================================
echo [summary] Solver Fix Verification
echo ========================================

if "%JACOBIAN_RESULT%"=="0" if "%REACHABILITY_RESULT%"=="0" (
    echo 🎉 SUCCESS! Key fixes are working:
    echo   ✓ Unified residual/Jacobian definition with frozen planes
    echo   ✓ Consistent plane-projected optimization
    echo   ✓ Solver and tests now use identical math
    echo.
    echo Expected improvements:
    echo   - Jacobian FD errors should be ~1e-6 instead of ~1e-1
    echo   - Reachability success rates should be ≥80%% instead of 0%%
    echo   - Final IK errors should be ~1e-3m instead of ~0.3m
) else (
    echo ❌ Some critical issues remain:
    
    if not "%JACOBIAN_RESULT%"=="0" (
        echo   - Jacobian-residual mismatch not fully resolved
        echo   - Solver may still be using different plane definition than tests
    )
    
    if not "%REACHABILITY_RESULT%"=="0" (
        echo   - Convergence issues persist despite Jacobian alignment
        echo   - May need solver parameter tuning or better initial guess
    )
    
    echo.
    echo Next steps:
    echo   1. If Jacobian fails: verify all calls use buildPlaneProjectedJacobian/Residual
    echo   2. If reachability fails: tune damping, check line search, verify targets
)

echo.
echo To run full test suite: .\rebuild_and_test.bat
echo To debug specific issues: build\bin\Release\test_jacobian.exe ^| build\bin\Release\test_reachability.exe

popd
exit /b 0

:fail
echo.
echo CRITICAL SOLVER FIXES FAILED
echo The unified residual/Jacobian approach needs debugging.
popd
exit /b 1