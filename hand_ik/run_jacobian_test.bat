@echo off
echo ========================================
echo Testing Jacobian Fix
echo ========================================

rem Build the test
cmake --build build --config Release --target test_jacobian
if errorlevel 1 (
    echo Build failed!
    exit /b 1
)

echo.
echo Running Jacobian test with offset targets fix...
echo This should now show non-zero analytical Jacobian norms.
echo.

build\bin\Release\test_jacobian.exe

echo.
echo ========================================
echo Analysis of Results
echo ========================================
echo.
echo EXPECTED with the fix:
echo - Base residual norm should be ~0.01 (1cm offset)
echo - Both analytic_norm and fd_norm should be non-zero (~1e-2 to 1e-1)
echo - Max error should be ~1e-6 instead of ~1e-1
echo - Jacobian test result should show PASS
echo.
echo If still failing:
echo - Check verbose output for "Building analytical Jacobian" debug info
echo - Verify frame Jacobians (J_mcp, J_distal) have reasonable norms
echo - Check chain rule calculation (df_dq values)
echo - Verify plane projection isn't zeroing everything out