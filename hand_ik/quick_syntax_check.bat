@echo off
echo ========================================
echo Quick Syntax Check
echo ========================================

rem Just try to build the hand_ik library target to check for syntax errors
echo Building hand_ik library to check syntax...

cmake --build build --config Release --target hand_ik
if errorlevel 1 (
    echo.
    echo ❌ Syntax errors found! 
    echo Check the error output above for specific line numbers and issues.
    echo.
    echo Common C++ syntax issues to check:
    echo - Missing semicolons
    echo - Unmatched braces ^{ ^}
    echo - Missing method implementations
    echo - Incorrect namespace/class scope
    echo - Missing include directives
    echo.
    exit /b 1
) else (
    echo.
    echo ✅ Syntax check PASSED!
    echo hand_ik library compiled successfully.
    echo Ready to test the critical solver fixes.
    echo.
    echo Run full test: .\test_solver_fixes.bat
    exit /b 0
)