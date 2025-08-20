@echo off
echo ========================================
echo Quick Configure Test
echo ========================================
echo.
echo Testing configure with Eigen fixes applied...
echo This will show the actual error that's causing configure to fail.
echo.

rem Clean build
if exist "build" rmdir /s /q "build" 2>nul

echo Running configure...
echo.

rem Run configure and show output directly (no redirection)
cmake -S . -B build -G "Visual Studio 17 2022" -A x64 ^
  -DCMAKE_TOOLCHAIN_FILE=C:\Users\nicol\vcpkg\scripts\buildsystems\vcpkg.cmake ^
  -DVCPKG_TARGET_TRIPLET=x64-windows ^
  -DUSE_SYSTEM_EIGEN=ON ^
  -DSKIP_VENDOR_EIGEN=ON ^
  -DMANUS_BUILD_TESTS=ON

echo.
echo Configure exit code: %ERRORLEVEL%
echo.

if %ERRORLEVEL% equ 0 (
    echo ✅ SUCCESS! Configure is now working.
    echo You can run: build_manus_integration.bat
) else (
    echo ❌ Configure failed. Check the error messages above.
    echo.
    echo Most likely causes (now that Eigen is fixed):
    echo   1. Missing vcpkg packages (boost, pinocchio, etc.)
    echo   2. Boost configuration issues
    echo   3. hand_ik subdirectory CMake issues
    echo   4. Manus SDK configuration problems
)

pause