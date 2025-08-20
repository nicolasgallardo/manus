@echo off
echo ========================================
echo Testing ExternalProject Fix
echo ========================================
echo.
echo This tests the fix for:
echo   "Unknown CMake command ExternalProject_Add"
echo.

rem Clean build
if exist "build" rmdir /s /q "build" 2>nul

echo Running configure with ExternalProject fix...
echo.

cmake -S . -B build -G "Visual Studio 17 2022" -A x64 ^
  -DCMAKE_TOOLCHAIN_FILE=C:\Users\nicol\vcpkg\scripts\buildsystems\vcpkg.cmake ^
  -DVCPKG_TARGET_TRIPLET=x64-windows ^
  -DUSE_SYSTEM_EIGEN=ON ^
  -DSKIP_VENDOR_EIGEN=ON ^
  -DMANUS_BUILD_TESTS=ON

echo.
echo Configure result: %ERRORLEVEL%

if %ERRORLEVEL% equ 0 (
    echo ✅ SUCCESS! ExternalProject fix worked.
    echo Configure completed successfully.
    echo.
    echo You can now run: build_manus_integration.bat
) else (
    echo ❌ Configure still failing.
    echo Check the error messages above for the new issue.
)

pause