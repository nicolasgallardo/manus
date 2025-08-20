@echo off
echo ========================================
echo Testing CMake Syntax Fix
echo ========================================
echo.
echo Testing the fix for:
echo   "Parse error. Expected a command name, got unquoted argument with text '/Release'"
echo.

rem Clean build
if exist "build" rmdir /s /q "build" 2>nul

echo Running configure to test syntax...
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
    echo ✅ SUCCESS! CMake syntax error has been fixed.
    echo Configure completed successfully.
    echo.
    echo You can now run: build_manus_integration.bat
) else (
    echo ❌ Configure still failing.
    echo.
    echo Let's check what the error is now...
    echo Looking for specific error patterns...
    
    rem Try a simpler configure to isolate the issue
    echo.
    echo Trying simplified configure...
    cmake -S . -B build_simple -DUSE_SYSTEM_EIGEN=ON -DMANUS_BUILD_TESTS=OFF
    
    if %ERRORLEVEL% equ 0 (
        echo ✅ Simplified configure works - issue is with vcpkg or test configuration
    ) else (
        echo ❌ Basic syntax issue remains
    )
)

pause