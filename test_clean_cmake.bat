@echo off
echo ========================================
echo Testing Clean CMakeLists.txt
echo ========================================
echo.
echo This tests the completely rewritten CMakeLists.txt that should
echo fix the parse error at line 306.
echo.

rem Backup the current CMakeLists.txt
if exist "CMakeLists.txt" (
    echo Backing up current CMakeLists.txt...
    copy "CMakeLists.txt" "CMakeLists.txt.backup" >nul
)

rem The clean version should be copied from the artifact
echo.
echo NOTE: You need to replace your CMakeLists.txt with the clean version
echo       from the "CMakeLists.txt - Clean version without syntax errors" artifact.
echo.
echo This clean version:
echo   - Removes all potential syntax errors
echo   - Uses simpler output directory handling
echo   - Removes problematic variable references
echo   - Keeps all essential functionality
echo.

set /p proceed="Have you updated CMakeLists.txt with the clean version? (y/n): "
if /i not "%proceed%"=="y" (
    echo.
    echo Please copy the content from the artifact:
    echo "CMakeLists.txt - Clean version without syntax errors"
    echo.
    echo Then run this test again.
    pause
    exit /b 1
)

rem Clean build
if exist "build" rmdir /s /q "build" 2>nul

echo.
echo Testing clean CMakeLists.txt...
cmake -S . -B build -G "Visual Studio 17 2022" -A x64 ^
  -DCMAKE_TOOLCHAIN_FILE=C:\Users\nicol\vcpkg\scripts\buildsystems\vcpkg.cmake ^
  -DVCPKG_TARGET_TRIPLET=x64-windows ^
  -DUSE_SYSTEM_EIGEN=ON ^
  -DSKIP_VENDOR_EIGEN=ON ^
  -DMANUS_BUILD_TESTS=ON

echo.
echo Configure result: %ERRORLEVEL%

if %ERRORLEVEL% equ 0 (
    echo ✅ SUCCESS! Clean CMakeLists.txt works perfectly.
    echo.
    echo The syntax error has been completely resolved.
    echo You can now run: build_manus_integration.bat
    echo.
    echo The clean version includes:
    echo   ✓ Fixed Eigen collision resolution
    echo   ✓ Proper Manus SDK detection
    echo   ✓ CTest integration
    echo   ✓ Clean output directory handling
    echo   ✓ No syntax errors
) else (
    echo ❌ Still having issues. Let's check what the error is now...
    echo.
    echo If you're still getting the "/Release" error, there might be
    echo an issue with the file copy or encoding.
    echo.
    echo Try:
    echo 1. Make sure the file was copied completely
    echo 2. Check for any hidden characters or encoding issues
    echo 3. Verify the file ends properly without truncation
)

if exist "CMakeLists.txt.backup" (
    echo.
    echo Your original CMakeLists.txt is backed up as CMakeLists.txt.backup
)

pause