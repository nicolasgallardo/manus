@echo off
echo ========================================
echo Checking for remaining M_PI usage...
echo ========================================

echo.
echo Searching in source files...
findstr /r /n "M_PI" src\*.cpp src\*.c include\*.hpp include\*.h tests\*.cpp 2>nul
if errorlevel 1 (
    echo No M_PI usage found in source files ✓
) else (
    echo Found M_PI usage ❌ - needs fixing
)

echo.
echo Searching for degree/radian conversion patterns...
findstr /r /n "\* *3\.14159\|/ *3\.14159\|\* *M_PI\|/ *M_PI" src\*.cpp include\*.hpp tests\*.cpp 2>nul
if errorlevel 1 (
    echo No hardcoded Pi usage found ✓
) else (
    echo Found hardcoded Pi usage ❌ - consider using degToRad/radToDeg
)

echo.
echo Checking that math_constants.hpp is included where needed...
findstr /l "#include \"math_constants.hpp\"" src\*.cpp tests\*.cpp 2>nul
if errorlevel 1 (
    echo WARNING: math_constants.hpp may not be included in some files
) else (
    echo math_constants.hpp includes found ✓
)

echo.
echo ========================================
echo Check complete
echo ========================================