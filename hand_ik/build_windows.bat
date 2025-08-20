@echo off
setlocal EnableExtensions EnableDelayedExpansion

rem === Configurable ===
set "GEN=Visual Studio 17 2022"
set "ARCH=x64"
set "CFG=Release"
set "BUILD_DIR=build"
set "VCPKG_ROOT=C:\Users\nicol\vcpkg"

rem Check for force reconfigure flag
set "FORCE_RECONFIG=0"
if "%1"=="--clean" set "FORCE_RECONFIG=1"
if "%1"=="-c" set "FORCE_RECONFIG=1"

rem Use ASCII to avoid mojibake
echo ========================================
echo Hand IK Windows Build Script
echo ========================================

rem Always run from repo root
pushd "%~dp0"

rem Check if we're in the right directory
if not exist "CMakeLists.txt" (
    echo Error: CMakeLists.txt not found!
    echo Please run this script from the hand_ik root directory.
    goto :fail
)

rem 1) Configure if needed
if "%FORCE_RECONFIG%"=="1" (
    echo [configure] Force reconfigure requested, cleaning build directory...
    rmdir /s /q "%BUILD_DIR%" 2>nul
)

if not exist "%BUILD_DIR%\CMakeCache.txt" (
    echo ========================================
    echo [configure] Generating build files...
    echo ========================================
    cmake -S . -B "%BUILD_DIR%" -G "%GEN%" -A %ARCH% ^
      -DCMAKE_TOOLCHAIN_FILE="%VCPKG_ROOT%\scripts\buildsystems\vcpkg.cmake" ^
      -DCMAKE_BUILD_TYPE=%CFG%
    if errorlevel 1 goto :fail_configure
) else (
    echo [configure] Build files exist, checking if reconfiguration is needed...
    rem Check if CMake cache is valid by testing for a key variable
    cmake -N -L -B "%BUILD_DIR%" 2>nul | findstr "CMAKE_PROJECT_NAME" >nul
    if errorlevel 1 (
        echo [configure] Cache appears corrupted, reconfiguring...
        rmdir /s /q "%BUILD_DIR%" 2>nul
        cmake -S . -B "%BUILD_DIR%" -G "%GEN%" -A %ARCH% ^
          -DCMAKE_TOOLCHAIN_FILE="%VCPKG_ROOT%\scripts\buildsystems\vcpkg.cmake" ^
          -DCMAKE_BUILD_TYPE=%CFG%
        if errorlevel 1 goto :fail_configure
    ) else (
        echo [configure] Cache is valid, skipping configuration...
    )
)

rem 2) Build externals (multi-config aware)
echo ========================================
echo [externals] Building staged dependencies...
echo ========================================
echo This may take several minutes on first build...

echo Building TinyXML2...
cmake --build "%BUILD_DIR%" --config %CFG% --target tinyxml2_ext
if errorlevel 1 goto :fail_externals

echo Building urdfdom headers...
cmake --build "%BUILD_DIR%" --config %CFG% --target urdfdom_headers_ext
if errorlevel 1 goto :fail_externals

echo Building urdfdom...
cmake --build "%BUILD_DIR%" --config %CFG% --target urdfdom_ext
if errorlevel 1 goto :fail_externals

echo Building Pinocchio...
cmake --build "%BUILD_DIR%" --config %CFG% --target pinocchio_ext
if errorlevel 1 goto :fail_externals

rem 3) Post-external sanity check (informational, using wildcards)
echo ========================================
echo [externals] Verifying staged libs and headers...
echo ========================================

set "STAGE_OK=1"

echo Checking TinyXML2 staging...
if exist "%BUILD_DIR%\stage\tinyxml2\lib" (
    echo   Libraries:
    dir /b "%BUILD_DIR%\stage\tinyxml2\lib\*tinyxml2*.lib" 2>nul
    if errorlevel 1 (
        echo     No tinyxml2 libraries found
        set "STAGE_OK=0"
    )
) else (
    echo   TinyXML2 lib directory missing
    set "STAGE_OK=0"
)

if exist "%BUILD_DIR%\stage\tinyxml2\include" (
    echo   Headers:
    dir /b "%BUILD_DIR%\stage\tinyxml2\include\*.h" 2>nul
    if errorlevel 1 (
        echo     No headers found
        set "STAGE_OK=0"
    )
) else (
    echo   TinyXML2 include directory missing
    set "STAGE_OK=0"
)

echo Checking urdfdom staging...
if exist "%BUILD_DIR%\stage\urdfdom\lib" (
    echo   Libraries:
    dir /b "%BUILD_DIR%\stage\urdfdom\lib\*urdfdom*.lib" 2>nul
    if errorlevel 1 (
        echo     No urdfdom libraries found
        set "STAGE_OK=0"
    )
) else (
    echo   urdfdom lib directory missing
    set "STAGE_OK=0"
)

if exist "%BUILD_DIR%\stage\urdfdom\include" (
    echo   Headers: (checking for urdf subdirectory)
    if exist "%BUILD_DIR%\stage\urdfdom\include\urdf" (
        dir /b "%BUILD_DIR%\stage\urdfdom\include\urdf\*.h*" 2>nul
        if errorlevel 1 echo     No urdf headers found
    ) else (
        echo     urdf header subdirectory missing
        set "STAGE_OK=0"
    )
) else (
    echo   urdfdom include directory missing
    set "STAGE_OK=0"
)

echo Checking Pinocchio staging...
if exist "%BUILD_DIR%\stage\pinocchio\lib" (
    echo   Libraries:
    dir /b "%BUILD_DIR%\stage\pinocchio\lib\*pinocchio*.lib" 2>nul
    if errorlevel 1 (
        echo     No pinocchio libraries found
        set "STAGE_OK=0"
    )
) else (
    echo   Pinocchio lib directory missing
    set "STAGE_OK=0"
)

if exist "%BUILD_DIR%\stage\pinocchio\include" (
    echo   Headers: (checking for pinocchio subdirectory)
    if exist "%BUILD_DIR%\stage\pinocchio\include\pinocchio" (
        echo     Pinocchio headers directory exists
    ) else (
        echo     pinocchio header subdirectory missing
        set "STAGE_OK=0"
    )
) else (
    echo   Pinocchio include directory missing
    set "STAGE_OK=0"
)

if "%STAGE_OK%"=="0" (
    echo.
    echo WARNING: Some staged libraries or headers are missing!
    echo This may cause linking or compilation issues, but continuing anyway...
    echo.
) else (
    echo.
    echo All external dependencies appear to be staged correctly.
    echo.
)

rem 4) Build library and executables
echo ========================================
echo [apps] Building library and executables...
echo ========================================

cmake --build "%BUILD_DIR%" --config %CFG% --target hand_ik hand_ik_example test_jacobian test_reachability validate_coupling
if errorlevel 1 goto :fail_apps

rem 5) Check that executables were built
echo ========================================
echo Build completed successfully!
echo ========================================

echo.
echo Executables are located in:
if exist "%BUILD_DIR%\bin\%CFG%\hand_ik_example.exe" (
    echo   %CD%\%BUILD_DIR%\bin\%CFG%\hand_ik_example.exe [OK]
) else (
    echo   hand_ik_example.exe [NOT FOUND]
)

if exist "%BUILD_DIR%\bin\%CFG%\validate_coupling.exe" (
    echo   %CD%\%BUILD_DIR%\bin\%CFG%\validate_coupling.exe [OK]
) else (
    echo   validate_coupling.exe [NOT FOUND]
)

if exist "%BUILD_DIR%\bin\%CFG%\test_jacobian.exe" (
    echo   %CD%\%BUILD_DIR%\bin\%CFG%\test_jacobian.exe [OK]
) else (
    echo   test_jacobian.exe [NOT FOUND]
)

if exist "%BUILD_DIR%\bin\%CFG%\test_reachability.exe" (
    echo   %CD%\%BUILD_DIR%\bin\%CFG%\test_reachability.exe [OK]
) else (
    echo   test_reachability.exe [NOT FOUND]
)

rem Check if URDF file exists and was auto-copied
echo.
if exist "%BUILD_DIR%\bin\%CFG%\surge_v13_hand_right_pybullet.urdf" (
    echo URDF file auto-copied successfully: surge_v13_hand_right_pybullet.urdf [OK]
) else (
    if exist "surge_v13_hand_right_pybullet.urdf" (
        echo WARNING: URDF file found in root but not auto-copied to executables.
        echo This may indicate an issue with the autocopy_urdf function.
    ) else (
        echo WARNING: URDF file not found!
        echo Please place 'surge_v13_hand_right_pybullet.urdf' in the root directory.
    )
    echo.
)

rem Offer to run tests
set /p run_tests="Run tests now? (y/n): "
if /i "%run_tests%"=="y" (
    echo.
    echo ========================================
    echo Running Tests...
    echo ========================================
    
    echo Running Passive Coupling Validation...
    "%BUILD_DIR%\bin\%CFG%\validate_coupling.exe"
    
    echo.
    echo Running Jacobian test...
    "%BUILD_DIR%\bin\%CFG%\test_jacobian.exe"
    
    echo.
    echo Running Reachability test...
    "%BUILD_DIR%\bin\%CFG%\test_reachability.exe"
    
    echo.
    echo Running Example...
    "%BUILD_DIR%\bin\%CFG%\hand_ik_example.exe"
)

echo.
echo [done] Build script completed successfully!
echo [done] Artifacts under: %CD%\%BUILD_DIR%\bin\%CFG%
echo.
echo Note: First build downloads and compiles dependencies.
echo       Subsequent builds will be much faster.

popd
exit /b 0

:fail_configure
echo.
echo Configuration failed!
echo.
echo Possible issues:
echo 1. CMake not found in PATH
echo 2. Visual Studio 2022 not installed
echo 3. vcpkg toolchain file not found at: %VCPKG_ROOT%\scripts\buildsystems\vcpkg.cmake
echo 4. Missing vcpkg packages (boost, eigen3)
echo 5. Corrupted CMake cache - try deleting build directory
echo.
echo Try installing vcpkg dependencies:
echo   vcpkg install boost-filesystem boost-system boost-serialization eigen3 --triplet x64-windows
echo.
echo Or force clean and reconfigure:
echo   rmdir /s /q %BUILD_DIR%
echo   cmake -S . -B %BUILD_DIR% -G "%GEN%" -A %ARCH% -DCMAKE_TOOLCHAIN_FILE="%VCPKG_ROOT%\scripts\buildsystems\vcpkg.cmake"
goto :fail

:fail_externals
echo.
echo External dependency build failed!
echo.
echo The most common cause is a corrupted CMake cache.
echo.
echo Troubleshooting steps:
echo 1. Clean and reconfigure (RECOMMENDED):
echo    rmdir /s /q %BUILD_DIR%
echo    %0
echo.
echo 2. Clean specific dependency and retry:
echo    rmdir /s /q %BUILD_DIR%\tinyxml2_ext-prefix
echo    rmdir /s /q %BUILD_DIR%\stage\tinyxml2
echo    cmake --build %BUILD_DIR% --target tinyxml2_ext --config %CFG%
echo.
echo 3. If vcpkg issues, verify packages are installed:
echo    vcpkg list boost eigen3
echo    vcpkg install boost-filesystem boost-system boost-serialization eigen3 --triplet x64-windows
echo.
echo 4. Verify vcpkg toolchain file exists:
echo    dir "%VCPKG_ROOT%\scripts\buildsystems\vcpkg.cmake"
goto :fail

:fail_apps
echo.
echo Application/test build failed!
echo.
echo This usually means:
echo 1. External dependencies didn't stage properly - check CMakeLists.txt imported targets
echo 2. LNK1181 errors - likely due to raw library names instead of imported targets
echo 3. Missing include files or link libraries
echo 4. Source code compilation errors
echo.
echo Try checking that all external dependencies built successfully:
echo   dir %BUILD_DIR%\stage\tinyxml2\lib\
echo   dir %BUILD_DIR%\stage\urdfdom\lib\
echo   dir %BUILD_DIR%\stage\pinocchio\lib\
echo.
echo If libraries are missing, rebuild external dependencies first:
echo   cmake --build %BUILD_DIR% --config %CFG% --target tinyxml2_ext urdfdom_headers_ext urdfdom_ext pinocchio_ext
echo   cmake --build %BUILD_DIR% --config %CFG%
echo.
echo If LNK1181 errors occur, ensure CMakeLists.txt uses imported targets:
echo   target_link_libraries(hand_ik PUBLIC pinocchio::pinocchio urdfdom::urdfdom_model tinyxml2::tinyxml2)
echo   NOT: target_link_libraries(hand_ik PUBLIC tinyxml2.lib urdfdom_model.lib pinocchio.lib)
echo.
echo Check the build output above for specific error messages.
goto :fail

:fail
echo.
echo BUILD FAILED
popd
exit /b 1