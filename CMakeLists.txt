cmake_minimum_required(VERSION 3.30)
project(ManusHandIKIntegration CXX)

# Set policy CMP0167 to NEW - use BoostConfig.cmake instead of FindBoost.cmake
if(POLICY CMP0167)
    cmake_policy(SET CMP0167 NEW)
endif()

# Strongly prefer Config packages (helps find_dependency() inside other packages too)
set(CMAKE_FIND_PACKAGE_PREFER_CONFIG TRUE)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Build configuration
set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type")

# Build options
option(BUILD_WITH_PINOCCHIO "Build with Pinocchio support" ON)
option(BUILD_MANUS_INTEGRATION "Build Manus integration bridge" ON)
option(BUILD_TESTING "Build tests" ON)

# Output directories for consistent binary placement
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# Multi-config generators (like Visual Studio)
foreach(OUTPUTCONFIG Debug Release RelWithDebInfo MinSizeRel)
    string(TOUPPER ${OUTPUTCONFIG} OUTPUTCONFIG_UPPER)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_${OUTPUTCONFIG_UPPER} ${CMAKE_BINARY_DIR}/bin)
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_${OUTPUTCONFIG_UPPER} ${CMAKE_BINARY_DIR}/lib)
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_${OUTPUTCONFIG_UPPER} ${CMAKE_BINARY_DIR}/lib)
endforeach()

# =============================================================================
# Find Dependencies
# =============================================================================

# Find required packages
find_package(Eigen3 CONFIG REQUIRED)  # Use CONFIG mode, provides Eigen3::Eigen
find_package(Threads REQUIRED)

# If vcpkg toolchain wasn't passed on CLI, optionally help CMake see vcpkg
if(DEFINED ENV{VCPKG_ROOT} AND NOT DEFINED CMAKE_TOOLCHAIN_FILE)
    list(APPEND CMAKE_PREFIX_PATH "$ENV{VCPKG_ROOT}/installed/${VCPKG_TARGET_TRIPLET}/share")
endif()

# Use Boost CONFIG mode (imported targets) - this fixes CMP0167 issue
find_package(Boost CONFIG REQUIRED COMPONENTS filesystem serialization system)

# Find Pinocchio using CONFIG mode
if(BUILD_WITH_PINOCCHIO)
    # First try to find pinocchio via CONFIG
    find_package(pinocchio CONFIG QUIET)
    
    if(pinocchio_FOUND)
        message(STATUS "Found pinocchio via CONFIG: ${pinocchio_VERSION}")
        set(PINOCCHIO_LIBRARIES pinocchio::pinocchio)
    else()
        # Fallback: try your staged build location
        set(PINOCCHIO_STAGE_PATH "C:/Users/nicol/Alt_Bionics/GitLab/surge/firmware/Apps/pinocchio_ik_v2/hand_superbuild/build/stage/pinocchio")
        
        if(EXISTS ${PINOCCHIO_STAGE_PATH})
            message(STATUS "Found staged Pinocchio at: ${PINOCCHIO_STAGE_PATH}")
            
            # Point to the staged Config package
            set(pinocchio_DIR "${PINOCCHIO_STAGE_PATH}/lib/cmake/pinocchio")
            find_package(pinocchio CONFIG QUIET)
            
            if(pinocchio_FOUND)
                message(STATUS "Successfully loaded staged Pinocchio CONFIG")
                set(PINOCCHIO_LIBRARIES pinocchio::pinocchio)
            else()
                message(WARNING "Staged Pinocchio CONFIG not found. Using manual configuration.")
                # Manual fallback
                set(PINOCCHIO_LIBRARIES "${PINOCCHIO_STAGE_PATH}/lib/pinocchio.lib")
                set(PINOCCHIO_INCLUDE_DIRS "${PINOCCHIO_STAGE_PATH}/include")
            endif()
        else()
            message(WARNING "Pinocchio not found. Disabling Pinocchio support.")
            set(BUILD_WITH_PINOCCHIO OFF)
            set(PINOCCHIO_LIBRARIES "")
            set(PINOCCHIO_INCLUDE_DIRS "")
        endif()
    endif()
else()
    message(STATUS "Pinocchio support disabled")
    set(PINOCCHIO_LIBRARIES "")
    set(PINOCCHIO_INCLUDE_DIRS "")
endif()

# =============================================================================
# Manus SDK Configuration (from your repository)
# =============================================================================

# Check if manus_sdk folder exists in your repository
set(MANUS_SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/manus_sdk")

if(EXISTS ${MANUS_SDK_DIR})
    message(STATUS "Found Manus SDK in repository: ${MANUS_SDK_DIR}")
    
    # Set Manus SDK include directories
    set(MANUS_SDK_INCLUDE_DIRS
        ${MANUS_SDK_DIR}/include
        ${MANUS_SDK_DIR}/CoreSDK/include
        ${MANUS_SDK_DIR}/ManusSDK/include
        # Add other include paths as needed based on your manus_sdk structure
    )
    
    # Find Manus SDK libraries
    find_library(MANUS_CORE_LIBRARY
        NAMES CoreSDK ManusCore manus_core
        PATHS ${MANUS_SDK_DIR}/lib 
              ${MANUS_SDK_DIR}/CoreSDK/lib
              ${MANUS_SDK_DIR}/bin
        NO_DEFAULT_PATH
    )
    
    find_library(MANUS_SDK_LIBRARY
        NAMES ManusSDK Manus manus_sdk
        PATHS ${MANUS_SDK_DIR}/lib
              ${MANUS_SDK_DIR}/ManusSDK/lib 
              ${MANUS_SDK_DIR}/bin
        NO_DEFAULT_PATH
    )
    
    # Set up library list
    set(MANUS_SDK_LIBRARIES)
    if(MANUS_CORE_LIBRARY)
        list(APPEND MANUS_SDK_LIBRARIES ${MANUS_CORE_LIBRARY})
        message(STATUS "Found Manus Core library: ${MANUS_CORE_LIBRARY}")
    endif()
    
    if(MANUS_SDK_LIBRARY)
        list(APPEND MANUS_SDK_LIBRARIES ${MANUS_SDK_LIBRARY})
        message(STATUS "Found Manus SDK library: ${MANUS_SDK_LIBRARY}")
    endif()
    
    # If no libraries found, try to find common library names
    if(NOT MANUS_SDK_LIBRARIES)
        message(STATUS "Searching for Manus libraries in ${MANUS_SDK_DIR}")
        file(GLOB_RECURSE MANUS_LIBS 
             "${MANUS_SDK_DIR}/*.lib"
             "${MANUS_SDK_DIR}/*.a" 
             "${MANUS_SDK_DIR}/*.so"
             "${MANUS_SDK_DIR}/*.dll"
        )
        
        if(MANUS_LIBS)
            message(STATUS "Found potential Manus libraries:")
            foreach(LIB ${MANUS_LIBS})
                message(STATUS "  ${LIB}")
            endforeach()
            
            # Filter for likely SDK libraries (you may need to adjust these patterns)
            foreach(LIB ${MANUS_LIBS})
                get_filename_component(LIB_NAME ${LIB} NAME_WE)
                if(LIB_NAME MATCHES ".*[Mm]anus.*" OR 
                   LIB_NAME MATCHES ".*[Cc]ore.*" OR
                   LIB_NAME MATCHES ".*[Ss]dk.*")
                    list(APPEND MANUS_SDK_LIBRARIES ${LIB})
                endif()
            endforeach()
        endif()
    endif()
    
else()
    message(WARNING "Manus SDK directory not found at: ${MANUS_SDK_DIR}")
    message(STATUS "Please ensure your repository contains the manus_sdk folder")
    message(STATUS "Or update MANUS_SDK_DIR in CMakeLists.txt to point to the correct location")
    
    # Fallback: manual configuration
    set(MANUS_SDK_INCLUDE_DIRS 
        # Add your actual Manus SDK include paths here
    )
    set(MANUS_SDK_LIBRARIES
        # Add your actual Manus SDK library names here
    )
endif()

# =============================================================================
# Hand IK Library (Your existing library)
# =============================================================================

# Check if hand_ik directory exists
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/hand_ik")
    message(STATUS "Found Hand IK library directory")
    
    # Option 1: If hand_ik has its own CMakeLists.txt, use it
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/hand_ik/CMakeLists.txt")
        message(STATUS "Using Hand IK's existing CMakeLists.txt")
        add_subdirectory(hand_ik)
    else()
        # Option 2: Build hand_ik manually if no CMakeLists.txt
        message(STATUS "Building Hand IK library manually")
        
        # Collect Hand IK source files
        file(GLOB HAND_IK_SOURCES 
             "${CMAKE_CURRENT_SOURCE_DIR}/hand_ik/src/*.cpp"
             "${CMAKE_CURRENT_SOURCE_DIR}/hand_ik/src/*.c"
        )
        
        if(HAND_IK_SOURCES)
            add_library(hand_ik STATIC ${HAND_IK_SOURCES})
            
            target_include_directories(hand_ik PUBLIC
                ${CMAKE_CURRENT_SOURCE_DIR}/hand_ik/include
            )
            
            # Link dependencies to hand_ik
            target_link_libraries(hand_ik PUBLIC Eigen3::Eigen)
            
            # Add Pinocchio using imported target
            if(BUILD_WITH_PINOCCHIO AND PINOCCHIO_LIBRARIES)
                if(TARGET pinocchio::pinocchio)
                    target_link_libraries(hand_ik PUBLIC pinocchio::pinocchio)
                else()
                    target_link_libraries(hand_ik PUBLIC ${PINOCCHIO_LIBRARIES})
                    if(PINOCCHIO_INCLUDE_DIRS)
                        target_include_directories(hand_ik PRIVATE ${PINOCCHIO_INCLUDE_DIRS})
                    endif()
                endif()
            endif()
            
            # Add Boost using imported targets
            target_link_libraries(hand_ik PRIVATE 
                Boost::filesystem
                Boost::serialization  
                Boost::system
            )
            
            target_compile_features(hand_ik PUBLIC cxx_std_17)
            
            if(WIN32)
                target_compile_definitions(hand_ik PRIVATE NOMINMAX WIN32_LEAN_AND_MEAN)
            endif()
        else()
            message(FATAL_ERROR "No Hand IK source files found in ${CMAKE_CURRENT_SOURCE_DIR}/hand_ik/src/")
        endif()
    endif()
else()
    message(FATAL_ERROR "Hand IK directory not found: ${CMAKE_CURRENT_SOURCE_DIR}/hand_ik")
endif()

# =============================================================================
# Manus Integration Bridge Library
# =============================================================================

add_library(ManusHandIK STATIC
    src/ManusHandIKBridge.cpp
    src/ManusSkeletonSetup.cpp
)

# Include directories
target_include_directories(ManusHandIK PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/hand_ik/include
    ${MANUS_SDK_INCLUDE_DIRS}
)

# Link libraries using imported targets
target_link_libraries(ManusHandIK PUBLIC
    hand_ik                     # Your Hand IK library
    Eigen3::Eigen              # Eigen using imported target
    Threads::Threads            # Threading support for performance monitoring
)

# Add Pinocchio if available
if(BUILD_WITH_PINOCCHIO AND PINOCCHIO_LIBRARIES)
    if(TARGET pinocchio::pinocchio)
        target_link_libraries(ManusHandIK PUBLIC pinocchio::pinocchio)
    else()
        target_link_libraries(ManusHandIK PUBLIC ${PINOCCHIO_LIBRARIES})
        if(PINOCCHIO_INCLUDE_DIRS)
            target_include_directories(ManusHandIK PRIVATE ${PINOCCHIO_INCLUDE_DIRS})
        endif()
    endif()
endif()

# Add Boost using imported targets
target_link_libraries(ManusHandIK PRIVATE
    Boost::filesystem
    Boost::serialization
    Boost::system
)

# Add Manus SDK libraries (when you specify them)
if(MANUS_SDK_LIBRARIES)
    target_link_libraries(ManusHandIK PUBLIC ${MANUS_SDK_LIBRARIES})
endif()

# Compiler features and definitions
target_compile_features(ManusHandIK PUBLIC cxx_std_17)

# Platform-specific settings
if(WIN32)
    target_compile_definitions(ManusHandIK PRIVATE NOMINMAX WIN32_LEAN_AND_MEAN)
endif()

# =============================================================================
# Main Executable (SDKClient)
# =============================================================================

add_executable(SDKClient
    src/SDKClientIntegration.cpp
)

# Link to the integration bridge
target_link_libraries(SDKClient PRIVATE
    ManusHandIK
)

# Include Manus SDK directories for the executable
if(MANUS_SDK_INCLUDE_DIRS)
    target_include_directories(SDKClient PRIVATE ${MANUS_SDK_INCLUDE_DIRS})
endif()

# Platform-specific settings for executable
if(WIN32)
    target_compile_definitions(SDKClient PRIVATE NOMINMAX WIN32_LEAN_AND_MEAN)
endif()

# =============================================================================
# Optional: Surge Integration Library (if you have existing surge code)
# =============================================================================

option(BUILD_SURGE_INTEGRATION "Build optional Surge integration" OFF)

if(BUILD_SURGE_INTEGRATION)
    add_library(SurgeIntegration STATIC
        src/SurgeIntegration.cpp
    )
    
    target_include_directories(SurgeIntegration PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/src
    )
    
    target_link_libraries(SurgeIntegration PUBLIC
        ManusHandIK
    )
    
    target_compile_features(SurgeIntegration PUBLIC cxx_std_17)
endif()

# =============================================================================
# Testing
# =============================================================================

option(BUILD_TESTING "Build tests" ON)

if(BUILD_TESTING)
    enable_testing()
    
    # Find Google Test
    find_package(GTest QUIET)
    
    if(GTest_FOUND OR TARGET gtest)
        add_executable(test_manus_integration
            tests/test_manus_integration.cpp
        )
        
        target_link_libraries(test_manus_integration PRIVATE
            ManusHandIK
            GTest::gtest 
            GTest::gtest_main
        )
        
        # Add the test
        add_test(NAME manus_integration_test 
                 COMMAND test_manus_integration)
                 
        # Set test properties
        set_tests_properties(manus_integration_test PROPERTIES
            WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
        )
        
    else()
        message(WARNING "Google Test not found. Skipping test build.")
        message(STATUS "To install Google Test:")
        message(STATUS "  Ubuntu: sudo apt install libgtest-dev")
        message(STATUS "  macOS: brew install googletest")
        message(STATUS "  Or use FetchContent to download automatically")
    endif()
endif()

# =============================================================================
# URDF File Handling
# =============================================================================

# Copy URDF file to build directory for easy access
set(URDF_CANDIDATES
    "${CMAKE_SOURCE_DIR}/surge_v13_hand_right_pybullet.urdf"
    "${CMAKE_SOURCE_DIR}/hand_ik/surge_v13_hand_right_pybullet.urdf"
)

foreach(URDF_PATH ${URDF_CANDIDATES})
    if(EXISTS ${URDF_PATH})
        configure_file(
            ${URDF_PATH}
            ${CMAKE_BINARY_DIR}/surge_v13_hand_right_pybullet.urdf
            COPYONLY
        )
        message(STATUS "URDF file found and copied: ${URDF_PATH}")
        break()
    endif()
endforeach()

# Also copy to bin directory for executables
foreach(URDF_PATH ${URDF_CANDIDATES})
    if(EXISTS ${URDF_PATH})
        configure_file(
            ${URDF_PATH}
            ${CMAKE_BINARY_DIR}/bin/surge_v13_hand_right_pybullet.urdf
            COPYONLY
        )
        break()
    endif()
endforeach()

# =============================================================================
# Installation (Optional)
# =============================================================================

# Install targets
install(TARGETS SDKClient
    RUNTIME DESTINATION bin
    COMPONENT applications
)

install(TARGETS ManusHandIK
    EXPORT ManusHandIKTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

# Install headers
install(FILES
    src/ManusHandIKBridge.h
    src/ManusSkeletonSetup.h
    DESTINATION include
    COMPONENT development
)

# Install URDF file
foreach(URDF_PATH ${URDF_CANDIDATES})
    if(EXISTS ${URDF_PATH})
        install(FILES ${URDF_PATH}
            DESTINATION share/manus_handik
            COMPONENT data
        )
        break()
    endif()
endforeach()

# =============================================================================
# Configuration Summary
# =============================================================================

message(STATUS "==================================================")
message(STATUS "Manus Hand IK Integration Configuration Summary")
message(STATUS "==================================================")
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "C++ compiler: ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}")
message(STATUS "C++ standard: ${CMAKE_CXX_STANDARD}")

message(STATUS "")
message(STATUS "Dependencies:")
message(STATUS "  Eigen3: ${Eigen3_FOUND}")
message(STATUS "  Boost: ${Boost_FOUND} (using CONFIG)")
if(Boost_FOUND)
    message(STATUS "    Components: filesystem serialization system")
endif()
if(BUILD_WITH_PINOCCHIO)
    if(pinocchio_FOUND)
        message(STATUS "  Pinocchio: Found (CONFIG)")
    elseif(PINOCCHIO_LIBRARIES)
        message(STATUS "  Pinocchio: Found (manual)")
    else()
        message(STATUS "  Pinocchio: Not found")
    endif()
else()
    message(STATUS "  Pinocchio: Disabled")
endif()

message(STATUS "")
message(STATUS "Components:")
message(STATUS "  Hand IK library: ${CMAKE_SOURCE_DIR}/hand_ik")
message(STATUS "  Integration bridge: ManusHandIK")
message(STATUS "  Main executable: SDKClient")
message(STATUS "  Testing: ${BUILD_TESTING}")
if(BUILD_SURGE_INTEGRATION)
    message(STATUS "  Surge integration: Enabled")
endif()

message(STATUS "")
message(STATUS "Manus SDK Configuration:")
if(EXISTS ${MANUS_SDK_DIR})
    message(STATUS "  SDK Directory: ${MANUS_SDK_DIR}")
    if(MANUS_SDK_LIBRARIES)
        message(STATUS "  Libraries Found:")
        foreach(LIB ${MANUS_SDK_LIBRARIES})
            message(STATUS "    ${LIB}")
        endforeach()
    else()
        message(STATUS "  Libraries: NONE FOUND")
        message(STATUS "  -> Check library paths in manus_sdk folder")
    endif()
    
    if(MANUS_SDK_INCLUDE_DIRS)
        message(STATUS "  Include Directories:")
        foreach(DIR ${MANUS_SDK_INCLUDE_DIRS})
            if(EXISTS ${DIR})
                message(STATUS "    ${DIR} ✓")
            else()
                message(STATUS "    ${DIR} ✗ (not found)")
            endif()
        endforeach()
    endif()
else()
    message(STATUS "  SDK Directory: NOT FOUND")
    message(STATUS "  Expected location: ${MANUS_SDK_DIR}")
    message(STATUS "  -> Ensure manus_sdk folder exists in repository root")
endif()

message(STATUS "")
message(STATUS "Build outputs will be in:")
message(STATUS "  Executables: ${CMAKE_BINARY_DIR}/bin/")
message(STATUS "  Libraries: ${CMAKE_BINARY_DIR}/lib/")

message(STATUS "")
message(STATUS "Next steps:")
message(STATUS "1. Configure with vcpkg toolchain:")
message(STATUS "   cmake -S . -B build -G \"Visual Studio 17 2022\" \\")
message(STATUS "     -DCMAKE_TOOLCHAIN_FILE=C:/Users/nicol/vcpkg/scripts/buildsystems/vcpkg.cmake \\")
message(STATUS "     -DVCPKG_TARGET_TRIPLET=x64-windows")
message(STATUS "2. Build: cmake --build build --config Release")
message(STATUS "3. Test: ./build/bin/test_manus_integration")
message(STATUS "4. Run: ./build/bin/SDKClient")
message(STATUS "==================================================")

# =============================================================================
# Development Helpers
# =============================================================================

# Custom target to show configuration info
add_custom_target(show_config
    COMMAND ${CMAKE_COMMAND} -E echo "=== Build Configuration ==="
    COMMAND ${CMAKE_COMMAND} -E echo "Source dir: ${CMAKE_SOURCE_DIR}"
    COMMAND ${CMAKE_COMMAND} -E echo "Binary dir: ${CMAKE_BINARY_DIR}"
    COMMAND ${CMAKE_COMMAND} -E echo "Build type: ${CMAKE_BUILD_TYPE}"
    COMMAND ${CMAKE_COMMAND} -E echo "Compiler: ${CMAKE_CXX_COMPILER_ID}"
    COMMAND ${CMAKE_COMMAND} -E echo ""
    COMMAND ${CMAKE_COMMAND} -E echo "To configure Manus SDK:"
    COMMAND ${CMAKE_COMMAND} -E echo "1. Edit CMakeLists.txt MANUS_SDK_LIBRARIES"
    COMMAND ${CMAKE_COMMAND} -E echo "2. Edit CMakeLists.txt MANUS_SDK_INCLUDE_DIRS"
    COMMAND ${CMAKE_COMMAND} -E echo "3. Replace includes in src/ManusSkeletonSetup.cpp"
    COMMAND ${CMAKE_COMMAND} -E echo "4. Replace includes in src/SDKClientIntegration.cpp"
)

# Custom target to check for missing dependencies
add_custom_target(check_deps
    COMMAND ${CMAKE_COMMAND} -E echo "=== Checking Dependencies ==="
    COMMAND ${CMAKE_COMMAND} -E echo "Eigen3: ${Eigen3_FOUND}"
    COMMAND ${CMAKE_COMMAND} -E echo "Pinocchio: ${PINOCCHIO_FOUND}"
    COMMAND ${CMAKE_COMMAND} -E echo "GTest: ${GTest_FOUND}"
    COMMAND ${CMAKE_COMMAND} -E echo ""
    COMMAND ${CMAKE_COMMAND} -E echo "Missing dependencies can be installed with:"
    COMMAND ${CMAKE_COMMAND} -E echo "  Ubuntu: sudo apt install libeigen3-dev libpinocchio-dev libgtest-dev"
    COMMAND ${CMAKE_COMMAND} -E echo "  macOS: brew install eigen pinocchio googletest"
)