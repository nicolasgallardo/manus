# File: CMakeLists.txt (replace your existing file)

cmake_minimum_required(VERSION 3.24)
project(manus_hand_ik CXX)

set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreadedDLL")
set(BUILD_SHARED_LIBS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_POLICY_VERSION_MINIMUM 3.5)

# Always use fetched sources; never prefer system packages
set(FETCHCONTENT_TRY_FIND_PACKAGE FALSE)
set(FETCHCONTENT_QUIET OFF)

include(FetchContent)
include(ExternalProject)

# Set C++ standard for all targets
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Build options
option(HAND_IK_BUILD_EXAMPLES "Build Hand IK examples" ON)
option(HAND_IK_BUILD_TESTS "Build Hand IK tests" ON)
option(BUILD_MANUS_INTEGRATION "Build Manus Core integration" ON)

# Put all binaries in predictable locations
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")

# Multi-config generators (MSVC): add config subdirs
foreach(OUTPUTCONFIG Debug Release RelWithDebInfo MinSizeRel)
    string(TOUPPER "${OUTPUTCONFIG}" OUTPUTCONFIG_UP)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_${OUTPUTCONFIG_UP} "${CMAKE_BINARY_DIR}/bin/${OUTPUTCONFIG}")
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_${OUTPUTCONFIG_UP} "${CMAKE_BINARY_DIR}/lib/${OUTPUTCONFIG}")
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_${OUTPUTCONFIG_UP} "${CMAKE_BINARY_DIR}/lib/${OUTPUTCONFIG}")
endforeach()

# Set policy to avoid FindBoost warnings
if(POLICY CMP0167)
    cmake_policy(SET CMP0167 NEW)
endif()
if(POLICY CMP0144)
    cmake_policy(SET CMP0144 NEW)
endif()

# =============================================================================
# External Dependencies
# =============================================================================

# ---------------- Eigen (header-only via FetchContent) ----------------
FetchContent_Declare(
  eigen
  GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
  GIT_TAG 3.4.0
)
FetchContent_MakeAvailable(eigen)
set(EIGEN3_INCLUDE_DIR "${eigen_SOURCE_DIR}" CACHE PATH "" FORCE)
set(Eigen3_DIR "" CACHE PATH "" FORCE)

# ---------------- tinyxml2 (ExternalProject with staged install) ----------------
set(TINYXML2_STAGE "${CMAKE_BINARY_DIR}/stage/tinyxml2")

ExternalProject_Add(tinyxml2_ext
  URL https://github.com/leethomason/tinyxml2/archive/refs/tags/11.0.0.tar.gz
  UPDATE_DISCONNECTED 1
  CMAKE_ARGS
    -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
    -DCMAKE_MSVC_RUNTIME_LIBRARY=${CMAKE_MSVC_RUNTIME_LIBRARY}
    -DBUILD_SHARED_LIBS=OFF
    -DCMAKE_INSTALL_PREFIX=${TINYXML2_STAGE}
  BUILD_COMMAND ${CMAKE_COMMAND} --build <BINARY_DIR> --config $<CONFIG>
  INSTALL_COMMAND ${CMAKE_COMMAND} --build <BINARY_DIR> --config $<CONFIG> --target install
  INSTALL_DIR ${TINYXML2_STAGE}
  BUILD_BYPRODUCTS
    ${TINYXML2_STAGE}/lib/tinyxml2.lib
    ${TINYXML2_STAGE}/lib/cmake/tinyxml2/tinyxml2Config.cmake
)

set(TINYXML2_ROOT "${TINYXML2_STAGE}" CACHE PATH "" FORCE)
list(PREPEND CMAKE_PREFIX_PATH "${TINYXML2_STAGE}")
set(CMAKE_PREFIX_PATH "${CMAKE_PREFIX_PATH}" CACHE STRING "" FORCE)

# ---------------- urdfdom_headers 2.0.0 (ExternalProject) ----------------
find_package(Git REQUIRED)

set(URDF_HEADERS_VER 2.0.0)
set(URDF_HEADERS_STAGE "${CMAKE_BINARY_DIR}/stage/urdfdom_headers")

ExternalProject_Add(urdfdom_headers_ext
  URL https://github.com/ros/urdfdom_headers/archive/refs/tags/${URDF_HEADERS_VER}.tar.gz
  UPDATE_DISCONNECTED 1
  CMAKE_ARGS
    -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
    -DCMAKE_MSVC_RUNTIME_LIBRARY=${CMAKE_MSVC_RUNTIME_LIBRARY}
    -DBUILD_SHARED_LIBS=OFF
    -DURDFDOM_HEADERS_BUILD_TESTS=OFF
    -DCMAKE_INSTALL_PREFIX=${URDF_HEADERS_STAGE}
  INSTALL_DIR ${URDF_HEADERS_STAGE}
  BUILD_BYPRODUCTS
    ${URDF_HEADERS_STAGE}/share/urdfdom_headers/urdfdom_headers-config.cmake
)

set(urdfdom_headers_DIR "${URDF_HEADERS_STAGE}/share/urdfdom_headers" CACHE PATH "" FORCE)

# ---------------- urdfdom 4.0.1 (ExternalProject) ----------------
set(URDF_STAGE "${CMAKE_BINARY_DIR}/stage/urdfdom")

ExternalProject_Add(urdfdom_ext
  URL https://github.com/ros/urdfdom/archive/refs/tags/4.0.1.tar.gz
  UPDATE_DISCONNECTED 1
  CMAKE_ARGS
    -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
    -DCMAKE_MSVC_RUNTIME_LIBRARY=${CMAKE_MSVC_RUNTIME_LIBRARY}
    -DBUILD_SHARED_LIBS=OFF
    -DURDFDOM_BUILD_TESTS=OFF
    -DTinyXML2_DIR=${TINYXML2_STAGE}/lib/cmake/tinyxml2
    -DTINYXML2_INCLUDE_DIR=${TINYXML2_STAGE}/include
    -DTINYXML2_LIBRARY=${TINYXML2_STAGE}/lib/tinyxml2.lib
    -Durdfdom_headers_DIR=${urdfdom_headers_DIR}
    -DCMAKE_INSTALL_PREFIX=${URDF_STAGE}
  BUILD_COMMAND ${CMAKE_COMMAND} --build <BINARY_DIR> --config $<CONFIG>
  INSTALL_COMMAND ${CMAKE_COMMAND} --build <BINARY_DIR> --config $<CONFIG> --target install
  INSTALL_DIR ${URDF_STAGE}
  BUILD_BYPRODUCTS
    ${URDF_STAGE}/lib/urdfdom_model.lib
    ${URDF_STAGE}/lib/cmake/urdfdom/urdfdom-config.cmake
)
add_dependencies(urdfdom_ext urdfdom_headers_ext)
add_dependencies(urdfdom_ext tinyxml2_ext)

set(urdfdom_DIR "${URDF_STAGE}/lib/cmake/urdfdom" CACHE PATH "" FORCE)
list(PREPEND CMAKE_PREFIX_PATH "${URDF_STAGE}" "${URDF_HEADERS_STAGE}" "${TINYXML2_STAGE}")
set(CMAKE_PREFIX_PATH "${CMAKE_PREFIX_PATH}" CACHE STRING "" FORCE)

# Create placeholder IMPORTED targets for urdfdom
if(NOT TARGET urdfdom::urdfdom_model)
    add_library(urdfdom::urdfdom_model UNKNOWN IMPORTED)
    set(_URDF_MODEL_RELEASE "${URDF_STAGE}/lib/urdfdom_model.lib")
    set(_URDF_MODEL_DEBUG   "${URDF_STAGE}/lib/urdfdom_modeld.lib")
    
    if(NOT EXISTS "${_URDF_MODEL_DEBUG}")
        set(_URDF_MODEL_DEBUG "${_URDF_MODEL_RELEASE}")
    endif()
    
    set_target_properties(urdfdom::urdfdom_model PROPERTIES
        IMPORTED_CONFIGURATIONS "Debug;Release"
        IMPORTED_LOCATION_RELEASE "${_URDF_MODEL_RELEASE}"
        IMPORTED_LOCATION_DEBUG   "${_URDF_MODEL_DEBUG}"
        INTERFACE_INCLUDE_DIRECTORIES "${URDF_STAGE}/include"
    )
endif()

add_custom_target(urdfdom_stage ALL DEPENDS urdfdom_ext)

# ---------------- Pinocchio (ExternalProject; staged install + submodules) ----------------
set(PIN_VER   2.6.21)
set(PIN_STAGE "${CMAKE_BINARY_DIR}/stage/pinocchio")

# vcpkg prefix for x64-windows
set(VCPKG_X64 "C:/Users/nicol/vcpkg/installed/x64-windows")

ExternalProject_Add(pinocchio_ext
  GIT_REPOSITORY https://github.com/stack-of-tasks/pinocchio.git
  GIT_TAG        v${PIN_VER}
  GIT_SHALLOW    FALSE
  UPDATE_DISCONNECTED 1

  PATCH_COMMAND "${GIT_EXECUTABLE}" submodule update --init --recursive

  CMAKE_ARGS
    -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
    -DCMAKE_MSVC_RUNTIME_LIBRARY=${CMAKE_MSVC_RUNTIME_LIBRARY}
    -DBUILD_SHARED_LIBS=OFF
    -DBUILD_TESTING=OFF
    -DBUILD_PYTHON_INTERFACE=OFF
    -DPINOCCHIO_WITH_HPP_FCL=OFF
    -DPINOCCHIO_WITH_URDFDOM=ON
    -DPINOCCHIO_BUILD_UNIT_TESTS=OFF
    -DPINOCCHIO_BUILD_BENCHMARK=OFF
    -DPINOCCHIO_BUILD_EXAMPLES=OFF
    -DPROJECT_VERSION=${PIN_VER}
    -DEIGEN3_INCLUDE_DIR=${eigen_SOURCE_DIR}
    
    -Durdfdom_DIR:PATH=${URDF_STAGE}/lib/cmake/urdfdom
    -Durdfdom_headers_DIR:PATH=${URDF_HEADERS_STAGE}/share/urdfdom_headers
    -DURDFDOM_INCLUDE_DIR:PATH=${URDF_STAGE}/include
    -DURDFDOM_HEADERS_INCLUDE_DIR:PATH=${URDF_HEADERS_STAGE}/include
    -DCMAKE_INCLUDE_PATH:PATH=${URDF_STAGE}/include;${URDF_HEADERS_STAGE}/include
    -DCMAKE_PREFIX_PATH=${URDF_STAGE};${URDF_HEADERS_STAGE};${TINYXML2_STAGE}
    
    -DTinyXML2_DIR=${TINYXML2_STAGE}/lib/cmake/tinyxml2

    # Boost configuration
    -DBoost_NO_BOOST_CMAKE=ON
    -DBoost_USE_STATIC_LIBS=ON
    -DBoost_USE_MULTITHREADED=ON
    -DBoost_USE_STATIC_RUNTIME=OFF
    -DBoost_NO_SYSTEM_PATHS=ON
    -DBOOST_ROOT=${VCPKG_X64}
    -DBOOST_INCLUDEDIR=${VCPKG_X64}/include
    -DBOOST_LIBRARYDIR=${VCPKG_X64}/lib

    -DCMAKE_INSTALL_PREFIX=${PIN_STAGE}
    -DCMAKE_INSTALL_LIBDIR=lib
    -DCMAKE_INSTALL_BINDIR=bin
    -DCMAKE_INSTALL_INCLUDEDIR=include
  
  BUILD_COMMAND ${CMAKE_COMMAND} --build <BINARY_DIR> --config $<CONFIG>
  INSTALL_COMMAND ${CMAKE_COMMAND} --build <BINARY_DIR> --config $<CONFIG> --target install
  INSTALL_DIR ${PIN_STAGE}
  
  BUILD_BYPRODUCTS
    ${PIN_STAGE}/lib/pinocchio.lib
    ${PIN_STAGE}/lib/cmake/pinocchio/pinocchio-config.cmake
)
add_dependencies(pinocchio_ext urdfdom_ext)

set(Pinocchio_DIR "${PIN_STAGE}/lib/cmake/pinocchio" CACHE PATH "" FORCE)
list(PREPEND CMAKE_PREFIX_PATH "${PIN_STAGE}")
set(CMAKE_PREFIX_PATH "${CMAKE_PREFIX_PATH}" CACHE STRING "" FORCE)

# Create placeholder IMPORTED target for pinocchio
if(NOT TARGET pinocchio::pinocchio)
    add_library(pinocchio::pinocchio UNKNOWN IMPORTED)
    set_target_properties(pinocchio::pinocchio PROPERTIES
        IMPORTED_CONFIGURATIONS "Debug;Release"
        IMPORTED_LOCATION_DEBUG   "${PIN_STAGE}/lib/pinocchio.lib"
        IMPORTED_LOCATION_RELEASE "${PIN_STAGE}/lib/pinocchio.lib"
        INTERFACE_INCLUDE_DIRECTORIES "${PIN_STAGE}/include"
    )
endif()

add_custom_target(pinocchio_stage ALL DEPENDS pinocchio_ext)

# =============================================================================
# URDF Auto-Copy Function
# =============================================================================

function(autocopy_urdf tgt)
  set(_cands
    "${CMAKE_SOURCE_DIR}/surge_v13_hand_right_pybullet.urdf"
    "${CMAKE_SOURCE_DIR}/hand_ik/surge_v13_hand_right_pybullet.urdf")
  foreach(p IN LISTS _cands)
    if(EXISTS "${p}")
      add_custom_command(TARGET ${tgt} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${p}"
                "$<TARGET_FILE_DIR:${tgt}>/surge_v13_hand_right_pybullet.urdf"
        COMMENT "Copying URDF to ${tgt} output dir")
      return()
    endif()
  endforeach()
  message(WARNING "[URDF] Not found in source; ${tgt} will rely on --urdf or HAND_IK_URDF at runtime.")
endfunction()

# =============================================================================
# Hand IK Library
# =============================================================================

add_library(hand_ik STATIC
    src/hand_ik.cpp
    include/hand_ik.hpp
    include/math_constants.hpp
)

add_dependencies(hand_ik pinocchio_stage)
add_dependencies(hand_ik urdfdom_stage)

target_include_directories(hand_ik PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${PIN_STAGE}/include
    ${eigen_SOURCE_DIR}
)

if(WIN32)
    target_include_directories(hand_ik PRIVATE ${VCPKG_X64}/include)
    target_compile_definitions(hand_ik PRIVATE NOMINMAX WIN32_LEAN_AND_MEAN)
endif()

target_compile_features(hand_ik PUBLIC cxx_std_17)

target_link_libraries(hand_ik PUBLIC
    pinocchio::pinocchio
    urdfdom::urdfdom_model
)

# =============================================================================
# Manus Integration
# =============================================================================

if(BUILD_MANUS_INTEGRATION)
    # Find Manus SDK - try multiple possible locations
    set(MANUS_SDK_CANDIDATES 
        "${CMAKE_SOURCE_DIR}/MANUS_Core_3.0.0_SDK"
        "${CMAKE_SOURCE_DIR}/ManusSDK" 
        "${CMAKE_SOURCE_DIR}/manus_sdk"
        "${CMAKE_SOURCE_DIR}/SDK"
    )
    
    set(MANUS_SDK_DIR "" CACHE PATH "Path to Manus SDK")
    
    # If user didn't specify, try to find it
    if(NOT MANUS_SDK_DIR OR NOT EXISTS "${MANUS_SDK_DIR}")
        foreach(candidate IN LISTS MANUS_SDK_CANDIDATES)
            if(EXISTS "${candidate}")
                set(MANUS_SDK_DIR "${candidate}" CACHE PATH "Path to Manus SDK" FORCE)
                message(STATUS "Auto-detected Manus SDK at: ${MANUS_SDK_DIR}")
                break()
            endif()
        endforeach()
    endif()
    
    if(NOT MANUS_SDK_DIR OR NOT EXISTS "${MANUS_SDK_DIR}")
        message(WARNING "Manus SDK not found. Tried locations:")
        foreach(candidate IN LISTS MANUS_SDK_CANDIDATES)
            message(WARNING "  - ${candidate}")
        endforeach()
        message(WARNING "Set MANUS_SDK_DIR manually or disable BUILD_MANUS_INTEGRATION")
        set(BUILD_MANUS_INTEGRATION OFF)
    else()
        message(STATUS "Using Manus SDK at: ${MANUS_SDK_DIR}")
    endif()
endif()

if(BUILD_MANUS_INTEGRATION)
    # Try different possible include directory structures
    set(MANUS_INCLUDE_CANDIDATES
        "${MANUS_SDK_DIR}/include"
        "${MANUS_SDK_DIR}/Include" 
        "${MANUS_SDK_DIR}/headers"
        "${MANUS_SDK_DIR}/Headers"
        "${MANUS_SDK_DIR}"
        "${MANUS_SDK_DIR}/ManusSDK/include"
    )
    
    set(MANUS_INCLUDE_DIR "")
    foreach(candidate IN LISTS MANUS_INCLUDE_CANDIDATES)
        if(EXISTS "${candidate}/ManusSDK.h")
            set(MANUS_INCLUDE_DIR "${candidate}")
            break()
        endif()
    endforeach()
    
    if(NOT MANUS_INCLUDE_DIR)
        message(WARNING "ManusSDK.h not found in any expected location:")
        foreach(candidate IN LISTS MANUS_INCLUDE_CANDIDATES)
            message(WARNING "  - ${candidate}/ManusSDK.h")
        endforeach()
        message(WARNING "Disabling Manus integration")
        set(BUILD_MANUS_INTEGRATION OFF)
    else()
        message(STATUS "Found ManusSDK.h at: ${MANUS_INCLUDE_DIR}/ManusSDK.h")
    endif()
endif()

if(BUILD_MANUS_INTEGRATION)
    # Try different possible library directory structures
    if(WIN32)
        if(CMAKE_SIZEOF_VOID_P EQUAL 8)
            set(MANUS_LIB_CANDIDATES
                "${MANUS_SDK_DIR}/lib/win64"
                "${MANUS_SDK_DIR}/lib/x64"
                "${MANUS_SDK_DIR}/lib/windows"
                "${MANUS_SDK_DIR}/lib"
                "${MANUS_SDK_DIR}/Libraries/win64"
                "${MANUS_SDK_DIR}/Libraries/x64"
                "${MANUS_SDK_DIR}/Libraries"
                "${MANUS_SDK_DIR}/Lib/win64"
                "${MANUS_SDK_DIR}/Lib/x64"
                "${MANUS_SDK_DIR}/Lib"
                "${MANUS_SDK_DIR}/ManusSDK/lib/win64"
                "${MANUS_SDK_DIR}/ManusSDK/lib"
            )
            set(MANUS_BIN_CANDIDATES
                "${MANUS_SDK_DIR}/bin/win64"
                "${MANUS_SDK_DIR}/bin/x64"
                "${MANUS_SDK_DIR}/bin/windows"
                "${MANUS_SDK_DIR}/bin"
                "${MANUS_SDK_DIR}/Binaries/win64"
                "${MANUS_SDK_DIR}/Binaries/x64"
                "${MANUS_SDK_DIR}/Binaries"
                "${MANUS_SDK_DIR}/dll"
                "${MANUS_SDK_DIR}/ManusSDK/bin/win64"
                "${MANUS_SDK_DIR}/ManusSDK/bin"
            )
        else()
            set(MANUS_LIB_CANDIDATES
                "${MANUS_SDK_DIR}/lib/win32"
                "${MANUS_SDK_DIR}/lib/x86"
                "${MANUS_SDK_DIR}/lib"
            )
            set(MANUS_BIN_CANDIDATES
                "${MANUS_SDK_DIR}/bin/win32"
                "${MANUS_SDK_DIR}/bin/x86"
                "${MANUS_SDK_DIR}/bin"
            )
        endif()
    else()
        # Linux/Mac
        set(MANUS_LIB_CANDIDATES
            "${MANUS_SDK_DIR}/lib/linux64"
            "${MANUS_SDK_DIR}/lib/unix"
            "${MANUS_SDK_DIR}/lib"
        )
    endif()
    
    # Find library
    set(MANUS_LIBRARY "")
    foreach(candidate IN LISTS MANUS_LIB_CANDIDATES)
        if(WIN32)
            if(EXISTS "${candidate}/ManusSDK.lib")
                set(MANUS_LIBRARY "${candidate}/ManusSDK.lib")
                set(MANUS_LIB_DIR "${candidate}")
                break()
            endif()
        else()
            if(EXISTS "${candidate}/libManusSDK.so")
                set(MANUS_LIBRARY "${candidate}/libManusSDK.so")
                set(MANUS_LIB_DIR "${candidate}")
                break()
            endif()
        endif()
    endforeach()
    
    if(NOT MANUS_LIBRARY)
        message(WARNING "Manus library not found in any expected location:")
        foreach(candidate IN LISTS MANUS_LIB_CANDIDATES)
            if(WIN32)
                message(WARNING "  - ${candidate}/ManusSDK.lib")
            else()
                message(WARNING "  - ${candidate}/libManusSDK.so")
            endif()
        endforeach()
        message(WARNING "Disabling Manus integration")
        set(BUILD_MANUS_INTEGRATION OFF)
    else()
        message(STATUS "Found Manus library at: ${MANUS_LIBRARY}")
    endif()
    
    # Find DLL (Windows only)
    if(WIN32 AND BUILD_MANUS_INTEGRATION)
        set(MANUS_DLL "")
        foreach(candidate IN LISTS MANUS_BIN_CANDIDATES)
            if(EXISTS "${candidate}/ManusSDK.dll")
                set(MANUS_DLL "${candidate}/ManusSDK.dll")
                set(MANUS_BIN_DIR "${candidate}")
                break()
            endif()
        endforeach()
        
        if(MANUS_DLL)
            message(STATUS "Found Manus DLL at: ${MANUS_DLL}")
        else()
            message(WARNING "ManusSDK.dll not found - runtime may fail")
        endif()
    endif()
endif()

if(BUILD_MANUS_INTEGRATION)
    message(STATUS "Manus SDK configuration:")
    message(STATUS "  SDK Directory: ${MANUS_SDK_DIR}")
    message(STATUS "  Include: ${MANUS_INCLUDE_DIR}")
    message(STATUS "  Library: ${MANUS_LIBRARY}")
    if(WIN32 AND MANUS_DLL)
        message(STATUS "  DLL: ${MANUS_DLL}")
    endif()
    
    # Create Manus Hand IK Bridge library
    add_library(manus_hand_ik_bridge STATIC
        src/ManusHandIKBridge.cpp
        src/ManusSkeletonSetup.cpp
        include/ManusHandIKBridge.h
        include/ManusSkeletonSetup.h
    )
    
    add_dependencies(manus_hand_ik_bridge pinocchio_stage urdfdom_stage)
    
    target_include_directories(manus_hand_ik_bridge PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${MANUS_INCLUDE_DIR}
        ${PIN_STAGE}/include
        ${eigen_SOURCE_DIR}
    )
    
    target_link_libraries(manus_hand_ik_bridge PUBLIC
        hand_ik
        pinocchio::pinocchio
        urdfdom::urdfdom_model
        ${MANUS_LIBRARY}
    )
    
    if(WIN32)
        target_include_directories(manus_hand_ik_bridge PRIVATE ${VCPKG_X64}/include)
        target_compile_definitions(manus_hand_ik_bridge PRIVATE NOMINMAX WIN32_LEAN_AND_MEAN)
    endif()
    
    # Main Manus integration executable
    add_executable(SDKClient
        src/SDKClientIntegration.cpp
    )
    
    add_dependencies(SDKClient pinocchio_stage urdfdom_stage)
    
    target_include_directories(SDKClient PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${MANUS_INCLUDE_DIR}
        ${PIN_STAGE}/include
        ${eigen_SOURCE_DIR}
    )
    
    target_link_libraries(SDKClient PRIVATE
        manus_hand_ik_bridge
        hand_ik
        pinocchio::pinocchio
        urdfdom::urdfdom_model
        ${MANUS_LIBRARY}
    )
    
    if(WIN32)
        target_include_directories(SDKClient PRIVATE ${VCPKG_X64}/include)
        target_compile_definitions(SDKClient PRIVATE NOMINMAX WIN32_LEAN_AND_MEAN)
    endif()
    
    # Auto-copy URDF and DLL for SDKClient
    autocopy_urdf(SDKClient)
    
    if(WIN32 AND MANUS_DLL)
        add_custom_command(TARGET SDKClient POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                    "${MANUS_DLL}"
                    "$<TARGET_FILE_DIR:SDKClient>/ManusSDK.dll"
            COMMENT "Copying Manus SDK DLL to output directory"
        )
        message(STATUS "Will copy ManusSDK.dll to output directory")
    endif()
    
    message(STATUS "✓ Manus integration targets configured")
    message(STATUS "  - manus_hand_ik_bridge (library)")
    message(STATUS "  - SDKClient (main executable)")
endif()

# =============================================================================
# Hand IK Examples and Tests
# =============================================================================

if(HAND_IK_BUILD_EXAMPLES)
    add_executable(hand_ik_example src/main_example.cpp)
    add_dependencies(hand_ik_example pinocchio_stage urdfdom_stage)
    target_link_libraries(hand_ik_example PRIVATE hand_ik)
    
    target_include_directories(hand_ik_example PRIVATE
        ${PIN_STAGE}/include
        ${eigen_SOURCE_DIR}
    )
    
    if(WIN32)
        target_include_directories(hand_ik_example PRIVATE ${VCPKG_X64}/include)
        target_compile_definitions(hand_ik_example PRIVATE NOMINMAX WIN32_LEAN_AND_MEAN)
    endif()
    
    autocopy_urdf(hand_ik_example)
endif()

if(HAND_IK_BUILD_TESTS)
    enable_testing()

    # Jacobian test
    add_executable(test_jacobian tests/test_jacobian.cpp)
    add_dependencies(test_jacobian pinocchio_stage urdfdom_stage)
    target_link_libraries(test_jacobian PRIVATE hand_ik)
    target_include_directories(test_jacobian PRIVATE
        ${PIN_STAGE}/include
        ${eigen_SOURCE_DIR}
    )
    if(WIN32)
        target_include_directories(test_jacobian PRIVATE ${VCPKG_X64}/include)
        target_compile_definitions(test_jacobian PRIVATE NOMINMAX WIN32_LEAN_AND_MEAN)
    endif()
    add_test(NAME jacobian_test COMMAND test_jacobian)
    autocopy_urdf(test_jacobian)

    # Reachability test
    add_executable(test_reachability tests/test_reachability.cpp)
    add_dependencies(test_reachability pinocchio_stage urdfdom_stage)
    target_link_libraries(test_reachability PRIVATE hand_ik)
    target_include_directories(test_reachability PRIVATE
        ${PIN_STAGE}/include
        ${eigen_SOURCE_DIR}
    )
    if(WIN32)
        target_include_directories(test_reachability PRIVATE ${VCPKG_X64}/include)
        target_compile_definitions(test_reachability PRIVATE NOMINMAX WIN32_LEAN_AND_MEAN)
    endif()
    add_test(NAME reachability_test COMMAND test_reachability)
    autocopy_urdf(test_reachability)

    # Passive coupling validation
    add_executable(validate_coupling tests/validate_coupling.cpp)
    add_dependencies(validate_coupling pinocchio_stage urdfdom_stage)
    target_link_libraries(validate_coupling PRIVATE hand_ik)
    target_include_directories(validate_coupling PRIVATE
        ${PIN_STAGE}/include
        ${eigen_SOURCE_DIR}
    )
    if(WIN32)
        target_include_directories(validate_coupling PRIVATE ${VCPKG_X64}/include)
        target_compile_definitions(validate_coupling PRIVATE NOMINMAX WIN32_LEAN_AND_MEAN)
    endif()
    add_test(NAME coupling_validation COMMAND validate_coupling)
    autocopy_urdf(validate_coupling)
    
    # Manus integration test (if enabled)
    if(BUILD_MANUS_INTEGRATION)
        add_executable(test_manus_integration
            tests/test_manus_integration.cpp
        )
        
        add_dependencies(test_manus_integration pinocchio_stage urdfdom_stage)
        
        target_include_directories(test_manus_integration PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/include
            ${MANUS_INCLUDE_DIR}
            ${PIN_STAGE}/include
            ${eigen_SOURCE_DIR}
        )
        
        target_link_libraries(test_manus_integration PRIVATE
            manus_hand_ik_bridge
            hand_ik
        )
        
        if(WIN32)
            target_include_directories(test_manus_integration PRIVATE ${VCPKG_X64}/include)
            target_compile_definitions(test_manus_integration PRIVATE NOMINMAX WIN32_LEAN_AND_MEAN)
        endif()
        
        autocopy_urdf(test_manus_integration)
        add_test(NAME manus_integration_test COMMAND test_manus_integration)
        
        message(STATUS "  - test_manus_integration (test)")
    endif()
endif()

# =============================================================================
# Configuration Summary
# =============================================================================

message(STATUS "=== Configuration Summary ===")
message(STATUS "C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
message(STATUS "Compiler: ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}")
message(STATUS "Platform: ${CMAKE_SYSTEM_NAME}")
message(STATUS "Eigen Source: ${eigen_SOURCE_DIR}")
message(STATUS "Pinocchio Stage: ${PIN_STAGE}")
message(STATUS "URDF Stage: ${URDF_STAGE}")
message(STATUS "TinyXML2 Stage: ${TINYXML2_STAGE}")
if(WIN32)
    message(STATUS "VCPKG Path: ${VCPKG_X64}")
endif()

message(STATUS "Build Options:")
message(STATUS "  Examples: ${HAND_IK_BUILD_EXAMPLES}")
message(STATUS "  Tests: ${HAND_IK_BUILD_TESTS}")
message(STATUS "  Manus Integration: ${BUILD_MANUS_INTEGRATION}")

if(BUILD_MANUS_INTEGRATION)
    message(STATUS "Manus Integration:")
    message(STATUS "  SDK Directory: ${MANUS_SDK_DIR}")
    message(STATUS "  Include Directory: ${MANUS_INCLUDE_DIR}")
    message(STATUS "  Library: ${MANUS_LIBRARY}")
    if(WIN32 AND MANUS_DLL)
        message(STATUS "  DLL: ${MANUS_DLL}")
    endif()
    message(STATUS "  Output: ${CMAKE_BINARY_DIR}/bin/\${CONFIG}")
endif()

message(STATUS "===============================")