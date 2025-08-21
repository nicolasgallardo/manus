cmake_minimum_required(VERSION 3.16)
project(manus_integration CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Build options
option(BUILD_MANUS_INTEGRATION "Build Manus SDK integration" OFF)

# ---- Manus SDK detection (minimal & robust) ----
set(MANUS_SDK_DIR "${CMAKE_SOURCE_DIR}/MANUS_Core_3.0.0_SDK" CACHE PATH "Root of Manus SDK")

# Known include locations (most important first)
set(_MANUS_SDK_INC_CANDIDATES
    "${MANUS_SDK_DIR}/SDKClient_Windows/ManusSDK/include"
    "${MANUS_SDK_DIR}/SDKMinimalClient_Windows/ManusSDK/include"
    "${MANUS_SDK_DIR}/ManusSDK/include"
    "${MANUS_SDK_DIR}/include"
)

find_path(MANUS_SDK_INCLUDE_DIR ManusSDK.h
  PATHS ${_MANUS_SDK_INC_CANDIDATES}
  NO_DEFAULT_PATH)

# Known library locations
set(_MANUS_SDK_LIB_CANDIDATES
    "${MANUS_SDK_DIR}/SDKClient_Windows/ManusSDK/lib/ManusSDK.lib"
    "${MANUS_SDK_DIR}/SDKMinimalClient_Windows/ManusSDK/lib/ManusSDK.lib"
    "${MANUS_SDK_DIR}/ManusSDK/lib/ManusSDK.lib"
)
set(MANUS_SDK_LIBRARY "")
foreach(_cand IN LISTS _MANUS_SDK_LIB_CANDIDATES)
  if(EXISTS "${_cand}")
    set(MANUS_SDK_LIBRARY "${_cand}")
    break()
  endif()
endforeach()

# Known DLL locations (optional copy/postbuild)
set(_MANUS_SDK_DLL_CANDIDATES
    "${MANUS_SDK_DIR}/SDKClient_Windows/ManusSDK/lib/ManusSDK.dll"
    "${MANUS_SDK_DIR}/SDKMinimalClient_Windows/ManusSDK/lib/ManusSDK.dll"
    "${MANUS_SDK_DIR}/Output/x64/Release/ManusSDK.dll"
    "${MANUS_SDK_DIR}/Output/x64/Debug/ManusSDK.dll"
)
set(MANUS_SDK_DLL "")
foreach(_cand IN LISTS _MANUS_SDK_DLL_CANDIDATES)
  if(EXISTS "${_cand}")
    set(MANUS_SDK_DLL "${_cand}")
    break()
  endif()
endforeach()

# Gate integration EXACTLY like before; only flip ON when both header & lib are found
if(MANUS_SDK_INCLUDE_DIR AND MANUS_SDK_LIBRARY)
  set(BUILD_MANUS_INTEGRATION ON CACHE BOOL "Build Manus integration" FORCE)
  message(STATUS "Manus SDK include dir: ${MANUS_SDK_INCLUDE_DIR}")
  message(STATUS "Manus SDK library:     ${MANUS_SDK_LIBRARY}")
  if(MANUS_SDK_DLL)
    message(STATUS "Manus SDK DLL:         ${MANUS_SDK_DLL}")
  endif()
else()
  set(BUILD_MANUS_INTEGRATION OFF CACHE BOOL "Build Manus integration" FORCE)
  message(WARNING "Manus SDK headers/lib not found. Manus integration disabled.
  Looked under: ${MANUS_SDK_DIR}")
endif()
# ---- end Manus SDK detection ----

# Find dependencies
find_package(Threads REQUIRED)

# Find Eigen3 (use system/vcpkg version to avoid alias collision)
find_package(Eigen3 REQUIRED)

# Add hand_ik subdirectory (preserves the working build structure)
add_subdirectory(hand_ik)

# Main SDKClient target (preserves the previous working target structure)
if(BUILD_MANUS_INTEGRATION)
    add_executable(SDKClient src/SDKClientIntegration.cpp)
    
    # Link to hand_ik library
    target_link_libraries(SDKClient PRIVATE hand_ik)
    
    # Add Manus SDK includes and libraries
    target_include_directories(SDKClient PRIVATE "${MANUS_SDK_INCLUDE_DIR}")
    target_link_libraries(SDKClient PRIVATE "${MANUS_SDK_LIBRARY}")
    
    # Link system libraries
    target_link_libraries(SDKClient PRIVATE Threads::Threads)
    
    # Optional: copy DLL next to the exe if MANUS_SDK_DLL resolved
    if(MANUS_SDK_DLL)
      add_custom_command(TARGET SDKClient POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${MANUS_SDK_DLL}"
                "$<TARGET_FILE_DIR:SDKClient>/ManusSDK.dll"
        COMMENT "Copying ManusSDK.dll to output directory")
    endif()
    
    # Preprocessor definition to enable Manus integration
    target_compile_definitions(SDKClient PRIVATE MANUS_SDK_AVAILABLE)
    
    message(STATUS "SDKClient will be built with Manus SDK integration")
else()
    message(STATUS "Manus SDK not found - SDKClient will not be built")
endif()

# Configuration summary
message(STATUS "=== Manus Integration Configuration Summary ===")
message(STATUS "Manus Integration: ${BUILD_MANUS_INTEGRATION}")
if(BUILD_MANUS_INTEGRATION)
    message(STATUS "Manus SDK Dir: ${MANUS_SDK_DIR}")
    message(STATUS "Manus Include: ${MANUS_SDK_INCLUDE_DIR}")
    message(STATUS "Manus Library: ${MANUS_SDK_LIBRARY}")
    if(MANUS_SDK_DLL)
        message(STATUS "Manus DLL: ${MANUS_SDK_DLL}")
    endif()
endif()
message(STATUS "Hand IK: Managed by hand_ik/CMakeLists.txt")
message(STATUS "==============================================")