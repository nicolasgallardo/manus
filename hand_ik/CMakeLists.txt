cmake_minimum_required(VERSION 3.24)
project(hand_ik CXX)

set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreadedDLL")
set(BUILD_SHARED_LIBS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_POLICY_VERSION_MINIMUM 3.5)

# Set C++ standard for all targets
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Build options (default ON so executables are built)
option(HAND_IK_BUILD_EXAMPLES "Build examples" ON)
option(HAND_IK_BUILD_TESTS    "Build tests"    ON)

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

# Set policy to avoid FindBoost warnings (optional but nice)
if(POLICY CMP0167)
    cmake_policy(SET CMP0167 NEW)
endif()
if(POLICY CMP0144)
    cmake_policy(SET CMP0144 NEW)
endif()

# Include required modules at top level
include(FetchContent)
include(ExternalProject)

# ============================================================================
# EIGEN HANDLING - RESPECT PARENT DECISION
# ============================================================================
message(STATUS "[hand_ik] Checking Eigen availability...")

# Check if parent already provided authoritative decision
if(DEFINED EIGEN3_FROM_SYSTEM AND EIGEN3_FROM_SYSTEM)
    message(STATUS "[hand_ik] Using parent-provided system Eigen3::Eigen")
    if(NOT TARGET Eigen3::Eigen)
        message(FATAL_ERROR "[hand_ik] Parent claims system Eigen but target missing!")
    endif()
elseif(DEFINED SKIP_VENDOR_EIGEN AND SKIP_VENDOR_EIGEN)
    message(STATUS "[hand_ik] Parent requests skipping vendor Eigen")
    if(NOT TARGET Eigen3::Eigen)
        message(FATAL_ERROR "[hand_ik] Parent skipped vendor Eigen but no target exists!")
    endif()
elseif(TARGET Eigen3::Eigen)
    message(STATUS "[hand_ik] Eigen3::Eigen target already exists (from parent or previous subdir)")
    # Don't create alias or fetch - use existing target
else()
    # Fallback: try to find or vendor Eigen ourselves
    message(STATUS "[hand_ik] No Eigen guidance from parent, handling locally...")
    
    # Try system first
    find_package(Eigen3 CONFIG QUIET)
    if(TARGET Eigen3::Eigen)
        message(STATUS "[hand_ik] Found system Eigen3::Eigen locally")
    else()
        # Last resort: vendor Eigen (only if no target exists)
        message(STATUS "[hand_ik] No system Eigen found, vendoring as fallback...")
        
        # Always use fetched sources; never prefer system packages in this fallback
        set(FETCHCONTENT_TRY_FIND_PACKAGE FALSE)
        set(FETCHCONTENT_QUIET OFF)
        
        # FetchContent and ExternalProject already included at top level
        
        FetchContent_Declare(
            eigen
            GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
            GIT_TAG 3.4.0
        )
        
        # Suppress Eigen's own tests/docs to reduce noise
        set(EIGEN_BUILD_DOC OFF CACHE BOOL "" FORCE)
        set(EIGEN_BUILD_TESTING OFF CACHE BOOL "" FORCE)
        set(EIGEN_BUILD_PKGCONFIG OFF CACHE BOOL "" FORCE)
        
        FetchContent_MakeAvailable(eigen)
        
        # Set fallback include dir for other parts of the build
        set(EIGEN3_INCLUDE_DIR "${eigen_SOURCE_DIR}" CACHE PATH "" FORCE)
        set(Eigen3_DIR "" CACHE PATH "" FORCE)
        
        # Defensive: create alias only if it doesn't exist
        if(NOT TARGET Eigen3::Eigen)
            add_library(Eigen3::Eigen INTERFACE IMPORTED)
            target_include_directories(Eigen3::Eigen INTERFACE "${eigen_SOURCE_DIR}")
            message(STATUS "[hand_ik] Created fallback Eigen3::Eigen INTERFACE target")
        endif()
    endif()
endif()

# Final validation
if(NOT TARGET Eigen3::Eigen)
    message(FATAL_ERROR "[hand_ik] Failed to resolve Eigen3::Eigen target!")
else()
    message(STATUS "[hand_ik] ✓ Eigen3::Eigen target is available")
endif()

# ============================================================================
# EXTERNAL DEPENDENCIES (unchanged)
# ============================================================================

# ---------------- tinyxml2 (ExternalProject with staged install) ----------------
set(TINYXML2_STAGE "${CMAKE_BINARY_DIR}/stage/tinyxml2")

ExternalProject_Add(tinyxml2_ext
  URL https://github.com/leethomason/tinyxml2/archive/refs/tags/11.0.0.tar.gz
  UPDATE_DISCONNECTED 1
  CMAKE_ARGS
    -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
    -DCMAKE_MSVC_RUNTIME_LIBRARY=${CMAKE_MSVC_RUNTIME_LIBRARY}
    -DBUILD_SHARED_LIBS=OFF
    -DBUILD_TESTING=OFF
    -DCMAKE_INSTALL_PREFIX=${TINYXML2_STAGE}
  # Multi-config build commands with explicit config
  BUILD_COMMAND ${CMAKE_COMMAND} --build <BINARY_DIR> --config $<CONFIG>
  INSTALL_COMMAND ${CMAKE_COMMAND} --build <BINARY_DIR> --config $<CONFIG> --target install
  INSTALL_DIR ${TINYXML2_STAGE}
  BUILD_BYPRODUCTS
    ${TINYXML2_STAGE}/lib/tinyxml2.lib
    ${TINYXML2_STAGE}/lib/cmake/tinyxml2/tinyxml2Config.cmake
)

# Set up TinyXML2 paths for urdfdom
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
    -DBUILD_TESTING=OFF
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
    -DBUILD_TESTING=OFF
    -DURDFDOM_BUILD_TESTS=OFF
    # Use staged TinyXML2 with config-independent paths
    -DTinyXML2_DIR=${TINYXML2_STAGE}/lib/cmake/tinyxml2  # preferred: use CONFIG mode
    -DTINYXML2_INCLUDE_DIR=${TINYXML2_STAGE}/include     # fallback: manual paths  
    -DTINYXML2_LIBRARY=${TINYXML2_STAGE}/lib/tinyxml2.lib # fallback: manual paths
    -Durdfdom_headers_DIR=${urdfdom_headers_DIR}
    -DCMAKE_INSTALL_PREFIX=${URDF_STAGE}
  # Multi-config build and install commands
  BUILD_COMMAND ${CMAKE_COMMAND} --build <BINARY_DIR> --config $<CONFIG>
  INSTALL_COMMAND ${CMAKE_COMMAND} --build <BINARY_DIR> --config $<CONFIG> --target install
  INSTALL_DIR ${URDF_STAGE}
  BUILD_BYPRODUCTS
    ${URDF_STAGE}/lib/urdfdom_model.lib
    ${URDF_STAGE}/lib/cmake/urdfdom/urdfdom-config.cmake
)
add_dependencies(urdfdom_ext urdfdom_headers_ext)
add_dependencies(urdfdom_ext tinyxml2_ext)  # ensure staged tinyxml2 is available

set(urdfdom_DIR "${URDF_STAGE}/lib/cmake/urdfdom" CACHE PATH "" FORCE)
list(PREPEND CMAKE_PREFIX_PATH "${URDF_STAGE}" "${URDF_HEADERS_STAGE}" "${TINYXML2_STAGE}")
set(CMAKE_PREFIX_PATH "${CMAKE_PREFIX_PATH}" CACHE STRING "" FORCE)

# Try to find urdfdom first (quiet, not required)
find_package(urdfdom CONFIG QUIET)

# If not found, create placeholder IMPORTED targets for urdfdom
if(NOT TARGET urdfdom::urdfdom_model)
    add_library(urdfdom::urdfdom_model UNKNOWN IMPORTED)
    # Try to guess the final filenames; fall back to standard names
    set(_URDF_MODEL_RELEASE "${URDF_STAGE}/lib/urdfdom_model.lib")
    set(_URDF_MODEL_DEBUG   "${URDF_STAGE}/lib/urdfdom_modeld.lib")
    
    # If debug lib doesn't exist, use release for both configs
    if(NOT EXISTS "${_URDF_MODEL_DEBUG}")
        set(_URDF_MODEL_DEBUG "${_URDF_MODEL_RELEASE}")
    endif()
    
    set_target_properties(urdfdom::urdfdom_model PROPERTIES
        IMPORTED_CONFIGURATIONS "Debug;Release"
        IMPORTED_LOCATION_RELEASE "${_URDF_MODEL_RELEASE}"
        IMPORTED_LOCATION_DEBUG   "${_URDF_MODEL_DEBUG}"
        INTERFACE_INCLUDE_DIRECTORIES "${URDF_STAGE}/include"
    )
    message(STATUS "Using placeholder imported target urdfdom::urdfdom_model until staged install completes.")
else()
    message(STATUS "Found staged urdfdom: ${urdfdom_VERSION}")
endif()

# Create a staging target to enforce build order for urdfdom
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

  # Ensure all submodules are present before configure
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
    
    # Use resolved Eigen (either system or vendored)
    -DEIGEN3_INCLUDE_DIR=${eigen_SOURCE_DIR}
    
    # URDFDOM configuration - pass parent include directories
    -Durdfdom_DIR:PATH=${URDF_STAGE}/lib/cmake/urdfdom
    -Durdfdom_headers_DIR:PATH=${URDF_HEADERS_STAGE}/share/urdfdom_headers
    -DURDFDOM_INCLUDE_DIR:PATH=${URDF_STAGE}/include
    -DURDFDOM_HEADERS_INCLUDE_DIR:PATH=${URDF_HEADERS_STAGE}/include
    -DCMAKE_INCLUDE_PATH:PATH=${URDF_STAGE}/include;${URDF_HEADERS_STAGE}/include
    -DCMAKE_PREFIX_PATH=${URDF_STAGE};${URDF_HEADERS_STAGE};${TINYXML2_STAGE}
    
    # TinyXML2 configuration
    -DTinyXML2_DIR=${TINYXML2_STAGE}/lib/cmake/tinyxml2  # use staged TinyXML2

    # ---- Boost (module mode) ---- use UPPERCASE only ----
    -DBoost_NO_BOOST_CMAKE=ON
    -DBoost_USE_STATIC_LIBS=ON
    -DBoost_USE_MULTITHREADED=ON
    -DBoost_USE_STATIC_RUNTIME=OFF
    -DBoost_NO_SYSTEM_PATHS=ON
    -DBOOST_ROOT=${VCPKG_X64}
    -DBOOST_INCLUDEDIR=${VCPKG_X64}/include
    -DBOOST_LIBRARYDIR=${VCPKG_X64}/lib

    # >>> Avoid Program Files install — stage locally
    -DCMAKE_INSTALL_PREFIX=${PIN_STAGE}         # <<< NEW
    -DCMAKE_INSTALL_LIBDIR=lib                  # <<< NEW (optional but tidy)
    -DCMAKE_INSTALL_BINDIR=bin                  # <<< NEW
    -DCMAKE_INSTALL_INCLUDEDIR=include          # <<< NEW
  
  # Multi-config build and install commands with explicit config
  BUILD_COMMAND ${CMAKE_COMMAND} --build <BINARY_DIR> --config $<CONFIG>
  INSTALL_COMMAND ${CMAKE_COMMAND} --build <BINARY_DIR> --config $<CONFIG> --target install
  INSTALL_DIR ${PIN_STAGE}                      # <<< NEW
  
  BUILD_BYPRODUCTS
    ${PIN_STAGE}/lib/pinocchio.lib
    ${PIN_STAGE}/lib/cmake/pinocchio/pinocchio-config.cmake
)
add_dependencies(pinocchio_ext urdfdom_ext)  # ensure urdfdom is staged first

# Where Pinocchio's package config will land:
set(Pinocchio_DIR "${PIN_STAGE}/lib/cmake/pinocchio" CACHE PATH "" FORCE)
list(PREPEND CMAKE_PREFIX_PATH "${PIN_STAGE}")
set(CMAKE_PREFIX_PATH "${CMAKE_PREFIX_PATH}" CACHE STRING "" FORCE)

# Try to find Pinocchio first (quiet, not required)
find_package(pinocchio CONFIG QUIET)

# If not found, create a placeholder IMPORTED target
if(NOT TARGET pinocchio::pinocchio)
    add_library(pinocchio::pinocchio UNKNOWN IMPORTED)
    # Set properties for both Debug and Release configs
    set_target_properties(pinocchio::pinocchio PROPERTIES
        IMPORTED_CONFIGURATIONS "Debug;Release"
        IMPORTED_LOCATION_DEBUG   "${PIN_STAGE}/lib/pinocchio.lib"
        IMPORTED_LOCATION_RELEASE "${PIN_STAGE}/lib/pinocchio.lib"
        INTERFACE_INCLUDE_DIRECTORIES "${PIN_STAGE}/include"
    )
    message(STATUS "Using placeholder imported target for pinocchio until staged install completes.")
else()
    message(STATUS "Found staged Pinocchio: ${pinocchio_VERSION}")
endif()

# Create a staging target to enforce build order
add_custom_target(pinocchio_stage ALL DEPENDS pinocchio_ext)

# =============================================================================
# URDF Auto-Copy Function (NEW)
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
# Hand IK Library and Targets
# =============================================================================

# Create the hand_ik library
add_library(hand_ik STATIC
    src/hand_ik.cpp
    include/hand_ik.hpp
    include/math_constants.hpp
)

# Make sure hand_ik builds after external deps
add_dependencies(hand_ik pinocchio_stage)  # use staging target instead of direct ext
add_dependencies(hand_ik urdfdom_stage)    # also wait for urdfdom staging

# Include directories for library
target_include_directories(hand_ik PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${PIN_STAGE}/include
)

# Add Eigen include directory (works for both system and vendored)
if(DEFINED eigen_SOURCE_DIR)
    target_include_directories(hand_ik PUBLIC ${eigen_SOURCE_DIR})
elseif(DEFINED EIGEN3_INCLUDE_DIR)
    target_include_directories(hand_ik PUBLIC ${EIGEN3_INCLUDE_DIR})
endif()

# Windows-specific includes
if(WIN32)
    target_include_directories(hand_ik PRIVATE
        ${VCPKG_X64}/include
    )
endif()

target_compile_features(hand_ik PUBLIC cxx_std_17)

# Link libraries - TinyXML2 comes transitively through urdfdom
target_link_libraries(hand_ik PUBLIC
    pinocchio::pinocchio  # use imported target instead of raw lib name
    urdfdom::urdfdom_model  # use imported target instead of raw lib name
    Eigen3::Eigen  # use resolved Eigen target
    # TinyXML2 is linked transitively through urdfdom - no need to link it directly
)

# Windows definitions
if(WIN32)
    target_compile_definitions(hand_ik PRIVATE NOMINMAX WIN32_LEAN_AND_MEAN)
endif()

# =============================================================================
# Executables
# =============================================================================

# Create example executable
if(HAND_IK_BUILD_EXAMPLES)
    # Check if we have a specific example source
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/examples/hand_ik_example.cpp")
        set(EXAMPLE_SOURCE "${CMAKE_CURRENT_SOURCE_DIR}/examples/hand_ik_example.cpp")
    elseif(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/main_example.cpp")
        set(EXAMPLE_SOURCE "${CMAKE_CURRENT_SOURCE_DIR}/src/main_example.cpp")
    else()
        message(STATUS "No example source found, skipping hand_ik_example")
        set(EXAMPLE_SOURCE "")
    endif()
    
    if(EXAMPLE_SOURCE)
        add_executable(hand_ik_example "${EXAMPLE_SOURCE}")
        add_dependencies(hand_ik_example pinocchio_stage)  # use staging target
        add_dependencies(hand_ik_example urdfdom_stage)    # also wait for urdfdom
        target_link_libraries(hand_ik_example PRIVATE hand_ik)

        # Include dirs for executables
        target_include_directories(hand_ik_example PRIVATE
            ${PIN_STAGE}/include
        )
        
        # Add Eigen include directory
        if(DEFINED eigen_SOURCE_DIR)
            target_include_directories(hand_ik_example PRIVATE ${eigen_SOURCE_DIR})
        elseif(DEFINED EIGEN3_INCLUDE_DIR)
            target_include_directories(hand_ik_example PRIVATE ${EIGEN3_INCLUDE_DIR})
        endif()

        if(WIN32)
            target_include_directories(hand_ik_example PRIVATE
                ${VCPKG_X64}/include
            )
            target_compile_definitions(hand_ik_example PRIVATE NOMINMAX WIN32_LEAN_AND_MEAN)
        endif()

        # Auto-copy URDF for this executable
        autocopy_urdf(hand_ik_example)
        
        message(STATUS "hand_ik_example will be built from: ${EXAMPLE_SOURCE}")
    endif()
endif()

# Create test executables
if(HAND_IK_BUILD_TESTS)
    enable_testing()

    # Jacobian test
    add_executable(test_jacobian tests/test_jacobian.cpp)
    add_dependencies(test_jacobian pinocchio_stage)  # use staging target
    add_dependencies(test_jacobian urdfdom_stage)    # also wait for urdfdom
    target_link_libraries(test_jacobian PRIVATE hand_ik)
    target_include_directories(test_jacobian PRIVATE
        ${PIN_STAGE}/include
    )
    
    # Add Eigen include directory
    if(DEFINED eigen_SOURCE_DIR)
        target_include_directories(test_jacobian PRIVATE ${eigen_SOURCE_DIR})
    elseif(DEFINED EIGEN3_INCLUDE_DIR)
        target_include_directories(test_jacobian PRIVATE ${EIGEN3_INCLUDE_DIR})
    endif()
    
    if(WIN32)
        target_include_directories(test_jacobian PRIVATE ${VCPKG_X64}/include)
        target_compile_definitions(test_jacobian PRIVATE NOMINMAX WIN32_LEAN_AND_MEAN)
    endif()
    add_test(NAME jacobian_test COMMAND test_jacobian)
    
    # Auto-copy URDF for this executable
    autocopy_urdf(test_jacobian)

    # Reachability test
    add_executable(test_reachability tests/test_reachability.cpp)
    add_dependencies(test_reachability pinocchio_stage)  # use staging target
    add_dependencies(test_reachability urdfdom_stage)    # also wait for urdfdom
    target_link_libraries(test_reachability PRIVATE hand_ik)
    target_include_directories(test_reachability PRIVATE
        ${PIN_STAGE}/include
    )
    
    # Add Eigen include directory
    if(DEFINED eigen_SOURCE_DIR)
        target_include_directories(test_reachability PRIVATE ${eigen_SOURCE_DIR})
    elseif(DEFINED EIGEN3_INCLUDE_DIR)
        target_include_directories(test_reachability PRIVATE ${EIGEN3_INCLUDE_DIR})
    endif()
    
    if(WIN32)
        target_include_directories(test_reachability PRIVATE ${VCPKG_X64}/include)
        target_compile_definitions(test_reachability PRIVATE NOMINMAX WIN32_LEAN_AND_MEAN)
    endif()
    add_test(NAME reachability_test COMMAND test_reachability)
    
    # Auto-copy URDF for this executable
    autocopy_urdf(test_reachability)

    # Passive coupling validation
    add_executable(validate_coupling tests/validate_coupling.cpp)
    add_dependencies(validate_coupling pinocchio_stage)  # use staging target
    add_dependencies(validate_coupling urdfdom_stage)    # also wait for urdfdom
    target_link_libraries(validate_coupling PRIVATE hand_ik)
    target_include_directories(validate_coupling PRIVATE
        ${PIN_STAGE}/include
    )
    
    # Add Eigen include directory
    if(DEFINED eigen_SOURCE_DIR)
        target_include_directories(validate_coupling PRIVATE ${eigen_SOURCE_DIR})
    elseif(DEFINED EIGEN3_INCLUDE_DIR)
        target_include_directories(validate_coupling PRIVATE ${EIGEN3_INCLUDE_DIR})
    endif()
    
    if(WIN32)
        target_include_directories(validate_coupling PRIVATE ${VCPKG_X64}/include)
        target_compile_definitions(validate_coupling PRIVATE NOMINMAX WIN32_LEAN_AND_MEAN)
    endif()
    add_test(NAME coupling_validation COMMAND validate_coupling)
    
    # Auto-copy URDF for this executable
    autocopy_urdf(validate_coupling)
endif()

# =============================================================================
# Configuration and Installation (Optional - for cross-platform compatibility)
# =============================================================================

# Create config directory in build
file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/config)

# Copy config files if they exist
if(EXISTS ${CMAKE_SOURCE_DIR}/config/hand_ik_example.yaml)
    configure_file(
        ${CMAKE_SOURCE_DIR}/config/hand_ik_example.yaml
        ${CMAKE_BINARY_DIR}/config/hand_ik_example.yaml
        COPYONLY
    )
endif()

# Print configuration summary
message(STATUS "=== Hand IK Configuration Summary ===")
message(STATUS "C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
message(STATUS "Compiler: ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}")
message(STATUS "Platform: ${CMAKE_SYSTEM_NAME}")

# Eigen status reporting
if(DEFINED EIGEN3_FROM_SYSTEM AND EIGEN3_FROM_SYSTEM)
    message(STATUS "Eigen Source: SYSTEM/VCPKG (${CMAKE_PREFIX_PATH})")
elseif(DEFINED eigen_SOURCE_DIR)
    message(STATUS "Eigen Source: VENDORED (${eigen_SOURCE_DIR})")
else()
    message(STATUS "Eigen Source: UNKNOWN (target exists but source unclear)")
endif()

message(STATUS "Pinocchio Stage: ${PIN_STAGE}")
message(STATUS "Expect pinocchio.lib at: ${PIN_STAGE}/lib/pinocchio.lib")
message(STATUS "URDF Stage: ${URDF_STAGE}")
message(STATUS "TinyXML2 Stage: ${TINYXML2_STAGE}")
if(WIN32)
    message(STATUS "VCPKG Path: ${VCPKG_X64}")
endif()

# Check for staged files after configure (diagnostics only, not required)
if(EXISTS "${PIN_STAGE}/lib/pinocchio.lib")
    message(STATUS "✓ Found staged pinocchio.lib")
else()
    message(STATUS "⚠ pinocchio.lib not found - will be built by pinocchio_ext")
endif()

if(EXISTS "${PIN_STAGE}/lib/cmake/pinocchio/pinocchio-config.cmake")
    message(STATUS "✓ Found staged pinocchio-config.cmake")
else()
    message(STATUS "⚠ pinocchio-config.cmake not found - will be built by pinocchio_ext")
endif()

if(EXISTS "${URDF_STAGE}/lib/urdfdom_model.lib")
    message(STATUS "✓ Found staged urdfdom_model.lib")
else()
    message(STATUS "⚠ urdfdom_model.lib not found - will be built by urdfdom_ext")
endif()

if(EXISTS "${URDF_STAGE}/lib/cmake/urdfdom/urdfdom-config.cmake")
    message(STATUS "✓ Found staged urdfdom-config.cmake")
else()
    message(STATUS "⚠ urdfdom-config.cmake not found - will be built by urdfdom_ext")
endif()

message(STATUS "=====================================")


# Optional: Add installation rules for easy distribution
if(HAND_IK_BUILD_EXAMPLES OR HAND_IK_BUILD_TESTS)
    set(INSTALL_TARGETS)
    if(HAND_IK_BUILD_EXAMPLES)
        list(APPEND INSTALL_TARGETS hand_ik_example)
    endif()
    if(HAND_IK_BUILD_TESTS)
        list(APPEND INSTALL_TARGETS test_jacobian test_reachability validate_coupling)
    endif()
    
    install(TARGETS ${INSTALL_TARGETS}
        RUNTIME DESTINATION bin
        COMPONENT applications
    )
    
    install(TARGETS hand_ik
        EXPORT hand_ik_targets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
        COMPONENT libraries
    )

    install(FILES 
        include/hand_ik.hpp
        include/math_constants.hpp
        DESTINATION include
        COMPONENT headers
    )

    if(EXISTS ${CMAKE_SOURCE_DIR}/config/hand_ik_example.yaml)
        install(FILES config/hand_ik_example.yaml
            DESTINATION share/hand_ik/config
            COMPONENT config
        )
    endif()
endif()

message(STATUS "Build configuration:")
message(STATUS "  Examples: ${HAND_IK_BUILD_EXAMPLES}")
message(STATUS "  Tests: ${HAND_IK_BUILD_TESTS}")
message(STATUS "  Executables will be in: ${CMAKE_BINARY_DIR}/bin/\${CONFIG}")
message(STATUS "  Libraries will be in: ${CMAKE_BINARY_DIR}/lib/\${CONFIG}")