# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file LICENSE.rst or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION ${CMAKE_VERSION}) # this file comes with cmake

# If CMAKE_DISABLE_SOURCE_CHANGES is set to true and the source directory is an
# existing directory in our source tree, calling file(MAKE_DIRECTORY) on it
# would cause a fatal error, even though it would be a no-op.
if(NOT EXISTS "C:/Users/nicol/Alt_Bionics/GitHub/manus/build/hand_ik/pinocchio_ext-prefix/src/pinocchio_ext")
  file(MAKE_DIRECTORY "C:/Users/nicol/Alt_Bionics/GitHub/manus/build/hand_ik/pinocchio_ext-prefix/src/pinocchio_ext")
endif()
file(MAKE_DIRECTORY
  "C:/Users/nicol/Alt_Bionics/GitHub/manus/build/hand_ik/pinocchio_ext-prefix/src/pinocchio_ext-build"
  "C:/Users/nicol/Alt_Bionics/GitHub/manus/build/stage/pinocchio"
  "C:/Users/nicol/Alt_Bionics/GitHub/manus/build/hand_ik/pinocchio_ext-prefix/tmp"
  "C:/Users/nicol/Alt_Bionics/GitHub/manus/build/hand_ik/pinocchio_ext-prefix/src/pinocchio_ext-stamp"
  "C:/Users/nicol/Alt_Bionics/GitHub/manus/build/hand_ik/pinocchio_ext-prefix/src"
  "C:/Users/nicol/Alt_Bionics/GitHub/manus/build/hand_ik/pinocchio_ext-prefix/src/pinocchio_ext-stamp"
)

set(configSubDirs Debug;Release;MinSizeRel;RelWithDebInfo)
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "C:/Users/nicol/Alt_Bionics/GitHub/manus/build/hand_ik/pinocchio_ext-prefix/src/pinocchio_ext-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "C:/Users/nicol/Alt_Bionics/GitHub/manus/build/hand_ik/pinocchio_ext-prefix/src/pinocchio_ext-stamp${cfgdir}") # cfgdir has leading slash
endif()
