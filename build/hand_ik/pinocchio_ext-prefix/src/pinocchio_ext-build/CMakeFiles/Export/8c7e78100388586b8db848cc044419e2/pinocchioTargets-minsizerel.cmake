#----------------------------------------------------------------
# Generated CMake target import file for configuration "MinSizeRel".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "pinocchio::pinocchio" for configuration "MinSizeRel"
set_property(TARGET pinocchio::pinocchio APPEND PROPERTY IMPORTED_CONFIGURATIONS MINSIZEREL)
set_target_properties(pinocchio::pinocchio PROPERTIES
  IMPORTED_IMPLIB_MINSIZEREL "C:/Users/nicol/Alt_Bionics/GitHub/manus/build/stage/pinocchio/lib/pinocchio.lib"
  IMPORTED_LOCATION_MINSIZEREL "C:/Users/nicol/Alt_Bionics/GitHub/manus/build/stage/pinocchio/bin/pinocchio.dll"
  )

list(APPEND _cmake_import_check_targets pinocchio::pinocchio )
list(APPEND _cmake_import_check_files_for_pinocchio::pinocchio "C:/Users/nicol/Alt_Bionics/GitHub/manus/build/stage/pinocchio/lib/pinocchio.lib" "C:/Users/nicol/Alt_Bionics/GitHub/manus/build/stage/pinocchio/bin/pinocchio.dll" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
