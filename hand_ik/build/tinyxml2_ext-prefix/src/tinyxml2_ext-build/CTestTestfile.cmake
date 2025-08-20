# CMake generated Testfile for 
# Source directory: C:/Users/nicol/Alt_Bionics/GitLab/surge/firmware/Apps/pinocchio_ik_v3/hand_ik/hand_ik/build/tinyxml2_ext-prefix/src/tinyxml2_ext
# Build directory: C:/Users/nicol/Alt_Bionics/GitLab/surge/firmware/Apps/pinocchio_ik_v3/hand_ik/hand_ik/build/tinyxml2_ext-prefix/src/tinyxml2_ext-build
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
if(CTEST_CONFIGURATION_TYPE MATCHES "^([Dd][Ee][Bb][Uu][Gg])$")
  add_test(xmltest "C:/Users/nicol/Alt_Bionics/GitLab/surge/firmware/Apps/pinocchio_ik_v3/hand_ik/hand_ik/build/tinyxml2_ext-prefix/src/tinyxml2_ext-build/Debug/xmltest.exe")
  set_tests_properties(xmltest PROPERTIES  PASS_REGULAR_EXPRESSION ", Fail 0" WORKING_DIRECTORY "C:/Users/nicol/Alt_Bionics/GitLab/surge/firmware/Apps/pinocchio_ik_v3/hand_ik/hand_ik/build/tinyxml2_ext-prefix/src/tinyxml2_ext" _BACKTRACE_TRIPLES "C:/Users/nicol/Alt_Bionics/GitLab/surge/firmware/Apps/pinocchio_ik_v3/hand_ik/hand_ik/build/tinyxml2_ext-prefix/src/tinyxml2_ext/CMakeLists.txt;49;add_test;C:/Users/nicol/Alt_Bionics/GitLab/surge/firmware/Apps/pinocchio_ik_v3/hand_ik/hand_ik/build/tinyxml2_ext-prefix/src/tinyxml2_ext/CMakeLists.txt;0;")
elseif(CTEST_CONFIGURATION_TYPE MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
  add_test(xmltest "C:/Users/nicol/Alt_Bionics/GitLab/surge/firmware/Apps/pinocchio_ik_v3/hand_ik/hand_ik/build/tinyxml2_ext-prefix/src/tinyxml2_ext-build/Release/xmltest.exe")
  set_tests_properties(xmltest PROPERTIES  PASS_REGULAR_EXPRESSION ", Fail 0" WORKING_DIRECTORY "C:/Users/nicol/Alt_Bionics/GitLab/surge/firmware/Apps/pinocchio_ik_v3/hand_ik/hand_ik/build/tinyxml2_ext-prefix/src/tinyxml2_ext" _BACKTRACE_TRIPLES "C:/Users/nicol/Alt_Bionics/GitLab/surge/firmware/Apps/pinocchio_ik_v3/hand_ik/hand_ik/build/tinyxml2_ext-prefix/src/tinyxml2_ext/CMakeLists.txt;49;add_test;C:/Users/nicol/Alt_Bionics/GitLab/surge/firmware/Apps/pinocchio_ik_v3/hand_ik/hand_ik/build/tinyxml2_ext-prefix/src/tinyxml2_ext/CMakeLists.txt;0;")
elseif(CTEST_CONFIGURATION_TYPE MATCHES "^([Mm][Ii][Nn][Ss][Ii][Zz][Ee][Rr][Ee][Ll])$")
  add_test(xmltest "C:/Users/nicol/Alt_Bionics/GitLab/surge/firmware/Apps/pinocchio_ik_v3/hand_ik/hand_ik/build/tinyxml2_ext-prefix/src/tinyxml2_ext-build/MinSizeRel/xmltest.exe")
  set_tests_properties(xmltest PROPERTIES  PASS_REGULAR_EXPRESSION ", Fail 0" WORKING_DIRECTORY "C:/Users/nicol/Alt_Bionics/GitLab/surge/firmware/Apps/pinocchio_ik_v3/hand_ik/hand_ik/build/tinyxml2_ext-prefix/src/tinyxml2_ext" _BACKTRACE_TRIPLES "C:/Users/nicol/Alt_Bionics/GitLab/surge/firmware/Apps/pinocchio_ik_v3/hand_ik/hand_ik/build/tinyxml2_ext-prefix/src/tinyxml2_ext/CMakeLists.txt;49;add_test;C:/Users/nicol/Alt_Bionics/GitLab/surge/firmware/Apps/pinocchio_ik_v3/hand_ik/hand_ik/build/tinyxml2_ext-prefix/src/tinyxml2_ext/CMakeLists.txt;0;")
elseif(CTEST_CONFIGURATION_TYPE MATCHES "^([Rr][Ee][Ll][Ww][Ii][Tt][Hh][Dd][Ee][Bb][Ii][Nn][Ff][Oo])$")
  add_test(xmltest "C:/Users/nicol/Alt_Bionics/GitLab/surge/firmware/Apps/pinocchio_ik_v3/hand_ik/hand_ik/build/tinyxml2_ext-prefix/src/tinyxml2_ext-build/RelWithDebInfo/xmltest.exe")
  set_tests_properties(xmltest PROPERTIES  PASS_REGULAR_EXPRESSION ", Fail 0" WORKING_DIRECTORY "C:/Users/nicol/Alt_Bionics/GitLab/surge/firmware/Apps/pinocchio_ik_v3/hand_ik/hand_ik/build/tinyxml2_ext-prefix/src/tinyxml2_ext" _BACKTRACE_TRIPLES "C:/Users/nicol/Alt_Bionics/GitLab/surge/firmware/Apps/pinocchio_ik_v3/hand_ik/hand_ik/build/tinyxml2_ext-prefix/src/tinyxml2_ext/CMakeLists.txt;49;add_test;C:/Users/nicol/Alt_Bionics/GitLab/surge/firmware/Apps/pinocchio_ik_v3/hand_ik/hand_ik/build/tinyxml2_ext-prefix/src/tinyxml2_ext/CMakeLists.txt;0;")
else()
  add_test(xmltest NOT_AVAILABLE)
endif()
