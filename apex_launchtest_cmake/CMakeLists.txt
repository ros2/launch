cmake_minimum_required(VERSION 3.5)

project(apex_launchtest_cmake NONE)

find_package(ament_cmake_core REQUIRED)

ament_package(
  CONFIG_EXTRAS "${PROJECT_NAME}-extras.cmake"
)

install(
  DIRECTORY cmake
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(apex_launchtest_cmake_pytests test)

  include(cmake/add_apex_launchtest.cmake)

  ament_index_has_resource(APEX_LAUNCHTEST_INSTALL_PREFIX packages apex_launchtest)
  if(NOT APEX_LAUNCHTEST_INSTALL_PREFIX)
      message(FATAL_ERROR "apex_launchtest package not found")
  endif()

  # Test argument passing.  This test won't pass unless you give it an argument
  add_apex_launchtest(
    "${APEX_LAUNCHTEST_INSTALL_PREFIX}/share/apex_launchtest/examples/args.test.py"
    ARGS "dut_node_arg:=--anything"
  )
endif()
