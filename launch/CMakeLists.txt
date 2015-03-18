cmake_minimum_required(VERSION 2.8.3)

project(launch)

find_package(ament_cmake REQUIRED)

if(AMENT_ENABLE_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_python_install_package(${PROJECT_NAME})

ament_package()

if(AMENT_ENABLE_TESTING)
  ament_add_nose_test(nosetests test)
endif()
