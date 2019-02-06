cmake_minimum_required(VERSION 3.5)

project(dummy)

add_executable(locking locking.cpp)

install(
  TARGETS locking
  RUNTIME DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}
)
