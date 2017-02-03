cmake_minimum_required(VERSION 2.8.3)

add_library(research-interface-common INTERFACE)
set_target_properties(research-interface-common PROPERTIES
  INTERFACE_INCLUDE_DIRECTORIES ${CMAKE_CURRENT_LIST_DIR}/include
)
