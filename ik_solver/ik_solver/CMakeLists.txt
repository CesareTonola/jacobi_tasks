cmake_minimum_required(VERSION 3.16)
project(ik_solver VERSION 0.0.1)

# Build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif()

# Compiler flags
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  if(CMAKE_BUILD_TYPE MATCHES "Release")
    add_compile_options(-funroll-loops -Wall -Ofast -flto -O3)
  else()
    add_compile_options(-Wall -g -Og)
  endif()
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Dependencies
find_package(pinocchio REQUIRED)
find_package(pybind11 REQUIRED)

# ------------------------------------------------------------------------------
# Core library
# ------------------------------------------------------------------------------
add_library(${PROJECT_NAME} SHARED
  src/ik_solver.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME} PUBLIC pinocchio::pinocchio)

# Alias
add_library("${PROJECT_NAME}::${PROJECT_NAME}" ALIAS ${PROJECT_NAME})

# Install headers
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include)

# ------------------------------------------------------------------------------
# Executable
# ------------------------------------------------------------------------------
add_executable(compute_ik src/compute_ik.cpp)
target_compile_definitions(compute_ik PRIVATE IK_SOLVER_DIR="${CMAKE_CURRENT_LIST_DIR}")
target_link_libraries(compute_ik PRIVATE ${PROJECT_NAME})
target_include_directories(compute_ik PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

# ------------------------------------------------------------------------------
# Python bindings (using pybind11)
# ------------------------------------------------------------------------------
add_library(ik_solver_py MODULE src/bindings.cpp)
target_link_libraries(ik_solver_py PRIVATE ${PROJECT_NAME} pinocchio::pinocchio pybind11::module)
target_include_directories(ik_solver_py PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include)

set_target_properties(ik_solver_py PROPERTIES
  PREFIX ""
  OUTPUT_NAME "ik_solver_py"
  SUFFIX ".so"
  LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/pybind11
)

add_custom_command(TARGET ik_solver_py POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy
          $<TARGET_FILE:ik_solver_py>
          ${CMAKE_CURRENT_SOURCE_DIR}/scripts/ik_solver/ik_solver_py.so
)

install(TARGETS ik_solver_py
  LIBRARY DESTINATION lib
)

# ------------------------------------------------------------------------------
# Install targets
# ------------------------------------------------------------------------------
install(TARGETS ${PROJECT_NAME} compute_ik
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(EXPORT ${PROJECT_NAME}Targets
  DESTINATION share/${PROJECT_NAME}/cmake
  NAMESPACE ${PROJECT_NAME}::
  FILE ${PROJECT_NAME}Targets.cmake
)
