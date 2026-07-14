cmake_minimum_required(VERSION 3.14)
project(SimulationPlugin VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(RPC_GATEWAY_PLUGIN_OUTPUT_DIR
    "${CMAKE_CURRENT_SOURCE_DIR}/../../rpc_gateway/plugins"
    CACHE PATH "Output directory for compiled rpc_gateway plugins")

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# --------- Dependencies ---------
include(cmake/CPM.cmake)

set(MUJOCO_BUILD_EXAMPLES OFF CACHE BOOL MuJoCo FORCE)
set(MUJOCO_BUILD_SIMULATE OFF CACHE BOOL MuJoCo FORCE)
set(MUJOCO_BUILD_STUDIO OFF CACHE BOOL MuJoCo FORCE)
set(MUJOCO_BUILD_TESTS OFF CACHE BOOL MuJoCo FORCE)
set(MUJOCO_TEST_PYTHON_UTIL OFF CACHE BOOL MuJoCo FORCE)
set(MUJOCO_USE_FILAMENT OFF CACHE BOOL MuJoCo FORCE)
CPMAddPackage(
  NAME mujoco
  VERSION 3.4.0
  GITHUB_REPOSITORY google-deepmind/mujoco
  GIT_TAG 3.4.0
)

find_package(nlohmann_json REQUIRED)
find_package(xplugin-dev REQUIRED)
find_package(Threads REQUIRED)

file(GLOB_RECURSE sources CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/source/*.cpp")
add_library(${PROJECT_NAME} SHARED ${sources})
target_compile_definitions(${PROJECT_NAME} PRIVATE PLUGIN_LOG_NAME=SimulationPlugin)

target_link_libraries(${PROJECT_NAME} PRIVATE
  nlohmann_json xplugin::sdk mujoco::mujoco Threads::Threads
)

set_target_properties(${PROJECT_NAME} PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY "${RPC_GATEWAY_PLUGIN_OUTPUT_DIR}"
  RUNTIME_OUTPUT_DIRECTORY "${RPC_GATEWAY_PLUGIN_OUTPUT_DIR}"
  CXX_VISIBILITY_PRESET hidden
  C_VISIBILITY_PRESET hidden
  VISIBILITY_INLINES_HIDDEN ON
)

foreach(config DEBUG RELEASE RELWITHDEBINFO MINSIZEREL)
  set_target_properties(${PROJECT_NAME} PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY_${config} "${RPC_GATEWAY_PLUGIN_OUTPUT_DIR}"
    RUNTIME_OUTPUT_DIRECTORY_${config} "${RPC_GATEWAY_PLUGIN_OUTPUT_DIR}"
  )
endforeach()

add_custom_command(TARGET ${PROJECT_NAME} PRE_LINK
  COMMAND ${CMAKE_COMMAND} -E make_directory "${RPC_GATEWAY_PLUGIN_OUTPUT_DIR}"
)

enable_testing()
add_executable(simulation_manager_smoke
  tests/simulation_manager_smoke.cpp
  source/simulation_instance.cpp
  source/simulation_instance_manager.cpp
  source/simulation_compiler.cpp
  source/simulation_scene_manager.cpp
  source/simulation_task_manager.cpp
  source/simulation_record_manager.cpp
  source/simulation_model_validator.cpp
  source/simulation_model_registry.cpp
)
target_link_libraries(simulation_manager_smoke PRIVATE
  nlohmann_json mujoco::mujoco Threads::Threads
)

add_test(NAME simulation_manager_smoke COMMAND simulation_manager_smoke)