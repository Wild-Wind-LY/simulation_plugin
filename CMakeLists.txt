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

# Bundled first-party MuJoCo plugins (elasticity/actuator/sensor/sdf). Each
# registers itself via a __attribute__((constructor)) (mjPLUGIN_LIB_INIT in
# mjplugin.h) as soon as its shared library is loaded, so linking them here is
# sufficient for <extension><plugin plugin="mujoco...."/></extension> scenes
# to resolve at mj_loadXML time -- no runtime mj_loadPluginLibrary needed.
# --no-as-needed is required around them: we never call a symbol from these
# libraries directly (the whole point is their load-time side effect), so the
# linker's default --as-needed would otherwise drop the NEEDED entry entirely
# and the constructor would never run.
target_link_libraries(${PROJECT_NAME} PRIVATE
  nlohmann_json xplugin::sdk mujoco::mujoco Threads::Threads
  "-Wl,--no-as-needed" elasticity actuator sensor sdf_plugin "-Wl,--as-needed"
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

