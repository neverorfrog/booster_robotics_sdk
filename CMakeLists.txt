# ============================= PROJECT SETUP =================================================
cmake_policy(SET CMP0048 NEW)
cmake_minimum_required(VERSION 3.22)
project(booster_robotics_sdk VERSION 0.1 LANGUAGES CXX)
set(LIBRARY ${PROJECT_NAME})
set(PROJECT booster_robotics_sdk)

# Provide standard install directory variables like CMAKE_INSTALL_LIBDIR
include(GNUInstallDirs)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
add_compile_definitions(PROJECT_ROOT="${CMAKE_SOURCE_DIR}")

file(WRITE "${CMAKE_CURRENT_SOURCE_DIR}/.clangd"
"CompileFlags:
  CompilationDatabase: ${CMAKE_CURRENT_BINARY_DIR}
")
# =============================================================================================

# ============================ BUILD OPTIONS ==================================================
option(BUILD_TESTS "Build tests" OFF)
option(GENERATE_CLANGD "Generate .clangd in project root" ON)

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_compile_definitions("DEBUG")
endif()

message(STATUS "CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}")
# =============================================================================================

# ============================= Dependencies ==================================================
find_package(Threads REQUIRED)

# Pick correct prebuilt archive for this host
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64" OR CMAKE_SYSTEM_PROCESSOR MATCHES "arm64")
    set(PREBUILT_ARCH_DIR "${CMAKE_CURRENT_SOURCE_DIR}/lib/aarch64")
else()
    set(PREBUILT_ARCH_DIR "${CMAKE_CURRENT_SOURCE_DIR}/lib/x86_64")
endif()

message(STATUS "Target architecture: ${CMAKE_SYSTEM_PROCESSOR}")
message(STATUS "Using prebuilt libs from: ${PREBUILT_ARCH_DIR}")
# =============================================================================================

# ============================ LIBRARY ==================================================
file(GLOB_RECURSE HEADERS ${CMAKE_SOURCE_DIR}/include/*.h)

set(PREBUILT_LIB "${PREBUILT_ARCH_DIR}/libbooster_robotics_sdk.a")
if(NOT EXISTS "${PREBUILT_LIB}")
    message(FATAL_ERROR "Prebuilt library not found: ${PREBUILT_LIB}")
endif()

add_library(${LIBRARY}_prebuilt STATIC IMPORTED)
set_target_properties(${LIBRARY}_prebuilt PROPERTIES
  IMPORTED_LOCATION "${PREBUILT_LIB}"
)
include_directories(BEFORE ${CMAKE_SOURCE_DIR}/include)

# Header-only interface target that consumers link against
add_library(${LIBRARY} INTERFACE)
target_include_directories(${LIBRARY}
  INTERFACE
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_link_libraries(${LIBRARY}
  INTERFACE
    ${LIBRARY}_prebuilt
    Threads::Threads
    ${CMAKE_DL_LIBS}
    rt
)

install(FILES "${PREBUILT_LIB}" DESTINATION "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}")

install(DIRECTORY "${CMAKE_SOURCE_DIR}/include/"
  DESTINATION "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_INCLUDEDIR}"
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

# Export the INTERFACE target so downstream consumers can find it via CMake config
install(TARGETS ${LIBRARY}
    EXPORT ${LIBRARY}Targets
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(
    EXPORT ${LIBRARY}Targets
    FILE ${LIBRARY}Targets.cmake
    NAMESPACE booster::
    DESTINATION lib/cmake/${LIBRARY}
)

include(CMakePackageConfigHelpers)

# Generate a config file that recreates the imported targets
set(CONFIG_CONTENT "include(CMakeFindDependencyMacro)\n")
string(APPEND CONFIG_CONTENT "find_dependency(Threads)\n\n")

string(APPEND CONFIG_CONTENT "# Import prebuilt library\n")
string(APPEND CONFIG_CONTENT "add_library(${LIBRARY}_prebuilt STATIC IMPORTED)\n")
string(APPEND CONFIG_CONTENT "set_target_properties(${LIBRARY}_prebuilt PROPERTIES\n")
string(APPEND CONFIG_CONTENT "  IMPORTED_LOCATION \"\${CMAKE_CURRENT_LIST_DIR}/../../../lib/libbooster_robotics_sdk.a\"\n")
string(APPEND CONFIG_CONTENT ")\n\n")

string(APPEND CONFIG_CONTENT "include(\"\${CMAKE_CURRENT_LIST_DIR}/${LIBRARY}Targets.cmake\")\n")
file(
    WRITE "${CMAKE_CURRENT_BINARY_DIR}/${LIBRARY}Config.cmake"
    "${CONFIG_CONTENT}"
)

write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/${LIBRARY}ConfigVersion.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

install(
    FILES
        "${CMAKE_CURRENT_BINARY_DIR}/${LIBRARY}Config.cmake"
        "${CMAKE_CURRENT_BINARY_DIR}/${LIBRARY}ConfigVersion.cmake"
    DESTINATION lib/cmake/${LIBRARY}
)

# ==============================================================================================
