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
option(BUILD_PYTHON_BINDING "Build python bindings" ON)
option(GENERATE_CLANGD "Generate .clangd in project root" ON)

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  add_compile_definitions("DEBUG")
endif()

message(STATUS "CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}")
# =============================================================================================

# ============================= Dependencies ==================================================
# Pick correct prebuilt archive for this host
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64" OR CMAKE_SYSTEM_PROCESSOR MATCHES "arm64")
  set(PREBUILT_ARCH_DIR "${CMAKE_CURRENT_SOURCE_DIR}/lib/aarch64")
  set(THIRD_PARTY_ARCH "aarch64")
else()
  set(PREBUILT_ARCH_DIR "${CMAKE_CURRENT_SOURCE_DIR}/lib/x86_64")
  set(THIRD_PARTY_ARCH "x86_64")
endif()

# Set up third_party dependencies path
set(THIRD_PARTY_LIB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/lib/${THIRD_PARTY_ARCH}")
set(THIRD_PARTY_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/include")

message(STATUS "Target architecture: ${CMAKE_SYSTEM_PROCESSOR}")
message(STATUS "Using prebuilt libs from: ${PREBUILT_ARCH_DIR}")
message(STATUS "Using third_party libs from: ${THIRD_PARTY_LIB_DIR}")
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
include_directories(BEFORE ${THIRD_PARTY_INCLUDE_DIR})

# Import third_party prebuilt libraries
add_library(fastcdr SHARED IMPORTED)
set_target_properties(fastcdr PROPERTIES
  IMPORTED_LOCATION "${THIRD_PARTY_LIB_DIR}/libfastcdr.so"
)

add_library(fastrtps SHARED IMPORTED)
set_target_properties(fastrtps PROPERTIES
  IMPORTED_LOCATION "${THIRD_PARTY_LIB_DIR}/libfastrtps.so"
)

add_library(foonathan_memory STATIC IMPORTED)
set_target_properties(foonathan_memory PROPERTIES
  IMPORTED_LOCATION "${THIRD_PARTY_LIB_DIR}/libfoonathan_memory-0.7.3.a"
)

# Header-only interface target that consumers link against
add_library(${LIBRARY} INTERFACE)
target_include_directories(${LIBRARY}
  INTERFACE
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${THIRD_PARTY_INCLUDE_DIR}>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_link_libraries(${LIBRARY}
  INTERFACE
    ${LIBRARY}_prebuilt
    fastrtps
    fastcdr
    foonathan_memory
)

install(FILES "${PREBUILT_LIB}" DESTINATION "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}")
install(FILES "${THIRD_PARTY_LIB_DIR}/libfastrtps.so"
              "${THIRD_PARTY_LIB_DIR}/libfastcdr.so"
              "${THIRD_PARTY_LIB_DIR}/libfoonathan_memory-0.7.3.a"
        DESTINATION "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}")
install(DIRECTORY "${CMAKE_SOURCE_DIR}/include/"
  DESTINATION "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_INCLUDEDIR}"
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)
install(DIRECTORY "${THIRD_PARTY_INCLUDE_DIR}/"
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
string(APPEND CONFIG_CONTENT "find_dependency(Threads REQUIRED)\n\n")
string(APPEND CONFIG_CONTENT "# Recreate imported third-party targets\n")
string(APPEND CONFIG_CONTENT "add_library(fastcdr SHARED IMPORTED)\n")
string(APPEND CONFIG_CONTENT "set_target_properties(fastcdr PROPERTIES\n")
string(APPEND CONFIG_CONTENT "  IMPORTED_LOCATION \"\${CMAKE_CURRENT_LIST_DIR}/../../../lib/libfastcdr.so\"\n")
string(APPEND CONFIG_CONTENT "  INTERFACE_INCLUDE_DIRECTORIES \"\${CMAKE_CURRENT_LIST_DIR}/../../../include\"\n")
string(APPEND CONFIG_CONTENT ")\n\n")

string(APPEND CONFIG_CONTENT "add_library(fastrtps SHARED IMPORTED)\n")
string(APPEND CONFIG_CONTENT "set_target_properties(fastrtps PROPERTIES\n")
string(APPEND CONFIG_CONTENT "  IMPORTED_LOCATION \"\${CMAKE_CURRENT_LIST_DIR}/../../../lib/libfastrtps.so\"\n")
string(APPEND CONFIG_CONTENT "  INTERFACE_INCLUDE_DIRECTORIES \"\${CMAKE_CURRENT_LIST_DIR}/../../../include\"\n")
string(APPEND CONFIG_CONTENT ")\n\n")

string(APPEND CONFIG_CONTENT "add_library(foonathan_memory STATIC IMPORTED)\n")
string(APPEND CONFIG_CONTENT "set_target_properties(foonathan_memory PROPERTIES\n")
string(APPEND CONFIG_CONTENT "  IMPORTED_LOCATION \"\${CMAKE_CURRENT_LIST_DIR}/../../../lib/libfoonathan_memory-0.7.3.a\"\n")
string(APPEND CONFIG_CONTENT "  INTERFACE_INCLUDE_DIRECTORIES \"\${CMAKE_CURRENT_LIST_DIR}/../../../include\"\n")
string(APPEND CONFIG_CONTENT ")\n\n")

string(APPEND CONFIG_CONTENT "# Import prebuilt library\n")
string(APPEND CONFIG_CONTENT "add_library(${LIBRARY}_prebuilt STATIC IMPORTED)\n")
string(APPEND CONFIG_CONTENT "set_target_properties(${LIBRARY}_prebuilt PROPERTIES\n")
string(APPEND CONFIG_CONTENT "  IMPORTED_LOCATION \"\${CMAKE_CURRENT_LIST_DIR}/../../../lib/libbooster_robotics_sdk.a\"\n")
string(APPEND CONFIG_CONTENT ")\n\n")

string(APPEND CONFIG_CONTENT "# Import glibc compatibility shim\n")
string(APPEND CONFIG_CONTENT "add_library(glibc_compat STATIC IMPORTED)\n")
string(APPEND CONFIG_CONTENT "set_target_properties(glibc_compat PROPERTIES\n")
string(APPEND CONFIG_CONTENT "  IMPORTED_LOCATION \"\${CMAKE_CURRENT_LIST_DIR}/../../../lib/libglibc_compat.a\"\n")
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

# ==============================================================================
#  PYTHON BINDINGS (The Pixi-Build-CMake Way)
# ==============================================================================
option(BUILD_PYTHON_BINDING "Build python bindings" ON) # Default to ON for this package
set(BINDINGS_SRC "")
if(EXISTS "${CMAKE_SOURCE_DIR}/python/binding.cpp")
  set(BINDINGS_SRC "${CMAKE_SOURCE_DIR}/python/binding.cpp")
endif()
if(BUILD_PYTHON_BINDING)
    # Python (Interpreter + Development.Module)
    set(_PY_COMPS Interpreter Development.Module)
    find_package(Python QUIET COMPONENTS ${_PY_COMPS})
    if(NOT Python_FOUND)
      find_package(Python3 REQUIRED COMPONENTS ${_PY_COMPS})
      set(Python_EXECUTABLE ${Python3_EXECUTABLE})
      set(Python_SITEARCH  ${Python3_SITEARCH})
    endif()
    
    # Locate nanobind's CMake package via the Python module
    execute_process(
      COMMAND "${Python_EXECUTABLE}" -m nanobind --cmake_dir
      OUTPUT_STRIP_TRAILING_WHITESPACE OUTPUT_VARIABLE _NB_CMAKE_DIR
      RESULT_VARIABLE _NB_RET
    )
    if(NOT _NB_RET EQUAL 0)
    message(FATAL_ERROR "Failed to locate nanobind via 'python -m nanobind --cmake_dir'. "
    "Is 'nanobind' installed in this pixi env?")
    endif()
    set(nanobind_DIR "${_NB_CMAKE_DIR}")
    find_package(nanobind CONFIG REQUIRED)

    # Determine Python's site-packages (platlib)
    execute_process(
        COMMAND "${Python_EXECUTABLE}" -c "import sysconfig; print(sysconfig.get_path('platlib', vars={'platbase': '.', 'base': '.'}))"
        OUTPUT_VARIABLE PYTHON_SITE_PACKAGES
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    string(REGEX REPLACE "^/" "" PYTHON_SITE_PACKAGES "${PYTHON_SITE_PACKAGES}")
    message(STATUS "Python bindings will be installed to: \${CMAKE_INSTALL_PREFIX}/${PYTHON_SITE_PACKAGES}")

    # Auto-detect module name from NB_MODULE(...) in Bindings.cpp (e.g., 'circuspy')
    file(READ "${BINDINGS_SRC}" _NB_SRC)
    string(REGEX MATCH "NB_MODULE\\s*\\(([A-Za-z_][A-Za-z0-9_]*)" _m "${_NB_SRC}")
    if(NOT _m)
      message(FATAL_ERROR "Could not find NB_MODULE(...) in ${BINDINGS_SRC}")
    endif()
    set(PY_MODULE_NAME "${CMAKE_MATCH_1}")
    message(STATUS "nanobind: detected module name '${PY_MODULE_NAME}' from ${BINDINGS_SRC}")

    # Build the nanobind module
    nanobind_add_module(${PY_MODULE_NAME} ${BINDINGS_SRC})
    target_link_libraries(${PY_MODULE_NAME} PRIVATE ${LIBRARY})
    
    # Set RPATH to find third_party libraries in the same directory (site-packages)
    # This allows the module to find libfastrtps.so and libfastcdr.so at runtime
    set_target_properties(${PY_MODULE_NAME} PROPERTIES
      BUILD_RPATH "${THIRD_PARTY_LIB_DIR}"
      INSTALL_RPATH "$ORIGIN"
      INSTALL_RPATH_USE_LINK_PATH TRUE
    )

    # Install third_party libraries FIRST so they're available when the module loads
    install(
        FILES "${THIRD_PARTY_LIB_DIR}/libfastrtps.so"
              "${THIRD_PARTY_LIB_DIR}/libfastcdr.so"
        DESTINATION ${PYTHON_SITE_PACKAGES}
    )
    
    # Create version-specific symlinks matching the SONAME that the prebuilt SDK expects
    # libfastrtps.so has SONAME libfastrtps.so.2.13
    # libfastcdr.so has SONAME libfastcdr.so.2
    install(CODE "
        execute_process(COMMAND ln -sf libfastrtps.so \${CMAKE_INSTALL_PREFIX}/${PYTHON_SITE_PACKAGES}/libfastrtps.so.2.13)
        execute_process(COMMAND ln -sf libfastcdr.so \${CMAKE_INSTALL_PREFIX}/${PYTHON_SITE_PACKAGES}/libfastcdr.so.2)
    ")

    # Install Python module into the environment's site-packages
    install(
        TARGETS ${PY_MODULE_NAME}
        LIBRARY DESTINATION ${PYTHON_SITE_PACKAGES}
    )
    
    # Generate stub files (.pyi) after installation
    # Set LD_LIBRARY_PATH so stubgen can find the bundled libraries and conda dependencies
    install(CODE "
        message(STATUS \"Generating stubs for ${PY_MODULE_NAME}...\")
        set(ENV{PYTHONPATH} \"\${CMAKE_INSTALL_PREFIX}/${PYTHON_SITE_PACKAGES}:\$ENV{PYTHONPATH}\")
        set(ENV{LD_LIBRARY_PATH} \"\${CMAKE_INSTALL_PREFIX}/${PYTHON_SITE_PACKAGES}:\${CMAKE_INSTALL_PREFIX}/lib:\$ENV{LD_LIBRARY_PATH}\")
        execute_process(
            COMMAND ${Python_EXECUTABLE} -m nanobind.stubgen 
                -m ${PY_MODULE_NAME}
                -o \${CMAKE_INSTALL_PREFIX}/${PYTHON_SITE_PACKAGES}/${PY_MODULE_NAME}.pyi
            RESULT_VARIABLE _stubgen_result
            OUTPUT_VARIABLE _stubgen_output
            ERROR_VARIABLE _stubgen_error
        )
        if(NOT _stubgen_result EQUAL 0)
            message(WARNING \"Failed to generate stubs for ${PY_MODULE_NAME}: \${_stubgen_error}\")
        else()
            message(STATUS \"Successfully generated stubs at \${CMAKE_INSTALL_PREFIX}/${PYTHON_SITE_PACKAGES}/${PY_MODULE_NAME}.pyi\")
        endif()
    ")
    
endif()