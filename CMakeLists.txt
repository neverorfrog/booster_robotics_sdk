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
find_package(fastrtps REQUIRED)
find_package(fastcdr REQUIRED)
find_package(foonathan_memory REQUIRED)
find_package(Threads REQUIRED)
# =============================================================================================

# ============================ LIBRARY ==================================================
file(GLOB_RECURSE HEADERS ${CMAKE_SOURCE_DIR}/include/*.h)

# Pick correct prebuilt archive for this host
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64" OR CMAKE_SYSTEM_PROCESSOR MATCHES "arm64")
  set(PREBUILT_ARCH_DIR "${CMAKE_CURRENT_SOURCE_DIR}/lib/aarch64")
else()
  set(PREBUILT_ARCH_DIR "${CMAKE_CURRENT_SOURCE_DIR}/lib/x86_64")
endif()

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
    fastrtps
    fastcdr
    foonathan_memory
    Threads::Threads
)

# Some toolchains/linkers require `-pthread` (not just `-lpthread`) so ensure
# consumers of the INTERFACE target receive the compile+link flags.
if(NOT MSVC)
  target_compile_options(${LIBRARY} INTERFACE -pthread)
  target_link_options(${LIBRARY} INTERFACE -pthread)
  
  # Workaround for prebuilt library built with newer glibc: allow undefined weak symbols
  # These symbols (__libc_single_threaded, pthread_cond_clockwait) will be resolved at runtime
  target_link_options(${LIBRARY} INTERFACE "LINKER:--warn-unresolved-symbols")
endif()

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

    # Install Python module into the environment's site-packages
    install(
        TARGETS ${PY_MODULE_NAME}
        LIBRARY DESTINATION ${PYTHON_SITE_PACKAGES}
    )
    
    # Generate stub files (.pyi) automatically after installation
    # nanobind.stubgen writes directly to the output path as a file, not a directory
    install(CODE "
        message(STATUS \"Generating stubs for ${PY_MODULE_NAME}...\")
        set(ENV{PYTHONPATH} \"\${CMAKE_INSTALL_PREFIX}/${PYTHON_SITE_PACKAGES}:\$ENV{PYTHONPATH}\")
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

# ============================== EXECUTABLES ==============================
# Only build examples when not doing a package build
# (avoids glibc version conflicts with prebuilt library during conda build)
option(BUILD_EXAMPLES "Build example executables" ON)

if(BUILD_EXAMPLES)
  add_executable(b1_loco_example_client example/high_level/b1_loco_example_client.cpp)
  add_executable(b1_arm_sdk_example_client example/high_level/b1_arm_sdk_example.cpp)
  add_executable(b1_7dof_arm_sdk_example_client example/high_level/b1_7dof_arm_sdk_example.cpp)
  add_executable(b1_low_level_publisher example/low_level/low_level_publisher.cpp)
  add_executable(b1_low_level_subscriber example/low_level/low_level_subscriber.cpp)
  add_executable(low_level_hand_data_subscriber example/low_level/low_level_hand_data_subscriber.cpp)
  add_executable(b1_low_sdk_example example/low_level/b1_low_sdk_example.cpp)
  add_executable(b1_7dof_arm_low_sdk_example example/low_level/b1_7dof_arm_low_sdk_example.cpp)
  add_executable(odometer_example example/low_level/odometer_example.cpp)

  # Link executables against the SDK interface target so they pull in the
  # prebuilt static library and its public dependencies.
  target_link_libraries(b1_loco_example_client PRIVATE ${LIBRARY})
  target_link_libraries(b1_arm_sdk_example_client PRIVATE ${LIBRARY})
  target_link_libraries(b1_7dof_arm_sdk_example_client PRIVATE ${LIBRARY})
  target_link_libraries(b1_low_level_publisher PRIVATE ${LIBRARY})
  target_link_libraries(b1_low_level_subscriber PRIVATE ${LIBRARY})
  target_link_libraries(low_level_hand_data_subscriber PRIVATE ${LIBRARY})
  target_link_libraries(b1_low_sdk_example PRIVATE ${LIBRARY})
  target_link_libraries(b1_7dof_arm_low_sdk_example PRIVATE ${LIBRARY})
  target_link_libraries(odometer_example PRIVATE ${LIBRARY})

  install(TARGETS
    b1_loco_example_client
    b1_arm_sdk_example_client
    b1_7dof_arm_sdk_example_client
    b1_low_level_publisher
    b1_low_level_subscriber
    low_level_hand_data_subscriber
    b1_low_sdk_example
    b1_7dof_arm_low_sdk_example
    odometer_example
    RUNTIME DESTINATION bin
  )
endif()

# ==================================================================================