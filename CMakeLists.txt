# ============================= PROJECT SETUP ==============
cmake_policy(SET CMP0048 NEW)
cmake_minimum_required(VERSION 3.22)
project(booster_robotics_sdk VERSION 0.1 LANGUAGES CXX)
set(LIBRARY ${PROJECT_NAME})
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
file(WRITE "${CMAKE_CURRENT_SOURCE_DIR}/.clangd"
  "CompileFlags:\n  CompilationDatabase: ${CMAKE_CURRENT_BINARY_DIR}\n")
# ======================================================


# ========== Dependencies ============================
set(DEPENDENCIES
  ament_cmake
  fastdds
  fastcdr
  foonathan_memory
)

foreach(PACKAGE ${DEPENDENCIES})
    find_package(${PACKAGE} REQUIRED)
    if(NOT ${PACKAGE}_FOUND)
        message(FATAL_ERROR "Package ${PACKAGE} not found!")
    else()
        message(STATUS "Found package: ${PACKAGE}")
    endif()
endforeach()
# =======================================================

# ============================ LIBRARY ==================
include(GNUInstallDirs)
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
)

target_link_libraries(${LIBRARY}
  INTERFACE
    ${LIBRARY}_prebuilt
    fastdds
    fastcdr
    foonathan_memory
)

install(FILES "${PREBUILT_LIB}" DESTINATION "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}")
install(DIRECTORY "${CMAKE_SOURCE_DIR}/include/"
  DESTINATION "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_INCLUDEDIR}"
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp")

# Export the INTERFACE target so downstream consumers can find it via CMake config
install(TARGETS ${LIBRARY}
    EXPORT ${LIBRARY}Targets
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
# ==============================================

# ========== AMENT PACKAGE =========================
install(DIRECTORY include/ DESTINATION include/)
# install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
# install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(${DEPENDENCIES})

ament_package()
# ==================================================


# TODO: Add executables in framework, not here
# add_executable(b1_loco_example_client example/high_level/b1_loco_example_client.cpp)
# add_executable(b1_arm_sdk_example_client example/high_level/b1_arm_sdk_example.cpp)
# add_executable(b1_7dof_arm_sdk_example_client example/high_level/b1_7dof_arm_sdk_example.cpp)
# add_executable(b1_low_level_publisher example/low_level/low_level_publisher.cpp)
# add_executable(b1_low_level_subscriber example/low_level/low_level_subscriber.cpp)
# add_executable(low_level_hand_data_subscriber example/low_level/low_level_hand_data_subscriber.cpp)
# add_executable(b1_low_sdk_example example/low_level/b1_low_sdk_example.cpp)
# add_executable(b1_7dof_arm_low_sdk_example example/low_level/b1_7dof_arm_low_sdk_example.cpp)
# add_executable(odometer_example example/low_level/odometer_example.cpp)