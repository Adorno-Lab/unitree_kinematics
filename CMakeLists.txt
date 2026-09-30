cmake_minimum_required(VERSION 3.16)

project(unitree_kinematics
    VERSION 0.1.0
    DESCRIPTION "DQ Robotics kinematic models of Unitree robots (Z1, B1+Z1) and their CoppeliaSim ZMQ counterparts"
    LANGUAGES CXX
)

# ---------------------------------------------------------------------------
# Project-wide settings
# ---------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

option(BUILD_SHARED_LIBS "Build using shared libraries" ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

# Homebrew installs outside CMake's default search paths on Apple silicon.
if(APPLE)
    list(APPEND CMAKE_PREFIX_PATH /opt/homebrew /usr/local)
endif()

# ---------------------------------------------------------------------------
# Dependencies
# ---------------------------------------------------------------------------

# Eigen3 is header-only; Eigen3::Eigen is the standard imported target its own
# Config file provides.
find_package(Eigen3 REQUIRED)

# DQ Robotics (https://dqrobotics.github.io) and its CoppeliaSim ZMQ interface
# (https://github.com/dqrobotics/cpp-interface-coppeliasim-zmq) do not ship a
# CMake package config -- they're installed as plain shared libraries with no
# discoverable Config.cmake. find_library()/find_path() locate them and give us
# absolute paths we can embed in our exported target, which is more portable
# than passing bare "-ldqrobotics" link flags through to consumers. Both honor
# CMAKE_PREFIX_PATH, so a non-sudo install under e.g. ~/opt is found as well.
find_path(DQROBOTICS_INCLUDE_DIR dqrobotics/DQ.h)
find_library(DQROBOTICS_LIBRARY dqrobotics)
if(NOT DQROBOTICS_INCLUDE_DIR OR NOT DQROBOTICS_LIBRARY)
    message(FATAL_ERROR
        "DQ Robotics not found. Install it (see https://dqrobotics.github.io) "
        "before configuring unitree_kinematics, or point CMAKE_PREFIX_PATH at "
        "its install prefix."
    )
endif()

find_path(DQROBOTICS_COPPELIASIM_ZMQ_INCLUDE_DIR
    dqrobotics/interfaces/coppeliasim/DQ_CoppeliaSimInterfaceZMQ.h
)
find_library(DQROBOTICS_COPPELIASIM_ZMQ_LIBRARY dqrobotics-interface-coppeliasim-zmq)
if(NOT DQROBOTICS_COPPELIASIM_ZMQ_INCLUDE_DIR OR NOT DQROBOTICS_COPPELIASIM_ZMQ_LIBRARY)
    message(FATAL_ERROR
        "DQ Robotics CoppeliaSim ZMQ interface not found. Install it (see "
        "https://github.com/dqrobotics/cpp-interface-coppeliasim-zmq) before "
        "configuring unitree_kinematics, or point CMAKE_PREFIX_PATH at its "
        "install prefix."
    )
endif()

# ---------------------------------------------------------------------------
# Library: unitree_kinematics
# ---------------------------------------------------------------------------
add_library(unitree_kinematics
    src/robots/CFFSerialRobot.cpp
    src/robots/UnitreeZ1Robot.cpp
    src/robots/UnitreeB1Z1MobileRobot.cpp
    src/interfaces/coppeliasim/robots/UnitreeB1Z1CoppeliaSimZMQRobot.cpp
)
add_library(unitree_kinematics::unitree_kinematics ALIAS unitree_kinematics)

target_compile_features(unitree_kinematics PUBLIC cxx_std_17)

target_include_directories(unitree_kinematics PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    ${DQROBOTICS_INCLUDE_DIR}
    ${DQROBOTICS_COPPELIASIM_ZMQ_INCLUDE_DIR}
)

# All PUBLIC: the installed headers derive from DQ Robotics / CoppeliaSim ZMQ
# classes and expose Eigen types, so consumers need them too.
target_link_libraries(unitree_kinematics
    PUBLIC
        Eigen3::Eigen
        ${DQROBOTICS_LIBRARY}
        ${DQROBOTICS_COPPELIASIM_ZMQ_LIBRARY}
)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(unitree_kinematics PRIVATE
        -Werror=return-type -Wall -Wextra -Wmissing-declarations
        -Wredundant-decls -Woverloaded-virtual
    )
endif()

set_target_properties(unitree_kinematics PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_VERSION_MAJOR}
)

# ---------------------------------------------------------------------------
# Install: headers, library, and the exported CMake package
# ---------------------------------------------------------------------------
install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(TARGETS unitree_kinematics
    EXPORT unitree_kinematicsTargets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(EXPORT unitree_kinematicsTargets
    FILE unitree_kinematicsTargets.cmake
    NAMESPACE unitree_kinematics::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/unitree_kinematics
)

configure_package_config_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/unitree_kinematicsConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/unitree_kinematicsConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/unitree_kinematics
)

write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/unitree_kinematicsConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/unitree_kinematicsConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/unitree_kinematicsConfigVersion.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/unitree_kinematics
)

# How to use
# find_package(unitree_kinematics REQUIRED)
# target_link_libraries(my_target PRIVATE unitree_kinematics::unitree_kinematics)
