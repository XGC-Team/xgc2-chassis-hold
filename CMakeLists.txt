cmake_minimum_required(VERSION 3.10)
project(XgcChassisHold VERSION 1.0.0 LANGUAGES CXX)
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
include(CTest)

option(XGC2_CHASSIS_HOLD_WERROR "Treat compiler warnings as errors" OFF)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
find_package(Threads REQUIRED)

# Component core: the domain. Needs the C++ standard library only.
add_library(xgc2_chassis_hold SHARED src/domain.cpp)
add_library(XgcChassisHold::core ALIAS xgc2_chassis_hold)
set_target_properties(xgc2_chassis_hold PROPERTIES EXPORT_NAME core VERSION ${PROJECT_VERSION} SOVERSION 1)
target_include_directories(xgc2_chassis_hold PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(xgc2_chassis_hold PRIVATE Threads::Threads)
target_compile_options(xgc2_chassis_hold PRIVATE -Wall -Wextra -Wpedantic)
if(XGC2_CHASSIS_HOLD_WERROR)
  target_compile_options(xgc2_chassis_hold PRIVATE -Werror)
endif()
install(TARGETS xgc2_chassis_hold EXPORT XgcChassisHoldCoreTargets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(EXPORT XgcChassisHoldCoreTargets NAMESPACE XgcChassisHold:: FILE XgcChassisHoldCoreTargets.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/XgcChassisHold)

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(DIRECTORY contracts/ DESTINATION ${CMAKE_INSTALL_DATADIR}/xgc2-chassis-hold/contracts)
configure_package_config_file(cmake/XgcChassisHoldConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/XgcChassisHoldConfig.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/XgcChassisHold)
write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/XgcChassisHoldConfigVersion.cmake
  VERSION ${PROJECT_VERSION} COMPATIBILITY SameMajorVersion)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/XgcChassisHoldConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/XgcChassisHoldConfigVersion.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/XgcChassisHold)

if(BUILD_TESTING)
  add_subdirectory(test)
endif()
