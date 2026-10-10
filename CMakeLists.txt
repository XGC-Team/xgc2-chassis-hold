cmake_minimum_required(VERSION 3.10)
project(XgcChassisHold VERSION 0.2.0 LANGUAGES CXX)
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
include(CTest)

option(XGC2_CHASSIS_HOLD_WERROR "Treat compiler warnings as errors" OFF)
option(XGC2_CHASSIS_HOLD_BUILD_UDP "Build component udp: the udp.v1 endpoint and the xgc2-hold tool (needs XgcXrpc udp)" ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
find_package(Threads REQUIRED)
find_package(XgcNlohmannJson REQUIRED)

# Component core: the domain and the transport-neutral JSON service. Needs the C++ standard library and
# nlohmann-json at build time only; no header exposes it.
add_library(xgc2_chassis_hold SHARED src/domain.cpp src/service.cpp)
add_library(XgcChassisHold::core ALIAS xgc2_chassis_hold)
set_target_properties(xgc2_chassis_hold PROPERTIES EXPORT_NAME core VERSION ${PROJECT_VERSION} SOVERSION 1)
target_include_directories(xgc2_chassis_hold PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(xgc2_chassis_hold PRIVATE nlohmann_json::nlohmann_json Threads::Threads)
target_compile_options(xgc2_chassis_hold PRIVATE -Wall -Wextra -Wpedantic)
if(XGC2_CHASSIS_HOLD_WERROR)
  target_compile_options(xgc2_chassis_hold PRIVATE -Werror)
endif()
install(TARGETS xgc2_chassis_hold EXPORT XgcChassisHoldCoreTargets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(EXPORT XgcChassisHoldCoreTargets NAMESPACE XgcChassisHold:: FILE XgcChassisHoldCoreTargets.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/XgcChassisHold)

# Component udp: the udp.v1 endpoint for drivers and the xgc2-hold command line tool. Needs the XRPC udp component
# (OpenSSL libcrypto and POSIX sockets only, GNU 7.5 and newer); no xrpc header is part of ours.
if(XGC2_CHASSIS_HOLD_BUILD_UDP)
  find_package(XgcXrpc 0.1.0 CONFIG REQUIRED COMPONENTS udp)
  add_library(xgc2_chassis_hold_udp SHARED src/udp_endpoint.cpp)
  add_library(XgcChassisHold::udp ALIAS xgc2_chassis_hold_udp)
  set_target_properties(xgc2_chassis_hold_udp PROPERTIES EXPORT_NAME udp VERSION ${PROJECT_VERSION} SOVERSION 1
    INSTALL_RPATH "$ORIGIN")
  target_link_libraries(xgc2_chassis_hold_udp PUBLIC xgc2_chassis_hold PRIVATE XgcXrpc::udp Threads::Threads)
  target_compile_options(xgc2_chassis_hold_udp PRIVATE -Wall -Wextra -Wpedantic)
  add_executable(xgc2-hold tools/xgc2_hold.cpp)
  set_target_properties(xgc2-hold PROPERTIES INSTALL_RPATH "$ORIGIN/../${CMAKE_INSTALL_LIBDIR}")
  target_link_libraries(xgc2-hold PRIVATE xgc2_chassis_hold XgcXrpc::udp nlohmann_json::nlohmann_json)
  target_compile_options(xgc2-hold PRIVATE -Wall -Wextra -Wpedantic)
  if(XGC2_CHASSIS_HOLD_WERROR)
    target_compile_options(xgc2_chassis_hold_udp PRIVATE -Werror)
    target_compile_options(xgc2-hold PRIVATE -Werror)
  endif()
  install(TARGETS xgc2_chassis_hold_udp EXPORT XgcChassisHoldUdpTargets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})
  install(EXPORT XgcChassisHoldUdpTargets NAMESPACE XgcChassisHold:: FILE XgcChassisHoldUdpTargets.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/XgcChassisHold)
  install(TARGETS xgc2-hold RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
endif()

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
