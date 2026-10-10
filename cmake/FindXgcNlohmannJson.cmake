# Provides nlohmann_json::nlohmann_json for nlohmann-json 3.x (CMake package) and for 2.1.1 (Ubuntu 18.04
# "nlohmann-json-dev": headers only, no CMake package).
if(TARGET nlohmann_json::nlohmann_json)
  return()
endif()
find_package(nlohmann_json CONFIG QUIET)
if(NOT TARGET nlohmann_json::nlohmann_json)
  find_path(XGC2_NLOHMANN_INCLUDE_DIR nlohmann/json.hpp)
  if(NOT XGC2_NLOHMANN_INCLUDE_DIR)
    message(FATAL_ERROR "nlohmann-json (2.1.1 or newer) was not found")
  endif()
  add_library(nlohmann_json::nlohmann_json INTERFACE IMPORTED)
  set_target_properties(nlohmann_json::nlohmann_json PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${XGC2_NLOHMANN_INCLUDE_DIR}")
endif()
