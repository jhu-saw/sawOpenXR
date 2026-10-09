cmake_minimum_required(VERSION 3.16)
project(saw_openxr VERSION 0.1.0 LANGUAGES CXX)

if(POLICY CMP0144)
  cmake_policy(SET CMP0144 NEW)
endif()

find_package(ament_cmake REQUIRED)
find_package(cisst 1.5 REQUIRED COMPONENTS
  cisstCommon
  cisstVector
  cisstOSAbstraction
  cisstMultiTask
  cisstParameterTypes)
find_package(PkgConfig REQUIRED)
find_package(Threads REQUIRED)
find_package(Vulkan REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED IMPORTED_TARGET
  gstreamer-1.0
  gstreamer-app-1.0
  gstreamer-video-1.0)

find_path(OPENXR_INCLUDE_DIR openxr/openxr.h REQUIRED)
find_library(OPENXR_LOADER_LIBRARY NAMES openxr_loader REQUIRED)
find_program(GLSLANG_VALIDATOR NAMES glslangValidator REQUIRED)

if(NOT CISST_HAS_JSON)
  message(FATAL_ERROR "saw_openxr requires cisst built with CISST_HAS_JSON")
endif()

set(SAW_OPENXR_SHADER_DIR "${CMAKE_CURRENT_BINARY_DIR}/shaders")
set(SAW_OPENXR_INSTALLED_SHADER_DIR
  "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/shaders")
set(SAW_OPENXR_VERTEX_SHADER "${SAW_OPENXR_SHADER_DIR}/texture.vert.spv")
set(SAW_OPENXR_FRAGMENT_SHADER "${SAW_OPENXR_SHADER_DIR}/texture.frag.spv")

add_custom_command(
  OUTPUT "${SAW_OPENXR_VERTEX_SHADER}"
  COMMAND "${CMAKE_COMMAND}" -E make_directory "${SAW_OPENXR_SHADER_DIR}"
  COMMAND "${GLSLANG_VALIDATOR}" -V --target-env vulkan1.0 -S vert
          -o "${SAW_OPENXR_VERTEX_SHADER}"
          "${CMAKE_CURRENT_SOURCE_DIR}/../core/runtime/shaders/texture.vert"
  DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/../core/runtime/shaders/texture.vert"
  VERBATIM)

add_custom_command(
  OUTPUT "${SAW_OPENXR_FRAGMENT_SHADER}"
  COMMAND "${CMAKE_COMMAND}" -E make_directory "${SAW_OPENXR_SHADER_DIR}"
  COMMAND "${GLSLANG_VALIDATOR}" -V --target-env vulkan1.0 -S frag
          -o "${SAW_OPENXR_FRAGMENT_SHADER}"
          "${CMAKE_CURRENT_SOURCE_DIR}/../core/runtime/shaders/texture.frag"
  DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/../core/runtime/shaders/texture.frag"
  VERBATIM)

add_custom_target(sawOpenXRShaders
  DEPENDS "${SAW_OPENXR_VERTEX_SHADER}" "${SAW_OPENXR_FRAGMENT_SHADER}")

add_library(sawOpenXR SHARED
  ../core/components/include/sawOpenXR/sawOpenXRExport.h
  ../core/components/include/sawOpenXR/OpenXRRuntime.h
  ../core/components/include/sawOpenXR/mtsOpenXR.h
  ../core/components/code/mtsOpenXR.cpp
  ../core/runtime/code/OpenXRRuntime.cpp)

add_dependencies(sawOpenXR sawOpenXRShaders)

target_compile_features(sawOpenXR PUBLIC cxx_std_20)

target_compile_definitions(sawOpenXR
  PRIVATE SAW_OPENXR_SHADER_DIR="${SAW_OPENXR_INSTALLED_SHADER_DIR}")

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(sawOpenXR
    PRIVATE -Wall -Wextra -Wno-missing-field-initializers)
endif()
target_include_directories(sawOpenXR
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../core/components/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${OPENXR_INCLUDE_DIR})
target_link_libraries(sawOpenXR
  PUBLIC ${cisst_LIBRARIES}
  PRIVATE
    ${OPENXR_LOADER_LIBRARY}
    PkgConfig::GSTREAMER
    Threads::Threads
    Vulkan::Vulkan)

install(DIRECTORY ../core/components/include/sawOpenXR/
  DESTINATION include/sawOpenXR)
install(TARGETS sawOpenXR
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(FILES
  "${SAW_OPENXR_VERTEX_SHADER}"
  "${SAW_OPENXR_FRAGMENT_SHADER}"
  DESTINATION share/${PROJECT_NAME}/shaders)
install(DIRECTORY ../share/
  DESTINATION share/${PROJECT_NAME}/config
  FILES_MATCHING PATTERN "*.json")
install(DIRECTORY ../share/schemas/
  DESTINATION share/${PROJECT_NAME}/schemas
  FILES_MATCHING PATTERN "*.json")
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

ament_export_include_directories(include)
ament_export_libraries(sawOpenXR)
ament_package()
