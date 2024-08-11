cmake_minimum_required(VERSION 3.10)

project(TermalConductivityInGraphs)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

file(GLOB_RECURSE SOURCES "src/*.cpp" "include/*.hpp")

add_executable(TermalConductivityInGraphs ${SOURCES})

target_include_directories(TermalConductivityInGraphs PRIVATE ${CMAKE_SOURCE_DIR}/libs/eigen-3.4.0)

add_subdirectory(libs/json)
target_link_libraries(TermalConductivityInGraphs PRIVATE nlohmann_json::nlohmann_json)

