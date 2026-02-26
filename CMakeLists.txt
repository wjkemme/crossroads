cmake_minimum_required(VERSION 3.14)
project(crossroads VERSION 0.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(BUILD_TESTS "Build unit tests" ON)

include(FetchContent)
FetchContent_Declare(
    nlohmann_json
    GIT_REPOSITORY https://github.com/nlohmann/json.git
    GIT_TAG v3.11.3
)
FetchContent_MakeAvailable(nlohmann_json)

find_package(SQLite3 QUIET)

include_directories(${CMAKE_SOURCE_DIR}/include)

add_executable(crossroads
    src/main.cpp
    src/SafetyChecker.cpp
    src/BasicLightController.cpp
    src/TrafficGenerator.cpp
    src/SimulatorEngine.cpp
    src/SimpleHttpUiServer.cpp
    src/IntersectionConfigJson.cpp
    src/db/Database.cpp
)
target_link_libraries(crossroads PRIVATE nlohmann_json::nlohmann_json)
if(SQLite3_FOUND)
    target_link_libraries(crossroads PRIVATE SQLite::SQLite3)
    target_compile_definitions(crossroads PRIVATE CROSSROADS_USE_SQLITE=1)
else()
    message(WARNING "SQLite3 dev package not found. Falling back to file-backed config persistence.")
endif()

if(BUILD_TESTS)
    FetchContent_Declare(
        Catch2
        GIT_REPOSITORY https://github.com/catchorg/Catch2.git
        GIT_TAG v3.4.0
    )
    FetchContent_MakeAvailable(Catch2)

    add_executable(test_safety
        tests/test_safety.cpp
        src/SafetyChecker.cpp
        src/BasicLightController.cpp
        src/TrafficGenerator.cpp
        src/SimulatorEngine.cpp
        src/IntersectionConfigJson.cpp
    )
    target_link_libraries(test_safety PRIVATE Catch2::Catch2WithMain nlohmann_json::nlohmann_json)
    include(CTest)
    add_test(NAME safety_test COMMAND test_safety)
endif()
