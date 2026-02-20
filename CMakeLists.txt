cmake_minimum_required(VERSION 3.14)
project(crossroads VERSION 0.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(BUILD_TESTS "Build unit tests" ON)

include_directories(${CMAKE_SOURCE_DIR}/include)

add_executable(crossroads
    src/main.cpp
    src/SafetyChecker.cpp
    src/BasicLightController.cpp
    src/TrafficGenerator.cpp
    src/SimulatorEngine.cpp
)

if(BUILD_TESTS)
    include(FetchContent)
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
    )
    target_link_libraries(test_safety PRIVATE Catch2::Catch2WithMain)
    include(CTest)
    add_test(NAME safety_test COMMAND test_safety)
endif()
