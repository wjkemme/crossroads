#define CATCH_CONFIG_MAIN
#include <catch2/catch_all.hpp>
#include "SafetyChecker.hpp"

using namespace crossroads;

TEST_CASE("SafetyChecker rejects conflicting greens", "safety")
{
    SafetyChecker c;
    IntersectionState s;
    s.north = LightState::Green;
    s.south = LightState::Red;
    s.east = LightState::Green; // conflict with north
    s.west = LightState::Red;
    REQUIRE(c.isSafe(s) == false);
}

TEST_CASE("SafetyChecker accepts single direction green", "safety")
{
    SafetyChecker c;
    IntersectionState s;
    s.north = LightState::Green;
    s.east = LightState::Red;
    s.south = LightState::Red;
    s.west = LightState::Red;
    REQUIRE(c.isSafe(s) == true);
}
