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

TEST_CASE("Transitions enforce G->O->R and R->G with orange timing", "transition")
{
    SafetyChecker c;
    IntersectionState prev;
    prev.north = LightState::Green;
    prev.east = LightState::Red;
    prev.south = LightState::Red;
    prev.west = LightState::Red;

    IntersectionState toOrange = prev;
    toOrange.north = LightState::Orange;
    REQUIRE(c.isValidTransition(prev, toOrange, 0.0) == true);

    IntersectionState toRed = toOrange;
    toRed.north = LightState::Red;
    // orange -> red requires at least ORANGE_DURATION seconds
    REQUIRE(c.isValidTransition(toOrange, toRed, 1.0) == false);
    REQUIRE(c.isValidTransition(toOrange, toRed, SafetyChecker::ORANGE_DURATION) == true);

    // direct green->red is invalid
    IntersectionState direct = prev;
    direct.north = LightState::Red;
    REQUIRE(c.isValidTransition(prev, direct, 0.1) == false);

    // red->green is valid
    IntersectionState red;
    red.north = LightState::Red;
    red.east = LightState::Red;
    red.south = LightState::Red;
    red.west = LightState::Red;
    IntersectionState green = red;
    green.north = LightState::Green;
    REQUIRE(c.isValidTransition(red, green, 0.1) == true);
}
TEST_CASE("Cannot go green if other route is green or orange", "crossing")
{
    SafetyChecker c;

    // Test North-South route: north cannot go green if east OR west is still active
    IntersectionState prev;
    prev.north = LightState::Red;
    prev.south = LightState::Red;
    prev.east = LightState::Green; // EW route is active
    prev.west = LightState::Red;

    IntersectionState attemptGreen = prev;
    attemptGreen.north = LightState::Green;
    REQUIRE(c.isValidTransition(prev, attemptGreen, 0.1) == false);

    // Test North-South route: north cannot go green if west is orange
    prev.east = LightState::Red;
    prev.west = LightState::Orange; // EW route is active

    attemptGreen = prev;
    attemptGreen.north = LightState::Green;
    REQUIRE(c.isValidTransition(prev, attemptGreen, 0.1) == false);

    // Test North-South route: north CAN go green if both east and west are red
    prev.east = LightState::Red;
    prev.west = LightState::Red;

    attemptGreen = prev;
    attemptGreen.north = LightState::Green;
    REQUIRE(c.isValidTransition(prev, attemptGreen, 0.1) == true);

    // Test East-West route: east cannot go green if north OR south is still active
    prev.north = LightState::Green; // NS route is active
    prev.south = LightState::Red;
    prev.east = LightState::Red;
    prev.west = LightState::Red;

    attemptGreen = prev;
    attemptGreen.east = LightState::Green;
    REQUIRE(c.isValidTransition(prev, attemptGreen, 0.1) == false);

    // Test North and South can BOTH be green (same route)
    prev.north = LightState::Red;
    prev.south = LightState::Red;
    prev.east = LightState::Red;
    prev.west = LightState::Red;

    IntersectionState bothNSGreen = prev;
    bothNSGreen.north = LightState::Green;
    bothNSGreen.south = LightState::Green;
    REQUIRE(c.isSafe(bothNSGreen) == true);
}

TEST_CASE("Turning lights enforce cross-traffic constraints", "turning_lights")
{
    SafetyChecker c;

    // Test turnSouthEast cannot go green if West is active
    IntersectionState prev;
    prev.north = LightState::Red;
    prev.south = LightState::Red;
    prev.east = LightState::Red;
    prev.west = LightState::Green; // West is active
    prev.turnSouthEast = LightState::Red;

    IntersectionState attemptTurnGreen = prev;
    attemptTurnGreen.turnSouthEast = LightState::Green;
    REQUIRE(c.isValidTransition(prev, attemptTurnGreen, 0.1) == false);

    // Test turnSouthEast CAN go green if West is red
    prev.west = LightState::Red;
    attemptTurnGreen = prev;
    attemptTurnGreen.turnSouthEast = LightState::Green;
    REQUIRE(c.isValidTransition(prev, attemptTurnGreen, 0.1) == true);

    // Test turnNorthWest cannot go green if East is orange
    prev.north = LightState::Red;
    prev.east = LightState::Orange; // East is active
    prev.turnNorthWest = LightState::Red;

    attemptTurnGreen = prev;
    attemptTurnGreen.turnNorthWest = LightState::Green;
    REQUIRE(c.isValidTransition(prev, attemptTurnGreen, 0.1) == false);

    // Test turnWestSouth cannot go green if North is green
    prev.north = LightState::Green; // North is active
    prev.east = LightState::Red;
    prev.west = LightState::Red;
    prev.turnWestSouth = LightState::Red;

    attemptTurnGreen = prev;
    attemptTurnGreen.turnWestSouth = LightState::Green;
    REQUIRE(c.isValidTransition(prev, attemptTurnGreen, 0.1) == false);

    // Test turnEastNorth cannot go green if South is active
    prev.north = LightState::Red;
    prev.south = LightState::Orange; // South is active
    prev.east = LightState::Red;
    prev.turnEastNorth = LightState::Red;

    attemptTurnGreen = prev;
    attemptTurnGreen.turnEastNorth = LightState::Green;
    REQUIRE(c.isValidTransition(prev, attemptTurnGreen, 0.1) == false);

    // Test turnEastNorth CAN go green if South is red
    prev.south = LightState::Red;
    attemptTurnGreen = prev;
    attemptTurnGreen.turnEastNorth = LightState::Green;
    REQUIRE(c.isValidTransition(prev, attemptTurnGreen, 0.1) == true);
}
