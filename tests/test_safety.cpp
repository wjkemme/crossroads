#define CATCH_CONFIG_MAIN
#include <catch2/catch_all.hpp>
#include "SafetyChecker.hpp"
#include "BasicLightController.hpp"

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

TEST_CASE("BasicLightController initializes NS green", "controller")
{
    BasicLightController ctrl(5.0, 5.0);
    IntersectionState state = ctrl.getCurrentState();

    REQUIRE(state.north == LightState::Green);
    REQUIRE(state.south == LightState::Green);
    REQUIRE(state.east == LightState::Red);
    REQUIRE(state.west == LightState::Red);
}

TEST_CASE("BasicLightController cycles through phases", "controller")
{
    BasicLightController ctrl(1.0, 1.0); // 1 second green phases
    SafetyChecker checker;

    // Initial state: NS green
    auto state = ctrl.getCurrentState();
    REQUIRE(state.north == LightState::Green);
    REQUIRE(state.south == LightState::Green);

    // After 1+ second: should transition to NS orange
    ctrl.tick(1.1);
    state = ctrl.getCurrentState();
    REQUIRE(state.north == LightState::Orange);
    REQUIRE(state.south == LightState::Orange);
    REQUIRE(state.east == LightState::Red);
    REQUIRE(state.west == LightState::Red);

    // After 2 more seconds: should be EW green (orange took 2 seconds)
    ctrl.tick(2.1);
    state = ctrl.getCurrentState();
    REQUIRE(state.north == LightState::Red);
    REQUIRE(state.south == LightState::Red);
    REQUIRE(state.east == LightState::Green);
    REQUIRE(state.west == LightState::Green);

    // After 1 more second: should be EW orange
    ctrl.tick(1.1);
    state = ctrl.getCurrentState();
    REQUIRE(state.east == LightState::Orange);
    REQUIRE(state.west == LightState::Orange);

    // All transitions should be safe
    REQUIRE(checker.isSafe(state) == true);
}

TEST_CASE("BasicLightController reset works", "controller")
{
    BasicLightController ctrl(3.0, 3.0);

    // Advance time
    ctrl.tick(5.1);
    auto state = ctrl.getCurrentState();

    // Reset and verify initial state
    ctrl.reset();
    state = ctrl.getCurrentState();

    // Should be back to NS green
    REQUIRE(state.north == LightState::Green);
    REQUIRE(state.south == LightState::Green);
    REQUIRE(state.east == LightState::Red);
    REQUIRE(state.west == LightState::Red);
}

TEST_CASE("All four turning lights work correctly", "turning_lights_all")
{
    SafetyChecker c;

    // Test 1: Zuid->Oost (South->East) - allowed when West is Red
    IntersectionState state1;
    state1.north = LightState::Red;
    state1.south = LightState::Red;
    state1.east = LightState::Red;
    state1.west = LightState::Red;
    state1.turnSouthEast = LightState::Green;
    state1.turnNorthWest = LightState::Red;
    state1.turnWestSouth = LightState::Red;
    state1.turnEastNorth = LightState::Red;
    REQUIRE(c.isSafe(state1) == true);

    // Test 2: Oost->Noord (East->North) - allowed when South is Red
    IntersectionState state2;
    state2.north = LightState::Red;
    state2.south = LightState::Red;
    state2.east = LightState::Red;
    state2.west = LightState::Red;
    state2.turnSouthEast = LightState::Red;
    state2.turnNorthWest = LightState::Red;
    state2.turnWestSouth = LightState::Red;
    state2.turnEastNorth = LightState::Green;
    REQUIRE(c.isSafe(state2) == true);

    // Test 3: Noord->West (North->West) - allowed when East is Red
    IntersectionState state3;
    state3.north = LightState::Red;
    state3.south = LightState::Red;
    state3.east = LightState::Red;
    state3.west = LightState::Red;
    state3.turnSouthEast = LightState::Red;
    state3.turnNorthWest = LightState::Green;
    state3.turnWestSouth = LightState::Red;
    state3.turnEastNorth = LightState::Red;
    REQUIRE(c.isSafe(state3) == true);

    // Test 4: West->Zuid (West->South) - allowed when North is Red
    IntersectionState state4;
    state4.north = LightState::Red;
    state4.south = LightState::Red;
    state4.east = LightState::Red;
    state4.west = LightState::Red;
    state4.turnSouthEast = LightState::Red;
    state4.turnNorthWest = LightState::Red;
    state4.turnWestSouth = LightState::Green;
    state4.turnEastNorth = LightState::Red;
    REQUIRE(c.isSafe(state4) == true);
}
