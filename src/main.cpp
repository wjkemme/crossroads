#include <iostream>
#include "SafetyChecker.hpp"

using namespace crossroads;

int main()
{
    std::cout << "Crossroads simulator (scaffold)" << std::endl;

    // Demo: Main direction lights
    std::cout << "\n--- Main Direction Lights ---" << std::endl;
    IntersectionState state;
    state.north = LightState::Green;
    state.south = LightState::Red;
    state.east = LightState::Red;
    state.west = LightState::Red;

    SafetyChecker checker;
    bool safe = checker.isSafe(state);
    std::cout << "NS route green: " << (safe ? "SAFE" : "UNSAFE") << std::endl;

    // Demo: Turning light (South->East turn)
    std::cout << "\n--- Turning Light Demo ---" << std::endl;
    IntersectionState turnState;
    turnState.south = LightState::Red;
    turnState.east = LightState::Red;
    turnState.west = LightState::Red;
    turnState.north = LightState::Red;
    turnState.turnSouthEast = LightState::Green;

    bool turnSafe = checker.isSafe(turnState);
    std::cout << "Turn South->East (when West is red): " << (turnSafe ? "SAFE" : "UNSAFE") << std::endl;

    // Demo: Invalid turning light (would conflict with West)
    std::cout << "\n--- Invalid Turn (Conflict) ---" << std::endl;
    IntersectionState invalidTurn;
    invalidTurn.south = LightState::Red;
    invalidTurn.east = LightState::Red;
    invalidTurn.west = LightState::Green;  // West is active
    invalidTurn.north = LightState::Red;
    invalidTurn.turnSouthEast = LightState::Green;  // Invalid: conflicts with West

    bool conflict = checker.isSafe(invalidTurn);
    std::cout << "Turn South->East (while West is green): " << (conflict ? "SAFE (wrong!)" : "UNSAFE (correct)") << std::endl;

    return 0;
}
