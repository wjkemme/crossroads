#include <iostream>
#include "SafetyChecker.hpp"

using namespace crossroads;

int main()
{
    std::cout << "Crossroads simulator (scaffold)" << std::endl;

    IntersectionState state;
    state.north = LightState::Green;
    state.south = LightState::Red;
    state.east = LightState::Red;
    state.west = LightState::Red;

    SafetyChecker checker;
    bool safe = checker.isSafe(state);
    std::cout << "Initial state is " << (safe ? "SAFE" : "UNSAFE") << std::endl;

    return safe ? 0 : 2;
}
