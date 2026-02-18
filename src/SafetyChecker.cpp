#include "SafetyChecker.hpp"

namespace crossroads
{

    // Basic safety rule: no conflicting greens between NS and EW
    bool SafetyChecker::isSafe(const IntersectionState &state) const
    {
        bool nsGreen = (state.north == LightState::Green) || (state.south == LightState::Green);
        bool ewGreen = (state.east == LightState::Green) || (state.west == LightState::Green);
        return !(nsGreen && ewGreen);
    }

} // namespace crossroads
