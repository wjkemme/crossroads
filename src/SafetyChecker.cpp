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

    bool SafetyChecker::isValidTransition(const IntersectionState &prev, const IntersectionState &next, double dt_seconds) const
    {
        auto valid_for_light = [&](LightState p, LightState n)
        {
            if (p == n)
                return true;
            if (p == LightState::Green && n == LightState::Orange)
                return true;
            if (p == LightState::Orange && n == LightState::Red)
                return true;
            if (p == LightState::Red && n == LightState::Green)
                return true;
            return false;
        };

        // per-light transition checks
        if (!valid_for_light(prev.north, next.north))
            return false;
        if (!valid_for_light(prev.east, next.east))
            return false;
        if (!valid_for_light(prev.south, next.south))
            return false;
        if (!valid_for_light(prev.west, next.west))
            return false;

        // Timing: ensure orange lasted at least ORANGE_DURATION when transitioning Orange->Red
        // We use the provided dt_seconds as the elapsed time for the transition step.
        if ((prev.north == LightState::Orange && next.north == LightState::Red) && dt_seconds < ORANGE_DURATION)
            return false;
        if ((prev.east == LightState::Orange && next.east == LightState::Red) && dt_seconds < ORANGE_DURATION)
            return false;
        if ((prev.south == LightState::Orange && next.south == LightState::Red) && dt_seconds < ORANGE_DURATION)
            return false;
        if ((prev.west == LightState::Orange && next.west == LightState::Red) && dt_seconds < ORANGE_DURATION)
            return false;

        // final safety check: next state must itself be safe (no conflicting greens)
        if (!isSafe(next))
            return false;

        // Crossing-light safety: cannot transition TO green if the opposite light is still green or orange
        // North/South are opposites, East/West are opposites
        auto is_active = [](LightState s) { return s == LightState::Green || s == LightState::Orange; };

        if (next.north == LightState::Green && is_active(next.south))
            return false;
        if (next.south == LightState::Green && is_active(next.north))
            return false;
        if (next.east == LightState::Green && is_active(next.west))
            return false;
        if (next.west == LightState::Green && is_active(next.east))
            return false;

        return true;
    }

} // namespace crossroads
