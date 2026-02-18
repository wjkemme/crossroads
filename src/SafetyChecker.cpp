#include "SafetyChecker.hpp"

namespace crossroads
{

    bool SafetyChecker::isSafe(const IntersectionState &state) const
    {
        return hasConflictingGreens(state) && checkTurningLightSafety(state);
    }

    bool SafetyChecker::hasConflictingGreens(const IntersectionState &state) const
    {
        bool nsGreen = (state.north == LightState::Green) || (state.south == LightState::Green);
        bool ewGreen = (state.east == LightState::Green) || (state.west == LightState::Green);
        return !(nsGreen && ewGreen);
    }

    bool SafetyChecker::checkTurningLightSafety(const IntersectionState &state) const
    {
        auto is_active = [](LightState s)
        { return s == LightState::Green || s == LightState::Orange; };

        // turnSouthEast cannot be green if West is active
        if (state.turnSouthEast == LightState::Green && is_active(state.west))
            return false;

        // turnNorthWest cannot be green if East is active
        if (state.turnNorthWest == LightState::Green && is_active(state.east))
            return false;

        // turnWestSouth cannot be green if North is active
        if (state.turnWestSouth == LightState::Green && is_active(state.north))
            return false;

        // turnEastNorth cannot be green if South is active
        if (state.turnEastNorth == LightState::Green && is_active(state.south))
            return false;

        return true;
    }

    bool SafetyChecker::isValidTransition(const IntersectionState &prev, const IntersectionState &next, double dt_seconds) const
    {
        return checkPerLightTransitions(prev, next) &&
               checkOrangeTiming(prev, next, dt_seconds) &&
               isSafe(next) &&
               checkCrossingLightSafety(prev, next) &&
               checkTurningLightTransitions(next);
    }

    bool SafetyChecker::checkPerLightTransitions(const IntersectionState &prev, const IntersectionState &next) const
    {
        auto valid_for_light = [](LightState p, LightState n)
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

        // Check all 8 lights for valid transitions
        return valid_for_light(prev.north, next.north) &&
               valid_for_light(prev.east, next.east) &&
               valid_for_light(prev.south, next.south) &&
               valid_for_light(prev.west, next.west) &&
               valid_for_light(prev.turnSouthEast, next.turnSouthEast) &&
               valid_for_light(prev.turnNorthWest, next.turnNorthWest) &&
               valid_for_light(prev.turnWestSouth, next.turnWestSouth) &&
               valid_for_light(prev.turnEastNorth, next.turnEastNorth);
    }

    bool SafetyChecker::checkOrangeTiming(const IntersectionState &prev, const IntersectionState &next, double dt_seconds) const
    {
        auto check_orange_duration = [dt_seconds](LightState p, LightState n)
        {
            return !((p == LightState::Orange && n == LightState::Red) && dt_seconds < ORANGE_DURATION);
        };

        // Verify orange lasted long enough for all 8 lights
        return check_orange_duration(prev.north, next.north) &&
               check_orange_duration(prev.east, next.east) &&
               check_orange_duration(prev.south, next.south) &&
               check_orange_duration(prev.west, next.west) &&
               check_orange_duration(prev.turnSouthEast, next.turnSouthEast) &&
               check_orange_duration(prev.turnNorthWest, next.turnNorthWest) &&
               check_orange_duration(prev.turnWestSouth, next.turnWestSouth) &&
               check_orange_duration(prev.turnEastNorth, next.turnEastNorth);
    }

    bool SafetyChecker::checkCrossingLightSafety(const IntersectionState &prev, const IntersectionState &next) const
    {
        auto is_active = [](LightState s)
        { return s == LightState::Green || s == LightState::Orange; };

        // Detect transitions to green for NS and EW routes
        bool north_going_green = (prev.north != LightState::Green && next.north == LightState::Green);
        bool south_going_green = (prev.south != LightState::Green && next.south == LightState::Green);
        bool east_going_green = (prev.east != LightState::Green && next.east == LightState::Green);
        bool west_going_green = (prev.west != LightState::Green && next.west == LightState::Green);

        // If NS route is transitioning to green, EW must be inactive
        if ((north_going_green || south_going_green) && (is_active(next.east) || is_active(next.west)))
            return false;

        // If EW route is transitioning to green, NS must be inactive
        if ((east_going_green || west_going_green) && (is_active(next.north) || is_active(next.south)))
            return false;

        return true;
    }

    bool SafetyChecker::checkTurningLightTransitions(const IntersectionState &next) const
    {
        auto is_active = [](LightState s)
        { return s == LightState::Green || s == LightState::Orange; };

        // turnSouthEast cannot go green if West is active
        if (next.turnSouthEast == LightState::Green && is_active(next.west))
            return false;

        // turnNorthWest cannot go green if East is active
        if (next.turnNorthWest == LightState::Green && is_active(next.east))
            return false;

        // turnWestSouth cannot go green if North is active
        if (next.turnWestSouth == LightState::Green && is_active(next.north))
            return false;

        // turnEastNorth cannot go green if South is active
        if (next.turnEastNorth == LightState::Green && is_active(next.south))
            return false;

        return true;
    }

} // namespace crossroads
