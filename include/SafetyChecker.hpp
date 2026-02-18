#pragma once

#include "Intersection.hpp"

namespace crossroads
{

    class SafetyChecker
    {
    public:
        SafetyChecker() = default;

        // Public validation methods
        bool isSafe(const IntersectionState &state) const;
        bool isValidTransition(const IntersectionState &prev, const IntersectionState &next, double dt_seconds) const;

        static constexpr double ORANGE_DURATION = 2.0; // seconds

    private:
        // Helper methods for isSafe()
        bool hasConflictingGreens(const IntersectionState &state) const;
        bool checkTurningLightSafety(const IntersectionState &state) const;

        // Helper methods for isValidTransition()
        bool checkPerLightTransitions(const IntersectionState &prev, const IntersectionState &next) const;
        bool checkOrangeTiming(const IntersectionState &prev, const IntersectionState &next, double dt_seconds) const;
        bool checkCrossingLightSafety(const IntersectionState &prev, const IntersectionState &next) const;
        bool checkTurningLightTransitions(const IntersectionState &next) const;
    };

} // namespace crossroads
