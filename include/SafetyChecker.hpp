#pragma once

#include "Intersection.hpp"

namespace crossroads
{

    class SafetyChecker
    {
    public:
        SafetyChecker() = default;
        // Returns true if the given intersection state is safe
        // Returns true if transition from prev->next with elapsed time dt_seconds is valid
        bool isValidTransition(const IntersectionState &prev, const IntersectionState &next, double dt_seconds) const;
        static constexpr double ORANGE_DURATION = 2.0; // seconds
        bool isSafe(const IntersectionState &state) const;
    };

} // namespace crossroads
