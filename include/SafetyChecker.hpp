#pragma once

#include "Intersection.hpp"

namespace crossroads
{

    class SafetyChecker
    {
    public:
        SafetyChecker() = default;
        // Returns true if the given intersection state is safe
        bool isSafe(const IntersectionState &state) const;
    };

} // namespace crossroads
