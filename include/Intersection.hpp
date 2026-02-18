#pragma once

namespace crossroads
{

    enum class LightState
    {
        Red,
        Yellow,
        Green
    };

    struct IntersectionState
    {
        // Order: North, East, South, West
        LightState north{LightState::Red};
        LightState east{LightState::Red};
        LightState south{LightState::Red};
        LightState west{LightState::Red};
    };

} // namespace crossroads
