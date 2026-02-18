#pragma once

namespace crossroads
{

    enum class LightState
    {
        Red,
        Orange,
        Green
    };

    struct IntersectionState
    {
        // Main direction lights: Order: North, East, South, West
        LightState north{LightState::Red};
        LightState east{LightState::Red};
        LightState south{LightState::Red};
        LightState west{LightState::Red};

        // Turning lights (direction A -> direction B)
        LightState turnSouthEast{LightState::Red}; // South -> East (enabled if West is Red)
        LightState turnNorthWest{LightState::Red}; // North -> West (enabled if East is Red)
        LightState turnWestSouth{LightState::Red}; // West -> South (enabled if North is Red)
        LightState turnEastNorth{LightState::Red}; // East -> North (enabled if South is Red)
    };

} // namespace crossroads
