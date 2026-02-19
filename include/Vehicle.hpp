#pragma once

#include <cstdint>

namespace crossroads
{

    // Direction of the lane where vehicle enters the intersection
    enum class Direction
    {
        North = 0,
        South = 1,
        East = 2,
        West = 3
    };

    // Represents a single vehicle approaching or crossing the intersection
    struct Vehicle
    {
        uint32_t id;          // Unique vehicle identifier
        Direction entry_lane; // Which lane the vehicle is entering from
        double arrival_time;  // When the vehicle arrived at the queue
        double crossing_time; // When the vehicle started crossing (or -1 if not started)
        double exit_time;     // When the vehicle completed crossing (or -1 if not completed)

        // Convenience methods
        Vehicle(uint32_t vid, Direction lane, double arrival)
            : id(vid), entry_lane(lane), arrival_time(arrival),
              crossing_time(-1.0), exit_time(-1.0) {}

        bool isWaiting() const { return crossing_time < 0.0; }
        bool isCrossing() const { return crossing_time >= 0.0 && exit_time < 0.0; }
        bool hasCrossed() const { return exit_time >= 0.0; }

        double waitTime() const
        {
            if (crossing_time < 0.0)
                return -1.0; // Not yet crossed
            return crossing_time - arrival_time;
        }

        double crossingDuration() const
        {
            if (exit_time < 0.0)
                return -1.0; // Not yet complete
            return exit_time - crossing_time;
        }
    };

} // namespace crossroads
