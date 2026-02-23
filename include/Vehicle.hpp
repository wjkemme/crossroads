#pragma once

#include <cstdint>
#include <cmath>

namespace crossroads
{
    enum class Direction
    {
        North = 0,
        South = 1,
        East = 2,
        West = 3
    };

    struct Vehicle
    {
        uint32_t id;
        Direction entry_lane;
        double arrival_time;
        double crossing_time;
        double exit_time;
        double current_speed;    // m/s, range [0, 10]
        double position_in_lane; // meters from queue start
        bool turning;            // true when vehicle uses turn lane
        uint8_t queue_index;     // 0 or 1 for straight lanes, 2 for turn lane

        Vehicle(uint32_t vid, Direction lane, double arrival)
            : id(vid), entry_lane(lane), arrival_time(arrival),
              crossing_time(-1.0), exit_time(-1.0),
              current_speed(0.0), position_in_lane(0.0), turning(false), queue_index(0) {}

        bool isWaiting() const { return crossing_time < 0.0; }
        bool isCrossing() const { return crossing_time >= 0.0 && exit_time < 0.0; }
        bool hasCrossed() const { return exit_time >= 0.0; }

        double waitTime() const
        {
            if (crossing_time < 0.0)
                return -1.0;
            return crossing_time - arrival_time;
        }

        double crossingDuration() const
        {
            if (exit_time < 0.0)
                return -1.0;
            return exit_time - crossing_time;
        }

        void updateSpeed(double target_speed, double dt_seconds)
        {
            const double ACCEL = 3.0; // m/s² - realistic car acceleration
            target_speed = std::max(0.0, std::min(10.0, target_speed));

            double delta = target_speed - current_speed;
            double max_change = ACCEL * dt_seconds;

            if (std::abs(delta) <= max_change)
                current_speed = target_speed;
            else if (delta > 0.0)
                current_speed += max_change;
            else
                current_speed -= max_change;
        }

        double getCrossingDuration(size_t queue_length) const
        {
            double density = std::min(1.0, queue_length / 10.0);
            double base = 2.5 + (density * 2.0); // 2.5-4.5 sec range
            // Turning vehicles travel longer arc path, need more time
            return turning ? base * 1.6 : base;
        }
    };

} // namespace crossroads