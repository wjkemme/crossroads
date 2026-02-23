#include "TrafficGenerator.hpp"

namespace crossroads
{
    namespace
    {
        constexpr double CAR_LENGTH_METERS = 4.0;
        constexpr double STOPLINE_TARGET_METERS = 69.0;                                      // 1m vóór de streep (streep op 70m)
        constexpr double STOPPED_GAP_METERS = 2.0;                                           // 2m bumper-bumper bij stilstand
        constexpr double MIN_FRONT_DISTANCE_METERS = CAR_LENGTH_METERS + STOPPED_GAP_METERS; // 6m front-to-front
        constexpr double FOLLOWING_TIME_SECONDS = 1.5;                                       // 1.5s following bij rijden
    }

    TrafficGenerator::TrafficGenerator(double rate)
        : arrival_rate(rate), time_accumulated(0.0), next_vehicle_id(1)
    {
    }

    std::deque<Vehicle> &TrafficGenerator::getQueueByDirection(Direction dir)
    {
        switch (dir)
        {
        case Direction::North:
            return north_queue;
        case Direction::South:
            return south_queue;
        case Direction::East:
            return east_queue;
        case Direction::West:
            return west_queue;
        }
        return north_queue;
    }

    const std::deque<Vehicle> &TrafficGenerator::getQueueByDirection(Direction dir) const
    {
        switch (dir)
        {
        case Direction::North:
            return north_queue;
        case Direction::South:
            return south_queue;
        case Direction::East:
            return east_queue;
        case Direction::West:
            return west_queue;
        }
        return north_queue;
    }

    double TrafficGenerator::getNextSpawnInterval()
    {
        if (arrival_rate <= 0.0)
            return 1000000.0;
        return 1.0 / arrival_rate;
    }

    void TrafficGenerator::generateTraffic(double dt_seconds, double current_time)
    {
        time_accumulated += dt_seconds;
        double spawn_interval = getNextSpawnInterval();

        while (time_accumulated >= spawn_interval)
        {
            time_accumulated -= spawn_interval;

            Direction directions[] = {Direction::North, Direction::South,
                                      Direction::East, Direction::West};

            for (Direction dir : directions)
            {
                Vehicle v(next_vehicle_id++, dir, current_time);
                v.turning = (v.id % 5 == 0);

                // Assign lane: turning vehicles go to lane 2, straight vehicles alternate 0/1
                if (v.turning)
                {
                    v.queue_index = 2;
                }
                else
                {
                    // Use a simple counter per direction to alternate lanes
                    // Count non-turning vehicles in this direction's queue
                    auto &queue = getQueueByDirection(dir);
                    size_t straight_count = 0;
                    for (const auto &veh : queue)
                    {
                        if (!veh.turning)
                            straight_count++;
                    }
                    v.queue_index = straight_count % 2;
                }

                auto &queue = getQueueByDirection(dir);
                if (!queue.empty())
                {
                    v.position_in_lane = queue.back().position_in_lane - MIN_FRONT_DISTANCE_METERS;
                }
                queue.push_back(v);
            }
        }
    }

    bool TrafficGenerator::startCrossing(Direction lane, uint32_t vehicle_id, double current_time)
    {
        auto &queue = getQueueByDirection(lane);

        if (queue.empty())
            return false;

        Vehicle &front = queue.front();
        if (front.id != vehicle_id)
            return false;

        front.crossing_time = current_time;
        return true;
    }

    bool TrafficGenerator::completeCrossing(uint32_t vehicle_id, double current_time)
    {
        for (auto dir : {Direction::North, Direction::South, Direction::East, Direction::West})
        {
            auto &queue = getQueueByDirection(dir);

            if (!queue.empty() && queue.front().id == vehicle_id)
            {
                Vehicle crossed = queue.front();
                queue.pop_front();
                crossed.exit_time = current_time;
                crossed_vehicles.push_back(crossed);
                return true;
            }
        }

        return false;
    }

    size_t TrafficGenerator::getQueueLength(Direction lane) const
    {
        switch (lane)
        {
        case Direction::North:
            return north_queue.size();
        case Direction::South:
            return south_queue.size();
        case Direction::East:
            return east_queue.size();
        case Direction::West:
            return west_queue.size();
        }
        return 0;
    }

    size_t TrafficGenerator::getTotalWaiting() const
    {
        return north_queue.size() + south_queue.size() +
               east_queue.size() + west_queue.size();
    }

    Vehicle *TrafficGenerator::peekNextVehicle(Direction lane)
    {
        auto &queue = getQueueByDirection(lane);
        return queue.empty() ? nullptr : &queue.front();
    }

    double TrafficGenerator::getAverageWaitTime() const
    {
        if (crossed_vehicles.empty())
            return 0.0;

        double total_wait = 0.0;
        for (const auto &v : crossed_vehicles)
        {
            total_wait += v.waitTime();
        }

        return total_wait / crossed_vehicles.size();
    }

    void TrafficGenerator::reset()
    {
        north_queue.clear();
        south_queue.clear();
        east_queue.clear();
        west_queue.clear();
        crossed_vehicles.clear();
        time_accumulated = 0.0;
        next_vehicle_id = 1;
    }

    void TrafficGenerator::updateVehicleSpeeds(double dt_seconds, const std::array<bool, 4> &lane_can_move)
    {
        const double STOP_LINE_POSITION = 70.0;
        const double STOP_TARGET = STOPLINE_TARGET_METERS; // 4m voor de streep
        const double MAX_SPEED = 10.0;                     // m/s
        const double BRAKE_DECEL = 4.5;                    // m/s^2

        for (int dir = 0; dir < 4; ++dir)
        {
            Direction d = static_cast<Direction>(dir);
            auto &queue = getQueueByDirection(d);
            bool can_move = lane_can_move[dir];

            for (size_t i = 0; i < queue.size(); ++i)
            {
                Vehicle &vehicle = queue[i];

                if (vehicle.isCrossing())
                    continue; // Already crossing, skip speed updates

                double target_speed = MAX_SPEED;

                // Nearest non-crossing vehicle ahead in the SAME lane (queue_index)
                int ahead_idx = static_cast<int>(i) - 1;
                while (ahead_idx >= 0)
                {
                    const Vehicle &ahead = queue[static_cast<size_t>(ahead_idx)];
                    if (!ahead.isCrossing() && ahead.queue_index == vehicle.queue_index)
                        break;
                    --ahead_idx;
                }
                const bool has_ahead = ahead_idx >= 0;

                // Desired following distance: stopped 2m gap; moving time-gap
                auto getDesiredGap = [](double speed)
                {
                    if (speed < 0.5)
                    {
                        return MIN_FRONT_DISTANCE_METERS; // 6m front-to-front (4m car + 2m gap)
                    }
                    return CAR_LENGTH_METERS + FOLLOWING_TIME_SECONDS * speed;
                };

                // Compute target position respecting stop target and front vehicle in same lane
                double target_position = STOP_TARGET;
                if (has_ahead)
                {
                    const Vehicle &ahead = queue[static_cast<size_t>(ahead_idx)];
                    target_position = std::min(target_position, ahead.position_in_lane - MIN_FRONT_DISTANCE_METERS);
                }

                if (has_ahead)
                {
                    const Vehicle &ahead = queue[static_cast<size_t>(ahead_idx)];
                    double spacing = ahead.position_in_lane - vehicle.position_in_lane;
                    double desired_gap = getDesiredGap(vehicle.current_speed);

                    if (can_move)
                    {
                        // Time-gap following
                        if (spacing < desired_gap)
                        {
                            double ratio = spacing / desired_gap;
                            target_speed = std::min(target_speed, ahead.current_speed + (MAX_SPEED - ahead.current_speed) * ratio);
                        }
                        if (spacing < MIN_FRONT_DISTANCE_METERS)
                        {
                            target_speed = 0.0;
                        }
                    }
                    else
                    {
                        // Red/orange: brake to stop target (or behind front car)
                        double dist_to_target = target_position - vehicle.position_in_lane;
                        if (dist_to_target <= 0.0)
                        {
                            target_speed = 0.0;
                        }
                        else
                        {
                            double safe_speed = std::sqrt(2.0 * BRAKE_DECEL * dist_to_target);
                            target_speed = std::min(target_speed, safe_speed);
                        }
                    }
                }
                else
                {
                    // No vehicle ahead
                    if (!can_move && vehicle.position_in_lane < STOP_LINE_POSITION)
                    {
                        double dist_to_stop = STOP_TARGET - vehicle.position_in_lane;
                        if (dist_to_stop <= 0.0)
                        {
                            target_speed = 0.0;
                        }
                        else
                        {
                            double safe_speed = std::sqrt(2.0 * BRAKE_DECEL * dist_to_stop);
                            target_speed = std::min(target_speed, safe_speed);
                        }
                    }
                }

                vehicle.updateSpeed(target_speed, dt_seconds);

                // Update position
                if (!vehicle.isCrossing())
                {
                    vehicle.position_in_lane += vehicle.current_speed * dt_seconds;

                    // Clamp to stop target if red/orange
                    if (!can_move)
                    {
                        if (vehicle.position_in_lane > target_position)
                        {
                            vehicle.position_in_lane = target_position;
                            vehicle.current_speed = 0.0;
                        }
                    }

                    // Maintain minimum distance behind vehicle ahead
                    if (has_ahead)
                    {
                        const Vehicle &ahead = queue[static_cast<size_t>(ahead_idx)];
                        double max_pos = ahead.position_in_lane - MIN_FRONT_DISTANCE_METERS;
                        if (vehicle.position_in_lane > max_pos)
                        {
                            vehicle.position_in_lane = max_pos;
                            vehicle.current_speed = std::min(vehicle.current_speed, ahead.current_speed);
                        }
                    }
                }
            }
        }
    }

    double TrafficGenerator::getAverageQueueDensity(Direction dir) const
    {
        return std::min(1.0, static_cast<double>(getQueueByDirection(dir).size()) / LANE_CAPACITY);
    }

    std::vector<LaneVehicleState> TrafficGenerator::getLaneVehicleStates(Direction dir) const
    {
        std::vector<LaneVehicleState> states;
        const auto &queue = getQueueByDirection(dir);
        states.reserve(queue.size());
        const size_t queue_len = queue.size();

        for (const auto &vehicle : queue)
        {
            LaneVehicleState state;
            state.id = vehicle.id;
            state.position_in_lane = vehicle.position_in_lane;
            state.speed = vehicle.current_speed;
            state.crossing = vehicle.isCrossing();
            state.turning = vehicle.turning;
            state.crossing_time = vehicle.crossing_time;
            state.crossing_duration = vehicle.getCrossingDuration(queue_len);
            state.queue_index = vehicle.queue_index;
            states.push_back(state);
        }

        return states;
    }

} // namespace crossroads
