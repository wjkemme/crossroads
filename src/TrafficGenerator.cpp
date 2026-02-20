#include "TrafficGenerator.hpp"

namespace crossroads
{
    namespace
    {
        constexpr double CAR_LENGTH_METERS = 4.0;
        constexpr double MIN_FREE_GAP_METERS = CAR_LENGTH_METERS * 0.5;
        constexpr double MIN_FRONT_DISTANCE_METERS = CAR_LENGTH_METERS + MIN_FREE_GAP_METERS;
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

        for (int dir = 0; dir < 4; ++dir)
        {
            Direction d = static_cast<Direction>(dir);
            auto &queue = getQueueByDirection(d);

            for (size_t i = 0; i < queue.size(); ++i)
            {
                Vehicle &vehicle = queue[i];

                if (vehicle.isCrossing())
                    continue; // Already crossing, skip speed updates

                double target_speed = 10.0; // Default max speed

                // Front vehicle must stop at red light
                if (i == 0 && !lane_can_move[dir] && vehicle.position_in_lane < STOP_LINE_POSITION)
                {
                    // Calculate braking distance needed to stop at stop line
                    const double distance_to_stopline = STOP_LINE_POSITION - vehicle.position_in_lane;
                    // v² = 2*a*d → v = sqrt(2*a*d), with comfortable decel of 3 m/s²
                    const double safe_speed = std::sqrt(2.0 * 3.0 * distance_to_stopline);
                    target_speed = std::max(0.0, std::min(target_speed, safe_speed));
                }

                // Check spacing to vehicle ahead
                if (i > 0)
                {
                    const Vehicle &ahead = queue[i - 1];

                    // If vehicle ahead is crossing, wait at stop line
                    if (ahead.isCrossing())
                    {
                        const double distance_to_stopline = STOP_LINE_POSITION - vehicle.position_in_lane;
                        if (distance_to_stopline > 0)
                        {
                            const double safe_speed = std::sqrt(2.0 * 3.0 * std::max(0.1, distance_to_stopline - 2.0));
                            target_speed = std::min(target_speed, safe_speed);
                        }
                        else
                        {
                            target_speed = 0.0;
                        }
                    }
                    else
                    {
                        // Normal following distance
                        double spacing = ahead.position_in_lane - vehicle.position_in_lane;
                        if (spacing < MIN_FRONT_DISTANCE_METERS * 1.5)
                        {
                            target_speed = std::min(target_speed, ahead.current_speed * 0.8);
                        }
                        if (spacing < MIN_FRONT_DISTANCE_METERS)
                        {
                            target_speed = 0.0;
                        }
                    }
                }

                vehicle.updateSpeed(target_speed, dt_seconds);

                if (!vehicle.isCrossing())
                {
                    double previous_position = vehicle.position_in_lane;
                    vehicle.position_in_lane += vehicle.current_speed * dt_seconds;

                    // Hard stop at stop line if light is red
                    if (!lane_can_move[dir] && vehicle.position_in_lane >= STOP_LINE_POSITION)
                    {
                        vehicle.position_in_lane = STOP_LINE_POSITION;
                        vehicle.current_speed = 0.0;
                    }

                    // Maintain safe distance behind vehicle ahead
                    if (i > 0)
                    {
                        const Vehicle &ahead = queue[i - 1];
                        double max_pos;
                        if (ahead.isCrossing())
                        {
                            max_pos = STOP_LINE_POSITION - 2.0;
                        }
                        else
                        {
                            max_pos = ahead.position_in_lane - MIN_FRONT_DISTANCE_METERS;
                        }
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
            states.push_back(state);
        }

        return states;
    }

} // namespace crossroads
