#include "TrafficGenerator.hpp"

namespace crossroads
{

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
                getQueueByDirection(dir).push_back(v);
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

    void TrafficGenerator::updateVehicleSpeeds(double dt_seconds)
    {
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

                if (i > 0)
                {
                    const Vehicle &ahead = queue[i - 1];
                    double spacing = ahead.position_in_lane - vehicle.position_in_lane;

                    if (spacing < VEHICLE_SPACING)
                        target_speed = 0.0; // Blocked by vehicle ahead
                }

                vehicle.updateSpeed(target_speed, dt_seconds);

                if (!vehicle.isCrossing())
                    vehicle.position_in_lane += vehicle.current_speed * dt_seconds;
            }
        }
    }

    double TrafficGenerator::getAverageQueueDensity(Direction dir) const
    {
        return std::min(1.0, static_cast<double>(getQueueByDirection(dir).size()) / LANE_CAPACITY);
    }

} // namespace crossroads
