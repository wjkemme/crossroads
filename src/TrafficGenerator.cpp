#include "TrafficGenerator.hpp"

#include <algorithm>

namespace crossroads
{
    namespace
    {
        constexpr double CAR_LENGTH_METERS = 4.0;
        constexpr double STOPLINE_TARGET_METERS = 69.5;                                      // 0.5m vóór de streep (streep op 70m)
        constexpr double STOPPED_GAP_METERS = 2.0;                                           // 2m bumper-bumper bij stilstand
        constexpr double MIN_FRONT_DISTANCE_METERS = CAR_LENGTH_METERS + STOPPED_GAP_METERS; // 6m front-to-front
        constexpr double FOLLOWING_TIME_SECONDS = 1.5;                                       // 1.5s following bij rijden

        ApproachId approachFromDirection(Direction dir)
        {
            switch (dir)
            {
            case Direction::North:
                return ApproachId::North;
            case Direction::South:
                return ApproachId::South;
            case Direction::East:
                return ApproachId::East;
            case Direction::West:
                return ApproachId::West;
            }
            return ApproachId::North;
        }

        size_t approachArrayIndex(ApproachId approach)
        {
            switch (approach)
            {
            case ApproachId::North:
                return 0;
            case ApproachId::East:
                return 1;
            case ApproachId::South:
                return 2;
            case ApproachId::West:
                return 3;
            }
            return 0;
        }
    }

    const ApproachConfig *TrafficGenerator::getApproachConfig(Direction dir) const
    {
        ApproachId approach = approachFromDirection(dir);
        size_t idx = approachArrayIndex(approach);
        if (idx >= intersection_config.approaches.size())
        {
            return nullptr;
        }
        return &intersection_config.approaches[idx];
    }

    bool TrafficGenerator::laneAllowsMovement(const LaneConfig &lane, MovementType movement) const
    {
        return std::find(lane.allowed_movements.begin(), lane.allowed_movements.end(), movement) != lane.allowed_movements.end();
    }

    const LaneConnectionConfig *TrafficGenerator::findLaneConnection(ApproachId from_approach, uint16_t from_lane_index, MovementType movement) const
    {
        auto it = std::find_if(intersection_config.lane_connections.begin(),
                               intersection_config.lane_connections.end(),
                               [&](const LaneConnectionConfig &connection)
                               {
                                   return connection.from_approach == from_approach &&
                                          connection.from_lane_index == from_lane_index &&
                                          connection.movement == movement;
                               });
        return it == intersection_config.lane_connections.end() ? nullptr : &(*it);
    }

    bool TrafficGenerator::resolveVehicleRoute(Vehicle &vehicle, ApproachId from_approach, uint16_t from_lane_index, MovementType movement) const
    {
        vehicle.movement = movement;
        vehicle.turning = (movement != MovementType::Straight);

        bool used_explicit_connection = false;
        ApproachId destination_approach = destinationApproachFor(from_approach, movement);
        uint16_t destination_lane_index = from_lane_index;

        if (const auto *connection = findLaneConnection(from_approach, from_lane_index, movement))
        {
            destination_approach = connection->to_approach;
            destination_lane_index = connection->to_lane_index;
            used_explicit_connection = true;
        }

        vehicle.destination_approach = destination_approach;
        vehicle.destination_lane_index = destination_lane_index;

        size_t destination_approach_index = approachArrayIndex(destination_approach);
        if (destination_approach_index >= intersection_config.approaches.size())
        {
            vehicle.destination_lane_id = 0;
            return used_explicit_connection;
        }

        const auto &destination_approach_cfg = intersection_config.approaches[destination_approach_index];
        const size_t destination_lane_count = effectiveToLaneCount(destination_approach_cfg);
        if (destination_lane_count == 0)
        {
            vehicle.destination_lane_id = 0;
            return used_explicit_connection;
        }

        size_t clamped_lane_index = std::min<size_t>(destination_lane_index, destination_lane_count - 1);
        vehicle.destination_lane_index = static_cast<uint16_t>(clamped_lane_index);
        vehicle.destination_lane_id = laneIdFor(destination_approach, clamped_lane_index);
        return used_explicit_connection;
    }

    size_t TrafficGenerator::chooseSpawnMovementIndex(const std::vector<MovementType> &movements, uint32_t vehicle_id) const
    {
        if (movements.empty())
        {
            return 0;
        }

        size_t straight_idx = movements.size();
        size_t right_idx = movements.size();
        size_t left_idx = movements.size();
        for (size_t i = 0; i < movements.size(); ++i)
        {
            if (movements[i] == MovementType::Straight && straight_idx == movements.size())
                straight_idx = i;
            if (movements[i] == MovementType::Right && right_idx == movements.size())
                right_idx = i;
            if (movements[i] == MovementType::Left && left_idx == movements.size())
                left_idx = i;
        }

        size_t roll = static_cast<size_t>(vehicle_id % 10);
        if (roll < 6 && straight_idx < movements.size())
            return straight_idx; // 60% straight preference
        if (roll < 8 && right_idx < movements.size())
            return right_idx; // 20% right
        if (left_idx < movements.size())
            return left_idx; // 20% left

        if (straight_idx < movements.size())
            return straight_idx;
        if (right_idx < movements.size())
            return right_idx;
        return 0;
    }

    size_t TrafficGenerator::choosePreferredLaneIndex(const ApproachConfig &approach, MovementType movement, size_t current_index) const
    {
        std::vector<size_t> candidates;
        for (size_t idx = 0; idx < approach.lanes.size(); ++idx)
        {
            if (!approach.lanes[idx].connected_to_intersection)
            {
                continue;
            }

            if (laneAllowsMovement(approach.lanes[idx], movement))
            {
                candidates.push_back(idx);
            }
        }

        if (candidates.empty())
        {
            return current_index;
        }

        if (movement == MovementType::Right)
        {
            return *std::max_element(candidates.begin(), candidates.end());
        }
        if (movement == MovementType::Left)
        {
            return *std::min_element(candidates.begin(), candidates.end());
        }

        size_t best = candidates.front();
        size_t best_dist = (best > current_index) ? (best - current_index) : (current_index - best);
        for (size_t idx : candidates)
        {
            size_t dist = (idx > current_index) ? (idx - current_index) : (current_index - idx);
            if (dist < best_dist)
            {
                best = idx;
                best_dist = dist;
            }
        }
        return best;
    }

    bool TrafficGenerator::hasSafeGapForLaneChange(const std::deque<Vehicle> &queue, size_t vehicle_index, LaneId target_lane_id) const
    {
        const Vehicle &vehicle = queue[vehicle_index];
        for (size_t j = 0; j < queue.size(); ++j)
        {
            if (j == vehicle_index)
            {
                continue;
            }
            const Vehicle &other = queue[j];
            if (other.isCrossing() || other.lane_id != target_lane_id)
            {
                continue;
            }

            double distance = std::abs(other.position_in_lane - vehicle.position_in_lane);
            if (distance < MIN_FRONT_DISTANCE_METERS)
            {
                return false;
            }
        }
        return true;
    }

    void TrafficGenerator::maybeApplyLaneChanges(Direction dir, std::deque<Vehicle> &queue)
    {
        if (!use_configured_spawns || queue.empty())
        {
            return;
        }

        const ApproachConfig *approach = getApproachConfig(dir);
        if (!approach || approach->lanes.empty())
        {
            return;
        }

        for (size_t i = 0; i < queue.size(); ++i)
        {
            Vehicle &vehicle = queue[i];
            if (vehicle.isCrossing())
            {
                continue;
            }

            auto current_it = std::find_if(approach->lanes.begin(), approach->lanes.end(),
                                           [&](const LaneConfig &lane)
                                           { return lane.id == vehicle.lane_id; });
            if (current_it == approach->lanes.end())
            {
                continue;
            }

            size_t current_index = static_cast<size_t>(std::distance(approach->lanes.begin(), current_it));
            bool movement_allowed = laneAllowsMovement(*current_it, vehicle.movement);

            if (movement_allowed)
            {
                resolveVehicleRoute(vehicle,
                                    approach->id,
                                    static_cast<uint16_t>(current_index),
                                    vehicle.movement);
                continue;
            }

            if (!vehicle.lane_change_allowed)
            {
                resolveVehicleRoute(vehicle,
                                    approach->id,
                                    static_cast<uint16_t>(current_index),
                                    MovementType::Straight);
                continue;
            }

            if (vehicle.position_in_lane > 55.0)
            {
                resolveVehicleRoute(vehicle,
                                    approach->id,
                                    static_cast<uint16_t>(current_index),
                                    MovementType::Straight);
                continue;
            }

            size_t target_index = choosePreferredLaneIndex(*approach, vehicle.movement, current_index);
            if (target_index >= approach->lanes.size())
            {
                continue;
            }

            LaneId target_lane = approach->lanes[target_index].id;
            if (target_lane == vehicle.lane_id)
            {
                resolveVehicleRoute(vehicle,
                                    approach->id,
                                    static_cast<uint16_t>(current_index),
                                    vehicle.movement);
                continue;
            }

            if (hasSafeGapForLaneChange(queue, i, target_lane))
            {
                vehicle.lane_id = target_lane;
                vehicle.queue_index = static_cast<uint8_t>(target_index % 3);
                vehicle.lane_change_allowed = approach->lanes[target_index].supports_lane_change;
                resolveVehicleRoute(vehicle,
                                    approach->id,
                                    static_cast<uint16_t>(target_index),
                                    vehicle.movement);
            }
        }
    }

    TrafficGenerator::TrafficGenerator(double rate)
        : intersection_config(makeDefaultIntersectionConfig()),
          use_configured_spawns(false),
          arrival_rate(rate), time_accumulated(0.0), next_vehicle_id(1)
    {
    }

    TrafficGenerator::TrafficGenerator(const IntersectionConfig &config, double rate)
        : intersection_config(config),
          use_configured_spawns(true),
          arrival_rate(rate), time_accumulated(0.0), next_vehicle_id(1)
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

                if (use_configured_spawns)
                {
                    ApproachId approach = approachFromDirection(dir);
                    size_t approach_idx = approachArrayIndex(approach);
                    const auto &approach_cfg = intersection_config.approaches[approach_idx];

                    if (!approach_cfg.lanes.empty())
                    {
                        std::vector<size_t> connected_lane_indices;
                        connected_lane_indices.reserve(approach_cfg.lanes.size());
                        for (size_t lane_idx = 0; lane_idx < approach_cfg.lanes.size(); ++lane_idx)
                        {
                            if (approach_cfg.lanes[lane_idx].connected_to_intersection)
                            {
                                connected_lane_indices.push_back(lane_idx);
                            }
                        }

                        if (connected_lane_indices.empty())
                        {
                            continue;
                        }

                        size_t cursor_slot = spawn_lane_cursor[approach_idx] % connected_lane_indices.size();
                        size_t cursor = connected_lane_indices[cursor_slot];
                        const auto &lane_cfg = approach_cfg.lanes[cursor];
                        spawn_lane_cursor[approach_idx] = (cursor_slot + 1) % connected_lane_indices.size();

                        std::vector<MovementType> available_movements;
                        for (size_t lane_idx : connected_lane_indices)
                        {
                            const auto &lane = approach_cfg.lanes[lane_idx];
                            for (MovementType movement : lane.allowed_movements)
                            {
                                if (std::find(available_movements.begin(), available_movements.end(), movement) == available_movements.end())
                                {
                                    available_movements.push_back(movement);
                                }
                            }
                        }

                        if (!available_movements.empty())
                        {
                            size_t movement_idx = chooseSpawnMovementIndex(available_movements, v.id);
                            v.movement = available_movements[movement_idx];
                        }
                        else
                        {
                            v.movement = MovementType::Straight;
                        }

                        size_t preferred_lane_idx = choosePreferredLaneIndex(approach_cfg, v.movement, cursor);
                        const auto &preferred_lane_cfg = approach_cfg.lanes[preferred_lane_idx];

                        v.queue_index = static_cast<uint8_t>(preferred_lane_idx % 3);
                        v.lane_id = preferred_lane_cfg.id;
                        v.lane_change_allowed = preferred_lane_cfg.supports_lane_change;

                        if (!resolveVehicleRoute(v,
                                                 approach,
                                                 static_cast<uint16_t>(preferred_lane_idx),
                                                 v.movement))
                        {
                            if (!preferred_lane_cfg.allowed_movements.empty())
                            {
                                resolveVehicleRoute(v,
                                                    approach,
                                                    static_cast<uint16_t>(preferred_lane_idx),
                                                    preferred_lane_cfg.allowed_movements.front());
                            }
                            else
                            {
                                resolveVehicleRoute(v,
                                                    approach,
                                                    static_cast<uint16_t>(preferred_lane_idx),
                                                    MovementType::Straight);
                            }
                        }
                    }
                    else
                    {
                        use_configured_spawns = false;
                    }
                }

                if (!use_configured_spawns)
                {
                    v.turning = (v.id % 5 == 0);

                    if (v.turning)
                    {
                        v.queue_index = 2;
                        v.lane_id = static_cast<LaneId>(static_cast<int>(dir) * 100 + 2);
                        v.movement = MovementType::Right;
                    }
                    else
                    {
                        auto &queue = getQueueByDirection(dir);
                        size_t straight_count = 0;
                        for (const auto &veh : queue)
                        {
                            if (!veh.turning)
                                straight_count++;
                        }
                        v.queue_index = straight_count % 2;
                        v.lane_id = static_cast<LaneId>(static_cast<int>(dir) * 100 + v.queue_index);
                        v.movement = MovementType::Straight;
                    }

                    ApproachId from_approach = approachFromDirection(dir);
                    v.destination_approach = destinationApproachFor(from_approach, v.movement);
                    v.destination_lane_index = v.queue_index;
                    v.destination_lane_id = laneIdFor(v.destination_approach, v.destination_lane_index);
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
        const double STOP_TARGET = STOPLINE_TARGET_METERS; // 0.5m voor de streep
        const double MAX_SPEED = 10.0;                     // m/s
        const double BRAKE_DECEL = 4.5;                    // m/s^2

        for (int dir = 0; dir < 4; ++dir)
        {
            Direction d = static_cast<Direction>(dir);
            auto &queue = getQueueByDirection(d);
            bool can_move = lane_can_move[dir];

            maybeApplyLaneChanges(d, queue);

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
                    if (!ahead.isCrossing() && ahead.lane_id == vehicle.lane_id)
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
            state.lane_id = vehicle.lane_id;
            state.movement = vehicle.movement;
            state.destination_approach = vehicle.destination_approach;
            state.destination_lane_index = vehicle.destination_lane_index;
            state.destination_lane_id = vehicle.destination_lane_id;
            state.lane_change_allowed = vehicle.lane_change_allowed;
            states.push_back(state);
        }

        return states;
    }

} // namespace crossroads
