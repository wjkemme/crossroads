#include "SimulatorEngine.hpp"
#include <array>
#include <algorithm>
#include <unordered_set>
#include <sstream>
#include <iostream>
#include <utility>

namespace crossroads
{
    namespace
    {
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

        const LaneConfig *findLaneConfigForVehicle(const IntersectionConfig &config, Direction dir, LaneId lane_id)
        {
            ApproachId approach = approachFromDirection(dir);
            auto approach_it = std::find_if(config.approaches.begin(), config.approaches.end(),
                                            [approach](const ApproachConfig &entry)
                                            { return entry.id == approach; });
            if (approach_it == config.approaches.end())
            {
                return nullptr;
            }

            auto lane_it = std::find_if(approach_it->lanes.begin(), approach_it->lanes.end(),
                                        [lane_id](const LaneConfig &lane)
                                        { return lane.id == lane_id; });
            if (lane_it == approach_it->lanes.end())
            {
                return nullptr;
            }
            return &(*lane_it);
        }

        double estimatedCrossingPathMeters(MovementType movement)
        {
            switch (movement)
            {
            case MovementType::Right:
                return 10.0;
            case MovementType::Left:
                return 20.0;
            case MovementType::Straight:
            default:
                return 18.0;
            }
        }

        ApproachId leftDestinationFor(Direction lane)
        {
            switch (lane)
            {
            case Direction::North:
                return ApproachId::East;
            case Direction::South:
                return ApproachId::West;
            case Direction::East:
                return ApproachId::South;
            case Direction::West:
                return ApproachId::North;
            }
            return ApproachId::North;
        }

        bool isEffectiveLeftTurn(Direction lane, const Vehicle &vehicle)
        {
            if (vehicle.movement == MovementType::Left)
            {
                return true;
            }
            return vehicle.destination_approach == leftDestinationFor(lane);
        }

        bool protectedLeftAllowed(Direction lane, const IntersectionState &state)
        {
            switch (lane)
            {
            case Direction::North:
                return state.south == LightState::Red;
            case Direction::South:
                return state.north == LightState::Red;
            case Direction::East:
                return state.west == LightState::Red;
            case Direction::West:
                return state.east == LightState::Red;
            }
            return false;
        }

        Direction directionFromApproach(ApproachId approach)
        {
            switch (approach)
            {
            case ApproachId::North:
                return Direction::North;
            case ApproachId::South:
                return Direction::South;
            case ApproachId::East:
                return Direction::East;
            case ApproachId::West:
                return Direction::West;
            }
            return Direction::North;
        }

        bool isDedicatedRightTurnLane(const LaneConfig &lane_cfg)
        {
            return lane_cfg.connected_to_intersection &&
                   lane_cfg.has_traffic_light &&
                   lane_cfg.allowed_movements.size() == 1 &&
                   lane_cfg.allowed_movements.front() == MovementType::Right;
        }

        bool isDedicatedLeftTurnLane(const LaneConfig &lane_cfg)
        {
            return lane_cfg.connected_to_intersection &&
                   lane_cfg.has_traffic_light &&
                   lane_cfg.allowed_movements.size() == 1 &&
                   lane_cfg.allowed_movements.front() == MovementType::Left;
        }

        bool isApproachMainRed(ApproachId approach, const IntersectionState &state)
        {
            switch (approach)
            {
            case ApproachId::North:
                return state.north == LightState::Red;
            case ApproachId::South:
                return state.south == LightState::Red;
            case ApproachId::East:
                return state.east == LightState::Red;
            case ApproachId::West:
                return state.west == LightState::Red;
            }
            return true;
        }

        LightState approachMainLight(ApproachId approach, const IntersectionState &state)
        {
            switch (approach)
            {
            case ApproachId::North:
                return state.north;
            case ApproachId::South:
                return state.south;
            case ApproachId::East:
                return state.east;
            case ApproachId::West:
                return state.west;
            }
            return LightState::Red;
        }

        bool hasExclusiveLaneConnection(const IntersectionConfig &config,
                                        ApproachId from_approach,
                                        LaneId from_lane_id,
                                        MovementType movement)
        {
            uint16_t from_lane_index = static_cast<uint16_t>(from_lane_id % 100);
            auto approach_it = std::find_if(config.approaches.begin(), config.approaches.end(),
                                            [&](const ApproachConfig &approach)
                                            { return approach.id == from_approach; });
            if (approach_it != config.approaches.end())
            {
                auto lane_it = std::find_if(approach_it->lanes.begin(), approach_it->lanes.end(),
                                            [&](const LaneConfig &lane)
                                            { return lane.id == from_lane_id; });
                if (lane_it != approach_it->lanes.end())
                {
                    from_lane_index = static_cast<uint16_t>(std::distance(approach_it->lanes.begin(), lane_it));
                }
            }

            auto self_connection_it = std::find_if(config.lane_connections.begin(), config.lane_connections.end(),
                                                   [&](const LaneConnectionConfig &connection)
                                                   {
                                                       return connection.from_approach == from_approach &&
                                                              connection.from_lane_index == from_lane_index &&
                                                              connection.movement == movement;
                                                   });
            if (self_connection_it == config.lane_connections.end())
            {
                return false;
            }

            const ApproachId to_approach = self_connection_it->to_approach;
            const uint16_t to_lane_index = self_connection_it->to_lane_index;
            const size_t incoming_count = static_cast<size_t>(std::count_if(config.lane_connections.begin(), config.lane_connections.end(),
                                                                            [&](const LaneConnectionConfig &connection)
                                                                            {
                                                                                return connection.to_approach == to_approach &&
                                                                                       connection.to_lane_index == to_lane_index;
                                                                            }));

            return incoming_count == 1;
        }

        void setRightTurnLight(IntersectionState &state, ApproachId approach, LightState color)
        {
            switch (approach)
            {
            case ApproachId::North:
                state.turnNorthWest = color;
                break;
            case ApproachId::South:
                state.turnSouthEast = color;
                break;
            case ApproachId::East:
                state.turnEastNorth = color;
                break;
            case ApproachId::West:
                state.turnWestSouth = color;
                break;
            }
        }

        void setApproachMainLight(IntersectionState &state, ApproachId approach, LightState color)
        {
            switch (approach)
            {
            case ApproachId::North:
                state.north = color;
                break;
            case ApproachId::South:
                state.south = color;
                break;
            case ApproachId::East:
                state.east = color;
                break;
            case ApproachId::West:
                state.west = color;
                break;
            }
        }

        void setLeftTurnLight(IntersectionState &state, ApproachId approach, LightState color)
        {
            switch (approach)
            {
            case ApproachId::North:
                state.turnNorthEast = color;
                break;
            case ApproachId::South:
                state.turnSouthWest = color;
                break;
            case ApproachId::East:
                state.turnEastSouth = color;
                break;
            case ApproachId::West:
                state.turnWestNorth = color;
                break;
            }
        }

        LightState rightTurnLightFor(Direction dir, const IntersectionState &state)
        {
            switch (dir)
            {
            case Direction::North:
                return state.turnNorthWest;
            case Direction::South:
                return state.turnSouthEast;
            case Direction::East:
                return state.turnEastNorth;
            case Direction::West:
                return state.turnWestSouth;
            }
            return LightState::Red;
        }

        LightState leftTurnLightFor(Direction dir, const IntersectionState &state)
        {
            switch (dir)
            {
            case Direction::North:
                return state.turnNorthEast;
            case Direction::South:
                return state.turnSouthWest;
            case Direction::East:
                return state.turnEastSouth;
            case Direction::West:
                return state.turnWestNorth;
            }
            return LightState::Red;
        }

        ApproachId opposingApproachForRightTurn(ApproachId approach)
        {
            switch (approach)
            {
            case ApproachId::North:
                return ApproachId::East;
            case ApproachId::South:
                return ApproachId::West;
            case ApproachId::East:
                return ApproachId::South;
            case ApproachId::West:
                return ApproachId::North;
            }
            return ApproachId::North;
        }

        ApproachId opposingApproachForLeftTurn(ApproachId approach)
        {
            switch (approach)
            {
            case ApproachId::North:
                return ApproachId::South;
            case ApproachId::South:
                return ApproachId::North;
            case ApproachId::East:
                return ApproachId::West;
            case ApproachId::West:
                return ApproachId::East;
            }
            return ApproachId::North;
        }
    }

    SimulatorEngine::SimulatorEngine(double traffic_rate, double ns_duration, double ew_duration)
        : SimulatorEngine(makeDefaultIntersectionConfig(), traffic_rate, ns_duration, ew_duration)
    {
    }

    SimulatorEngine::SimulatorEngine(const IntersectionConfig &intersection_config,
                                     double traffic_rate,
                                     double ns_duration,
                                     double ew_duration)
        : checker(intersection_config),
          control_mode(ControlMode::Basic),
          traffic(intersection_config, traffic_rate),
          ns_duration(ns_duration),
          ew_duration(ew_duration),
          intersection_config(intersection_config),
          current_time(0.0),
          safety_violations(0)
    {
        if (this->intersection_config.signal_groups.empty())
        {
            controller = std::make_unique<BasicControllerAdapter>(ns_duration, ew_duration);
        }
        else
        {
            controller = std::make_unique<ConfigurableSignalGroupController>(this->intersection_config);
        }

        refreshEffectiveSignalState();
    }

    void SimulatorEngine::simulate(double duration_seconds, double time_step)
    {
        reset();
        start();
        while (current_time < duration_seconds)
            tick(time_step);
        stop();
    }

    void SimulatorEngine::tick(double dt)
    {
        if (!running)
        {
            return;
        }

        std::array<bool, 4> approach_demand = {
            !traffic.getQueueByDirection(Direction::North).empty(),
            !traffic.getQueueByDirection(Direction::South).empty(),
            !traffic.getQueueByDirection(Direction::East).empty(),
            !traffic.getQueueByDirection(Direction::West).empty()};
        if (controller)
        {
            controller->setDemandByDirection(approach_demand);
        }

        advanceController(dt);

        generateTraffic(dt);
        refreshEffectiveSignalState();
        std::array<bool, 4> lane_can_move = {
            effective_light_state.north == LightState::Green,
            effective_light_state.south == LightState::Green,
            effective_light_state.east == LightState::Green,
            effective_light_state.west == LightState::Green};
        traffic.updateVehicleSpeeds(dt,
                                    lane_can_move,
                                    [&](Direction dir, const Vehicle &vehicle)
                                    {
                                        const LaneConfig *lane_cfg = findLaneConfigForVehicle(intersection_config, dir, vehicle.lane_id);
                                        if (lane_cfg && !lane_cfg->connected_to_intersection)
                                        {
                                            return false;
                                        }
                                        return signalAllowsVehicle(dir, vehicle, lane_cfg, effective_light_state);
                                    });
        processVehicleCrossings();
        completeVehicleCrossings();

        IntersectionState current_state = controller ? controller->getCurrentState() : IntersectionState{};
        if (!checker.isSafe(current_state) || !isConfigSignalStateSafe(current_state))
        {
            safety_violations++;
            if (control_mode != ControlMode::NullControl)
            {
                setControlMode(ControlMode::NullControl);
            }
        }

        current_time += dt;
    }

    void SimulatorEngine::refreshEffectiveSignalState()
    {
        IntersectionState base_state = controller ? controller->getCurrentState() : IntersectionState{};

        if (traffic.getTotalWaiting() == 0)
        {
            effective_light_state = IntersectionState{};
            return;
        }

        effective_light_state = base_state;

        for (const auto &approach_cfg : intersection_config.approaches)
        {
            const Direction dir = directionFromApproach(approach_cfg.id);
            const auto &queue = traffic.getQueueByDirection(dir);
            if (queue.empty())
            {
                setApproachMainLight(effective_light_state, approach_cfg.id, LightState::Red);
            }
        }

        for (const auto &approach_cfg : intersection_config.approaches)
        {
            const ApproachId approach = approach_cfg.id;
            const Direction dir = directionFromApproach(approach);
            const auto &queue = traffic.getQueueByDirection(dir);

            std::vector<LaneId> dedicated_right_lane_ids;
            std::vector<LaneId> exclusive_right_lane_ids;
            dedicated_right_lane_ids.reserve(approach_cfg.lanes.size());
            exclusive_right_lane_ids.reserve(approach_cfg.lanes.size());
            for (const auto &lane_cfg : approach_cfg.lanes)
            {
                if (isDedicatedRightTurnLane(lane_cfg))
                {
                    dedicated_right_lane_ids.push_back(lane_cfg.id);
                    if (hasExclusiveLaneConnection(intersection_config, approach, lane_cfg.id, MovementType::Right))
                    {
                        exclusive_right_lane_ids.push_back(lane_cfg.id);
                    }
                }
            }

            if (dedicated_right_lane_ids.empty())
            {
                setRightTurnLight(effective_light_state, approach, LightState::Red);
                continue;
            }

            const bool has_demand = std::any_of(queue.begin(), queue.end(),
                                                [&](const Vehicle &vehicle)
                                                {
                                                    if (std::find(dedicated_right_lane_ids.begin(), dedicated_right_lane_ids.end(), vehicle.lane_id) == dedicated_right_lane_ids.end())
                                                    {
                                                        return false;
                                                    }
                                                    return vehicle.isWaiting() || vehicle.isCrossing();
                                                });

            const bool has_non_right_or_mixed_lane_demand = std::any_of(queue.begin(), queue.end(),
                                                                        [&](const Vehicle &vehicle)
                                                                        {
                                                                            if (!vehicle.isWaiting() && !vehicle.isCrossing())
                                                                            {
                                                                                return false;
                                                                            }
                                                                            if (vehicle.movement != MovementType::Right)
                                                                            {
                                                                                return true;
                                                                            }
                                                                            return std::find(dedicated_right_lane_ids.begin(), dedicated_right_lane_ids.end(), vehicle.lane_id) == dedicated_right_lane_ids.end();
                                                                        });
            if (!has_non_right_or_mixed_lane_demand)
            {
                setApproachMainLight(effective_light_state, approach, LightState::Red);
            }

            const bool has_exclusive_demand = std::any_of(queue.begin(), queue.end(),
                                                          [&](const Vehicle &vehicle)
                                                          {
                                                              if (std::find(exclusive_right_lane_ids.begin(), exclusive_right_lane_ids.end(), vehicle.lane_id) == exclusive_right_lane_ids.end())
                                                              {
                                                                  return false;
                                                              }
                                                              return vehicle.isWaiting() || vehicle.isCrossing();
                                                          });

            const ApproachId opposing = opposingApproachForRightTurn(approach);
            const bool opposing_red = isApproachMainRed(opposing, effective_light_state);
            const bool opposing_not_green = approachMainLight(opposing, effective_light_state) != LightState::Green;
            const std::size_t approach_idx = approachIndex(approach);

            if (has_demand && opposing_red)
            {
                right_turn_green_hold_until[approach_idx] = std::max(right_turn_green_hold_until[approach_idx],
                                                                     current_time + right_turn_min_green_seconds);
            }

            const bool hold_active = right_turn_green_hold_until[approach_idx] > current_time;
            const bool turn_green = has_exclusive_demand ? (has_demand || hold_active)
                                                         : (opposing_red && (has_demand || hold_active));
            setRightTurnLight(effective_light_state, approach, turn_green ? LightState::Green : LightState::Red);
        }

        for (const auto &approach_cfg : intersection_config.approaches)
        {
            const ApproachId approach = approach_cfg.id;
            const Direction dir = directionFromApproach(approach);
            const auto &queue = traffic.getQueueByDirection(dir);

            std::vector<LaneId> dedicated_left_lane_ids;
            dedicated_left_lane_ids.reserve(approach_cfg.lanes.size());
            for (const auto &lane_cfg : approach_cfg.lanes)
            {
                if (isDedicatedLeftTurnLane(lane_cfg))
                {
                    dedicated_left_lane_ids.push_back(lane_cfg.id);
                }
            }

            if (dedicated_left_lane_ids.empty())
            {
                setLeftTurnLight(effective_light_state, approach, LightState::Red);
                continue;
            }

            const bool has_demand = std::any_of(queue.begin(), queue.end(),
                                                [&](const Vehicle &vehicle)
                                                {
                                                    if (std::find(dedicated_left_lane_ids.begin(), dedicated_left_lane_ids.end(), vehicle.lane_id) == dedicated_left_lane_ids.end())
                                                    {
                                                        return false;
                                                    }
                                                    return vehicle.isWaiting() || vehicle.isCrossing();
                                                });

            const ApproachId opposing = opposingApproachForLeftTurn(approach);
            const bool opposing_red = isApproachMainRed(opposing, effective_light_state);
            const bool opposing_right_red = rightTurnLightFor(directionFromApproach(opposing), effective_light_state) == LightState::Red;
            bool cross_traffic_main_red = true;
            if (approach == ApproachId::North || approach == ApproachId::South)
            {
                cross_traffic_main_red = (effective_light_state.east == LightState::Red && effective_light_state.west == LightState::Red);
            }
            else
            {
                cross_traffic_main_red = (effective_light_state.north == LightState::Red && effective_light_state.south == LightState::Red);
            }

            const bool left_green = has_demand && opposing_red && opposing_right_red && cross_traffic_main_red;
            setLeftTurnLight(effective_light_state, approach, left_green ? LightState::Green : LightState::Red);

            const bool has_dedicated_left_crossing = std::any_of(queue.begin(), queue.end(),
                                                                 [&](const Vehicle &vehicle)
                                                                 {
                                                                     if (!vehicle.isCrossing())
                                                                     {
                                                                         return false;
                                                                     }
                                                                     if (std::find(dedicated_left_lane_ids.begin(), dedicated_left_lane_ids.end(), vehicle.lane_id) == dedicated_left_lane_ids.end())
                                                                     {
                                                                         return false;
                                                                     }
                                                                     return isEffectiveLeftTurn(dir, vehicle);
                                                                 });
            if (has_dedicated_left_crossing)
            {
                setApproachMainLight(effective_light_state, opposing, LightState::Red);
            }

            const bool unprotected_mixed_left = std::any_of(queue.begin(), queue.end(),
                                                            [&](const Vehicle &vehicle)
                                                            {
                                                                if (!vehicle.isWaiting() && !vehicle.isCrossing())
                                                                {
                                                                    return false;
                                                                }
                                                                if (!isEffectiveLeftTurn(dir, vehicle))
                                                                {
                                                                    return false;
                                                                }
                                                                return std::find(dedicated_left_lane_ids.begin(), dedicated_left_lane_ids.end(), vehicle.lane_id) == dedicated_left_lane_ids.end();
                                                            });

            if (unprotected_mixed_left && !(opposing_red && opposing_right_red))
            {
                setApproachMainLight(effective_light_state, approach, LightState::Red);
            }

            const bool has_any_left_demand = std::any_of(queue.begin(), queue.end(),
                                                         [&](const Vehicle &vehicle)
                                                         {
                                                             if (!vehicle.isWaiting() && !vehicle.isCrossing())
                                                             {
                                                                 return false;
                                                             }
                                                             return isEffectiveLeftTurn(dir, vehicle);
                                                         });
            if (has_any_left_demand)
            {
                const LightState approach_main = approachMainLight(approach, effective_light_state);
                if (approach_main == LightState::Green)
                {
                    setApproachMainLight(effective_light_state, opposing, LightState::Red);
                }
            }
        }
    }

    void SimulatorEngine::generateTraffic(double dt)
    {
        traffic.generateTraffic(dt, current_time);
    }

    void SimulatorEngine::processVehicleCrossings()
    {
        // Start crossing when vehicle has reached stopzone.
        // Multiple vehicles in the same lane are allowed, but only with safe headway.
        const double STOP_TARGET = 69.5;
        const double MIN_SAME_LANE_HEADWAY_SECONDS = 0.7;

        for (int dir = 0; dir < 4; ++dir)
        {
            Direction lane = static_cast<Direction>(dir);
            auto &queue = traffic.getQueueByDirection(lane);
            for (size_t i = 0; i < queue.size(); ++i)
            {
                auto &vehicle = queue[i];
                if (!vehicle.isWaiting() || vehicle.position_in_lane < STOP_TARGET)
                {
                    continue;
                }

                bool blocked_by_same_lane_front = false;
                for (size_t j = 0; j < i; ++j)
                {
                    const auto &ahead = queue[j];
                    if (ahead.lane_id != vehicle.lane_id)
                    {
                        continue;
                    }

                    if (ahead.isWaiting())
                    {
                        blocked_by_same_lane_front = true;
                        break;
                    }

                    if (ahead.isCrossing())
                    {
                        const double elapsed = std::max(0.0, current_time - ahead.crossing_time);
                        if (elapsed < MIN_SAME_LANE_HEADWAY_SECONDS)
                        {
                            blocked_by_same_lane_front = true;
                            break;
                        }
                    }
                }
                if (blocked_by_same_lane_front)
                {
                    continue;
                }

                const LaneConfig *lane_cfg = findLaneConfigForVehicle(intersection_config, lane, vehicle.lane_id);
                bool connected = lane_cfg ? lane_cfg->connected_to_intersection : true;
                bool has_traffic_light = lane_cfg ? lane_cfg->has_traffic_light : true;

                if (!connected)
                {
                    continue;
                }

                bool can_cross = signalAllowsVehicle(lane, vehicle, lane_cfg, effective_light_state);
                if (can_cross)
                {
                    vehicle.crossing_time = current_time;
                }
            }
        }
    }

    void SimulatorEngine::completeVehicleCrossings()
    {
        // Complete all vehicles that have finished crossing, regardless of queue position.
        for (int dir = 0; dir < 4; ++dir)
        {
            Direction lane = static_cast<Direction>(dir);
            const auto &queue = traffic.getQueueByDirection(lane);
            std::vector<uint32_t> finished_ids;
            finished_ids.reserve(queue.size());

            for (const auto &vehicle : queue)
            {
                if (!vehicle.isCrossing())
                {
                    continue;
                }

                const double crossing_time = vehicle.getCrossingDuration(queue.size());
                if (current_time - vehicle.crossing_time >= crossing_time)
                {
                    finished_ids.push_back(vehicle.id);
                }
            }

            for (uint32_t id : finished_ids)
            {
                traffic.completeCrossing(id, current_time);
            }
        }
    }

    void SimulatorEngine::advanceController(double dt)
    {
        controller->tick(dt);
    }

    IntersectionState SimulatorEngine::getCurrentLightState() const
    {
        return controller ? controller->getCurrentState() : IntersectionState{};
    }

    SimulatorMetrics SimulatorEngine::getMetrics() const
    {
        SimulatorMetrics metrics;
        metrics.total_time = current_time;
        metrics.vehicles_generated = traffic.getTotalGenerated();
        metrics.vehicles_crossed = traffic.getTotalCrossed();
        metrics.average_wait_time = traffic.getAverageWaitTime();
        metrics.queue_lengths = {
            traffic.getQueueLength(Direction::North),
            traffic.getQueueLength(Direction::East),
            traffic.getQueueLength(Direction::South),
            traffic.getQueueLength(Direction::West)};
        metrics.total_queue_length =
            metrics.queue_lengths[0] + metrics.queue_lengths[1] +
            metrics.queue_lengths[2] + metrics.queue_lengths[3];
        metrics.safety_violations = safety_violations;
        return metrics;
    }

    SimulatorSnapshot SimulatorEngine::getSnapshot() const
    {
        SimulatorSnapshot snapshot;
        snapshot.sim_time = current_time;
        snapshot.running = running;
        snapshot.metrics = getMetrics();
        snapshot.lights = effective_light_state;
        return snapshot;
    }

    namespace
    {
        const char *toString(LightState state)
        {
            switch (state)
            {
            case LightState::Red:
                return "red";
            case LightState::Orange:
                return "orange";
            case LightState::Green:
                return "green";
            }
            return "red";
        }

        const char *toString(MovementType movement)
        {
            switch (movement)
            {
            case MovementType::Straight:
                return "straight";
            case MovementType::Left:
                return "left";
            case MovementType::Right:
                return "right";
            }
            return "straight";
        }

        const char *toString(ApproachId approach)
        {
            switch (approach)
            {
            case ApproachId::North:
                return "north";
            case ApproachId::East:
                return "east";
            case ApproachId::South:
                return "south";
            case ApproachId::West:
                return "west";
            }
            return "north";
        }
    }

    std::string SimulatorEngine::getSnapshotJson() const
    {
        auto northVehicles = traffic.getLaneVehicleStates(Direction::North);
        auto eastVehicles = traffic.getLaneVehicleStates(Direction::East);
        auto southVehicles = traffic.getLaneVehicleStates(Direction::South);
        auto westVehicles = traffic.getLaneVehicleStates(Direction::West);

        auto appendLaneVehicles = [](std::ostringstream &out, const std::vector<LaneVehicleState> &vehicles)
        {
            out << "[";
            for (size_t i = 0; i < vehicles.size(); ++i)
            {
                const auto &v = vehicles[i];
                if (i > 0)
                {
                    out << ",";
                }
                out << "{";
                out << "\"id\":" << v.id << ",";
                out << "\"position\":" << v.position_in_lane << ",";
                out << "\"speed\":" << v.speed << ",";
                out << "\"crossing\":" << (v.crossing ? "true" : "false") << ",";
                out << "\"turning\":" << (v.turning ? "true" : "false") << ",";
                out << "\"crossing_time\":" << v.crossing_time << ",";
                out << "\"crossing_duration\":" << v.crossing_duration << ",";
                out << "\"queue_index\":" << static_cast<int>(v.queue_index) << ",";
                out << "\"lane_id\":" << v.lane_id << ",";
                out << "\"movement\":\"" << toString(v.movement) << "\",";
                out << "\"destination_approach\":\"" << toString(v.destination_approach) << "\",";
                out << "\"destination_lane_index\":" << v.destination_lane_index << ",";
                out << "\"destination_lane_id\":" << v.destination_lane_id << ",";
                out << "\"lane_change_allowed\":" << (v.lane_change_allowed ? "true" : "false");
                out << "}";
            }
            out << "]";
        };

        SimulatorSnapshot snapshot = getSnapshot();
        std::ostringstream out;
        out << "{";
        out << "\"sim_time\":" << snapshot.sim_time << ",";
        out << "\"running\":" << (snapshot.running ? "true" : "false") << ",";
        out << "\"metrics\":{";
        out << "\"vehicles_generated\":" << snapshot.metrics.vehicles_generated << ",";
        out << "\"vehicles_crossed\":" << snapshot.metrics.vehicles_crossed << ",";
        out << "\"average_wait_time\":" << snapshot.metrics.average_wait_time << ",";
        out << "\"safety_violations\":" << snapshot.metrics.safety_violations << ",";
        out << "\"queues\":{";
        out << "\"north\":" << snapshot.metrics.queue_lengths[0] << ",";
        out << "\"east\":" << snapshot.metrics.queue_lengths[1] << ",";
        out << "\"south\":" << snapshot.metrics.queue_lengths[2] << ",";
        out << "\"west\":" << snapshot.metrics.queue_lengths[3];
        out << "}},";
        out << "\"lights\":{";
        out << "\"north\":\"" << toString(snapshot.lights.north) << "\",";
        out << "\"east\":\"" << toString(snapshot.lights.east) << "\",";
        out << "\"south\":\"" << toString(snapshot.lights.south) << "\",";
        out << "\"west\":\"" << toString(snapshot.lights.west) << "\",";
        out << "\"turnSouthEast\":\"" << toString(snapshot.lights.turnSouthEast) << "\",";
        out << "\"turnNorthWest\":\"" << toString(snapshot.lights.turnNorthWest) << "\",";
        out << "\"turnWestSouth\":\"" << toString(snapshot.lights.turnWestSouth) << "\",";
        out << "\"turnEastNorth\":\"" << toString(snapshot.lights.turnEastNorth) << "\",";
        out << "\"turnNorthEast\":\"" << toString(snapshot.lights.turnNorthEast) << "\",";
        out << "\"turnSouthWest\":\"" << toString(snapshot.lights.turnSouthWest) << "\",";
        out << "\"turnEastSouth\":\"" << toString(snapshot.lights.turnEastSouth) << "\",";
        out << "\"turnWestNorth\":\"" << toString(snapshot.lights.turnWestNorth) << "\"";
        out << "},";
        out << "\"left_protection\":{";
        out << "\"north\":" << (protectedLeftAllowed(Direction::North, snapshot.lights) ? "true" : "false") << ",";
        out << "\"east\":" << (protectedLeftAllowed(Direction::East, snapshot.lights) ? "true" : "false") << ",";
        out << "\"south\":" << (protectedLeftAllowed(Direction::South, snapshot.lights) ? "true" : "false") << ",";
        out << "\"west\":" << (protectedLeftAllowed(Direction::West, snapshot.lights) ? "true" : "false");
        out << "},";
        out << "\"spawn\":{";
        out << "\"rate\":" << traffic.getArrivalRate() << ",";
        auto spawn_filter = traffic.getSpawnLaneFilter();
        if (spawn_filter.has_value())
        {
            out << "\"focus\":{";
            out << "\"active\":true,";
            out << "\"approach\":\"" << toString(spawn_filter->approach) << "\",";
            out << "\"lane_index\":" << spawn_filter->lane_index;
            out << "}";
        }
        else
        {
            out << "\"focus\":{\"active\":false}";
        }
        out << "},";
        out << "\"lanes\":{";
        out << "\"north\":";
        appendLaneVehicles(out, northVehicles);
        out << ",\"east\":";
        appendLaneVehicles(out, eastVehicles);
        out << ",\"south\":";
        appendLaneVehicles(out, southVehicles);
        out << ",\"west\":";
        appendLaneVehicles(out, westVehicles);
        out << "}}";
        return out.str();
    }

    void SimulatorEngine::reset()
    {
        current_time = 0.0;
        running = false;
        safety_violations = 0;
        traffic.reset();
        setControlMode(ControlMode::Basic);
        right_turn_green_hold_until = {0.0, 0.0, 0.0, 0.0};
        refreshEffectiveSignalState();
    }

    void SimulatorEngine::start()
    {
        running = true;
    }

    void SimulatorEngine::stop()
    {
        running = false;
    }

    bool SimulatorEngine::isRunning() const
    {
        return running;
    }

    void SimulatorEngine::handleCommand(UICommand command, double dt)
    {
        switch (command)
        {
        case UICommand::Start:
            start();
            break;
        case UICommand::Stop:
            stop();
            break;
        case UICommand::Reset:
            reset();
            break;
        case UICommand::Step:
            if (!running)
            {
                start();
                tick(dt);
                stop();
            }
            else
            {
                tick(dt);
            }
            break;
        }
    }

    void SimulatorEngine::setControlMode(ControlMode mode)
    {
        control_mode = mode;

        if (control_mode == ControlMode::Basic)
        {
            if (intersection_config.signal_groups.empty())
            {
                controller = std::make_unique<BasicControllerAdapter>(ns_duration, ew_duration);
            }
            else
            {
                controller = std::make_unique<ConfigurableSignalGroupController>(intersection_config);
            }
        }
        else
        {
            controller = std::make_unique<NullControlController>();
        }

        controller->reset();
    }

    SimulatorEngine::ControlMode SimulatorEngine::getControlMode() const
    {
        return control_mode;
    }

    void SimulatorEngine::setController(std::unique_ptr<ITrafficLightController> custom_controller, ControlMode mode)
    {
        control_mode = mode;
        controller = std::move(custom_controller);
        if (controller)
        {
            controller->reset();
        }
    }

    const IntersectionConfig &SimulatorEngine::getIntersectionConfig() const
    {
        return intersection_config;
    }

    void SimulatorEngine::setSpawnLaneFilter(const std::optional<TrafficGenerator::SpawnLaneFilter> &filter)
    {
        traffic.setSpawnLaneFilter(filter);
    }

    std::optional<TrafficGenerator::SpawnLaneFilter> SimulatorEngine::getSpawnLaneFilter() const
    {
        return traffic.getSpawnLaneFilter();
    }

    void SimulatorEngine::setTrafficRate(double rate)
    {
        traffic.setArrivalRate(rate);
    }

    double SimulatorEngine::getTrafficRate() const
    {
        return traffic.getArrivalRate();
    }

    bool SimulatorEngine::isLightGreen(Direction dir) const
    {
        auto state = getCurrentLightState();
        switch (dir)
        {
        case Direction::North:
            return state.north == LightState::Green;
        case Direction::South:
            return state.south == LightState::Green;
        case Direction::East:
            return state.east == LightState::Green;
        case Direction::West:
            return state.west == LightState::Green;
        }
        return false;
    }

    bool SimulatorEngine::signalAllowsVehicle(Direction dir,
                                              const Vehicle &vehicle,
                                              const LaneConfig *lane_cfg,
                                              const IntersectionState &state) const
    {
        const bool has_traffic_light = lane_cfg ? lane_cfg->has_traffic_light : true;
        const bool effective_left_turn = isEffectiveLeftTurn(dir, vehicle);

        if (lane_cfg && !lane_cfg->allowed_movements.empty())
        {
            const bool movement_allowed = std::find(lane_cfg->allowed_movements.begin(),
                                                    lane_cfg->allowed_movements.end(),
                                                    vehicle.movement) != lane_cfg->allowed_movements.end();
            if (!movement_allowed)
            {
                return false;
            }
        }

        if (!has_traffic_light)
        {
            if (effective_left_turn)
            {
                return false;
            }
            return true;
        }

        bool main_green = false;
        switch (dir)
        {
        case Direction::North:
            main_green = state.north == LightState::Green;
            break;
        case Direction::South:
            main_green = state.south == LightState::Green;
            break;
        case Direction::East:
            main_green = state.east == LightState::Green;
            break;
        case Direction::West:
            main_green = state.west == LightState::Green;
            break;
        }
        const bool turn_green = rightTurnLightFor(dir, state) == LightState::Green;
        const bool left_turn_green = leftTurnLightFor(dir, state) == LightState::Green;
        const bool dedicated_right = lane_cfg ? isDedicatedRightTurnLane(*lane_cfg) : false;
        const bool dedicated_left = lane_cfg ? isDedicatedLeftTurnLane(*lane_cfg) : false;

        switch (vehicle.movement)
        {
        case MovementType::Straight:
            return main_green;
        case MovementType::Left:
            if (dedicated_left)
            {
                return left_turn_green;
            }
            return main_green;
        case MovementType::Right:
            if (dedicated_right)
            {
                return turn_green;
            }
            return main_green || turn_green;
        }

        return false;
    }

    std::vector<SignalGroupId> SimulatorEngine::resolveActiveSignalGroups(const IntersectionState &state) const
    {
        auto is_active = [](LightState value)
        { return value == LightState::Green || value == LightState::Orange; };

        auto findApproachForLane = [&](LaneId lane_id, ApproachId &approach_out)
        {
            for (const auto &approach : intersection_config.approaches)
            {
                auto lane_it = std::find_if(approach.lanes.begin(), approach.lanes.end(),
                                            [lane_id](const LaneConfig &lane)
                                            { return lane.id == lane_id; });
                if (lane_it != approach.lanes.end())
                {
                    approach_out = approach.id;
                    return true;
                }
            }
            return false;
        };

        std::unordered_set<SignalGroupId> active_ids;

        auto includeMatchingGroups = [&](ApproachId approach, MovementType movement)
        {
            for (const auto &group : intersection_config.signal_groups)
            {
                bool movement_match = std::find(group.green_movements.begin(), group.green_movements.end(), movement) != group.green_movements.end();
                if (!movement_match)
                {
                    continue;
                }

                for (LaneId lane_id : group.controlled_lanes)
                {
                    ApproachId lane_approach;
                    if (findApproachForLane(lane_id, lane_approach) && lane_approach == approach)
                    {
                        active_ids.insert(group.id);
                        break;
                    }
                }
            }
        };

        if (is_active(state.north))
        {
            includeMatchingGroups(ApproachId::North, MovementType::Straight);
            includeMatchingGroups(ApproachId::North, MovementType::Left);
        }
        if (is_active(state.south))
        {
            includeMatchingGroups(ApproachId::South, MovementType::Straight);
            includeMatchingGroups(ApproachId::South, MovementType::Left);
        }
        if (is_active(state.east))
        {
            includeMatchingGroups(ApproachId::East, MovementType::Straight);
            includeMatchingGroups(ApproachId::East, MovementType::Left);
        }
        if (is_active(state.west))
        {
            includeMatchingGroups(ApproachId::West, MovementType::Straight);
            includeMatchingGroups(ApproachId::West, MovementType::Left);
        }

        if (is_active(state.turnSouthEast))
            includeMatchingGroups(ApproachId::South, MovementType::Right);
        if (is_active(state.turnNorthWest))
            includeMatchingGroups(ApproachId::North, MovementType::Right);
        if (is_active(state.turnWestSouth))
            includeMatchingGroups(ApproachId::West, MovementType::Right);
        if (is_active(state.turnEastNorth))
            includeMatchingGroups(ApproachId::East, MovementType::Right);

        return std::vector<SignalGroupId>(active_ids.begin(), active_ids.end());
    }

    bool SimulatorEngine::isConfigSignalStateSafe(const IntersectionState &state) const
    {
        if (intersection_config.signal_groups.empty())
        {
            return true;
        }

        if (!checker.isConfigValid())
        {
            return false;
        }

        std::vector<SignalGroupId> active_groups = resolveActiveSignalGroups(state);
        if (active_groups.empty())
        {
            return true;
        }

        return checker.areSignalGroupsConflictFree(active_groups);
    }

} // namespace crossroads