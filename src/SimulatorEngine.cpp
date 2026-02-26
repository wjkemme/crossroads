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

        advanceController(dt);

        generateTraffic(dt);
        std::array<bool, 4> lane_can_move = {
            isLightGreen(Direction::North),
            isLightGreen(Direction::South),
            isLightGreen(Direction::East),
            isLightGreen(Direction::West)};
        traffic.updateVehicleSpeeds(dt, lane_can_move);
        processVehicleCrossings();
        completeVehicleCrossings();

        IntersectionState current_state = getCurrentLightState();
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

    void SimulatorEngine::generateTraffic(double dt)
    {
        traffic.generateTraffic(dt, current_time);
    }

    void SimulatorEngine::processVehicleCrossings()
    {
        // Start crossing for ALL vehicles that have reached de stopzone (4m voor streep)
        const double STOP_TARGET = 69.5;

        for (int dir = 0; dir < 4; ++dir)
        {
            Direction lane = static_cast<Direction>(dir);
            auto &queue = traffic.getQueueByDirection(lane);
            for (auto &vehicle : queue)
            {
                if (!vehicle.isWaiting() || vehicle.position_in_lane < STOP_TARGET)
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

                bool can_cross = !has_traffic_light || isLightGreen(lane);
                if (can_cross)
                {
                    vehicle.crossing_time = current_time;
                }
            }
        }
    }

    void SimulatorEngine::completeVehicleCrossings()
    {
        // Complete crossing for the front vehicle when it has finished
        for (int dir = 0; dir < 4; ++dir)
        {
            Direction lane = static_cast<Direction>(dir);
            Vehicle *v = traffic.peekNextVehicle(lane);
            if (v && v->isCrossing())
            {
                double crossing_time = v->getCrossingDuration(traffic.getQueueLength(lane));
                if (current_time - v->crossing_time >= crossing_time)
                {
                    traffic.completeCrossing(v->id, current_time);
                }
            }
        }
    }

    void SimulatorEngine::advanceController(double dt)
    {
        controller->tick(dt);
    }

    IntersectionState SimulatorEngine::getCurrentLightState() const
    {
        return controller->getCurrentState();
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
        snapshot.lights = getCurrentLightState();
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
        out << "\"turnEastNorth\":\"" << toString(snapshot.lights.turnEastNorth) << "\"";
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