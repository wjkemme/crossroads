#include "SimulatorEngine.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <sstream>
#include <unordered_set>
#include <utility>

namespace crossroads {
    namespace {
        ApproachId approachFromDirection(Direction dir) {
            switch (dir) {
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

        const LaneConfig* findLaneConfigForVehicle(const IntersectionConfig& config, Direction dir, LaneId lane_id) {
            ApproachId approach = approachFromDirection(dir);
            auto approach_it = std::find_if(config.approaches.begin(),
                                            config.approaches.end(),
                                            [approach](const ApproachConfig& entry) { return entry.id == approach; });
            if (approach_it == config.approaches.end()) {
                return nullptr;
            }

            auto lane_it = std::find_if(approach_it->lanes.begin(),
                                        approach_it->lanes.end(),
                                        [lane_id](const LaneConfig& lane) { return lane.id == lane_id; });
            if (lane_it == approach_it->lanes.end()) {
                return nullptr;
            }
            return &(*lane_it);
        }

        double estimatedCrossingPathMeters(MovementType movement) {
            switch (movement) {
                case MovementType::Right:
                    return 10.0;
                case MovementType::Left:
                    return 20.0;
                case MovementType::Straight:
                default:
                    return 18.0;
            }
        }

        ApproachId leftDestinationFor(Direction lane) {
            switch (lane) {
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

        bool isEffectiveLeftTurn(Direction lane, const Vehicle& vehicle) {
            if (vehicle.movement == MovementType::Left) {
                return true;
            }
            return vehicle.destination_approach == leftDestinationFor(lane);
        }

        bool protectedLeftAllowed(Direction lane, const IntersectionState& state) {
            switch (lane) {
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

        Direction directionFromApproach(ApproachId approach) {
            switch (approach) {
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

        bool isDedicatedRightTurnLane(const LaneConfig& lane_cfg) {
            return lane_cfg.connected_to_intersection && lane_cfg.has_traffic_light &&
                   lane_cfg.allowed_movements.size() == 1 && lane_cfg.allowed_movements.front() == MovementType::Right;
        }

        bool isDedicatedLeftTurnLane(const LaneConfig& lane_cfg) {
            return lane_cfg.connected_to_intersection && lane_cfg.has_traffic_light &&
                   lane_cfg.allowed_movements.size() == 1 && lane_cfg.allowed_movements.front() == MovementType::Left;
        }

        bool isApproachMainRed(ApproachId approach, const IntersectionState& state) {
            switch (approach) {
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

        LightState approachMainLight(ApproachId approach, const IntersectionState& state) {
            switch (approach) {
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

        bool hasExclusiveLaneConnection(const IntersectionConfig& config,
                                        ApproachId from_approach,
                                        LaneId from_lane_id,
                                        MovementType movement) {
            uint16_t from_lane_index = static_cast<uint16_t>(from_lane_id % 100);
            auto approach_it =
                std::find_if(config.approaches.begin(), config.approaches.end(), [&](const ApproachConfig& approach) {
                    return approach.id == from_approach;
                });
            if (approach_it != config.approaches.end()) {
                auto lane_it = std::find_if(approach_it->lanes.begin(),
                                            approach_it->lanes.end(),
                                            [&](const LaneConfig& lane) { return lane.id == from_lane_id; });
                if (lane_it != approach_it->lanes.end()) {
                    from_lane_index = static_cast<uint16_t>(std::distance(approach_it->lanes.begin(), lane_it));
                }
            }

            auto self_connection_it = std::find_if(config.lane_connections.begin(),
                                                   config.lane_connections.end(),
                                                   [&](const LaneConnectionConfig& connection) {
                                                       return connection.from_approach == from_approach &&
                                                              connection.from_lane_index == from_lane_index &&
                                                              connection.movement == movement;
                                                   });
            if (self_connection_it == config.lane_connections.end()) {
                return false;
            }

            const ApproachId to_approach = self_connection_it->to_approach;
            const uint16_t to_lane_index = self_connection_it->to_lane_index;
            const size_t incoming_count = static_cast<size_t>(std::count_if(
                config.lane_connections.begin(),
                config.lane_connections.end(),
                [&](const LaneConnectionConfig& connection) {
                    return connection.to_approach == to_approach && connection.to_lane_index == to_lane_index;
                }));

            return incoming_count == 1;
        }

        void setRightTurnLight(IntersectionState& state, ApproachId approach, LightState color) {
            switch (approach) {
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

        void setApproachMainLight(IntersectionState& state, ApproachId approach, LightState color) {
            switch (approach) {
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

        void setLeftTurnLight(IntersectionState& state, ApproachId approach, LightState color) {
            switch (approach) {
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

        LightState rightTurnLightFor(Direction dir, const IntersectionState& state) {
            switch (dir) {
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

        LightState leftTurnLightFor(Direction dir, const IntersectionState& state) {
            switch (dir) {
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

        ApproachId opposingApproachForRightTurn(ApproachId approach) {
            switch (approach) {
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

        ApproachId opposingApproachForLeftTurn(ApproachId approach) {
            switch (approach) {
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

        bool hasRedToOrangeTransition(const IntersectionState& prev, const IntersectionState& next) {
            auto is_invalid = [](LightState from, LightState to) {
                return from == LightState::Red && to == LightState::Orange;
            };

            return is_invalid(prev.north, next.north) || is_invalid(prev.east, next.east) ||
                   is_invalid(prev.south, next.south) || is_invalid(prev.west, next.west) ||
                   is_invalid(prev.turnSouthEast, next.turnSouthEast) ||
                   is_invalid(prev.turnNorthWest, next.turnNorthWest) ||
                   is_invalid(prev.turnWestSouth, next.turnWestSouth) ||
                   is_invalid(prev.turnEastNorth, next.turnEastNorth) ||
                   is_invalid(prev.turnNorthEast, next.turnNorthEast) ||
                   is_invalid(prev.turnSouthWest, next.turnSouthWest) ||
                   is_invalid(prev.turnEastSouth, next.turnEastSouth) ||
                   is_invalid(prev.turnWestNorth, next.turnWestNorth);
        }

        constexpr std::size_t kRouteCount = 12;

        std::size_t routeIndex(ApproachId approach, MovementType movement) {
            std::size_t approach_idx = approachIndex(approach);
            std::size_t movement_idx = 0;
            switch (movement) {
                case MovementType::Straight:
                    movement_idx = 0;
                    break;
                case MovementType::Left:
                    movement_idx = 1;
                    break;
                case MovementType::Right:
                    movement_idx = 2;
                    break;
            }
            return approach_idx * 3 + movement_idx;
        }

        bool areOpposingApproaches(ApproachId lhs, ApproachId rhs) {
            return (lhs == ApproachId::North && rhs == ApproachId::South) ||
                   (lhs == ApproachId::South && rhs == ApproachId::North) ||
                   (lhs == ApproachId::East && rhs == ApproachId::West) ||
                   (lhs == ApproachId::West && rhs == ApproachId::East);
        }

        const char* routeName(ApproachId approach, MovementType movement) {
            switch (approach) {
                case ApproachId::North:
                    switch (movement) {
                        case MovementType::Straight:
                            return "N->S";
                        case MovementType::Left:
                            return "N->E";
                        case MovementType::Right:
                            return "N->W";
                    }
                    break;
                case ApproachId::East:
                    switch (movement) {
                        case MovementType::Straight:
                            return "E->W";
                        case MovementType::Left:
                            return "E->S";
                        case MovementType::Right:
                            return "E->N";
                    }
                    break;
                case ApproachId::South:
                    switch (movement) {
                        case MovementType::Straight:
                            return "S->N";
                        case MovementType::Left:
                            return "S->W";
                        case MovementType::Right:
                            return "S->E";
                    }
                    break;
                case ApproachId::West:
                    switch (movement) {
                        case MovementType::Straight:
                            return "W->E";
                        case MovementType::Left:
                            return "W->N";
                        case MovementType::Right:
                            return "W->S";
                    }
                    break;
            }
            return "?";
        }

        void setRouteLight(IntersectionState& state, ApproachId approach, MovementType movement, LightState color) {
            switch (movement) {
                case MovementType::Straight:
                    setApproachMainLight(state, approach, color);
                    break;
                case MovementType::Left:
                    setLeftTurnLight(state, approach, color);
                    break;
                case MovementType::Right:
                    setRightTurnLight(state, approach, color);
                    break;
            }
        }

        bool routeIsGreen(const IntersectionState& state, ApproachId approach, MovementType movement) {
            switch (movement) {
                case MovementType::Straight:
                    return approachMainLight(approach, state) == LightState::Green;
                case MovementType::Left:
                    return leftTurnLightFor(directionFromApproach(approach), state) == LightState::Green;
                case MovementType::Right:
                    return rightTurnLightFor(directionFromApproach(approach), state) == LightState::Green;
            }
            return false;
        }

        struct RoutePoint {
            double x = 0.0;
            double y = 0.0;
        };

        double laneLateralOffset(ApproachId approach, std::size_t lane_index, std::size_t lane_count) {
            if (lane_count <= 1) {
                return 0.0;
            }

            const double step = 0.22;
            const double half_span = (static_cast<double>(lane_count - 1) * step) * 0.5;
            const double raw = -half_span + static_cast<double>(lane_index) * step;

            switch (approach) {
                case ApproachId::North:
                    return -raw;
                case ApproachId::South:
                    return raw;
                case ApproachId::East:
                    return raw;
                case ApproachId::West:
                    return -raw;
            }
            return raw;
        }

        RoutePoint entryPointFor(ApproachId approach, std::size_t lane_index, std::size_t lane_count) {
            const double lateral = laneLateralOffset(approach, lane_index, lane_count);
            switch (approach) {
                case ApproachId::North:
                    return {lateral, 1.0};
                case ApproachId::South:
                    return {lateral, -1.0};
                case ApproachId::East:
                    return {1.0, lateral};
                case ApproachId::West:
                    return {-1.0, lateral};
            }
            return {0.0, 0.0};
        }

        RoutePoint exitPointFor(ApproachId approach, std::size_t lane_index, std::size_t lane_count) {
            const double lateral = laneLateralOffset(approach, lane_index, lane_count);
            switch (approach) {
                case ApproachId::North:
                    return {lateral, 1.0};
                case ApproachId::South:
                    return {lateral, -1.0};
                case ApproachId::East:
                    return {1.0, lateral};
                case ApproachId::West:
                    return {-1.0, lateral};
            }
            return {0.0, 0.0};
        }

        RoutePoint inboundDirection(ApproachId approach) {
            switch (approach) {
                case ApproachId::North:
                    return {0.0, -1.0};
                case ApproachId::South:
                    return {0.0, 1.0};
                case ApproachId::East:
                    return {-1.0, 0.0};
                case ApproachId::West:
                    return {1.0, 0.0};
            }
            return {0.0, 0.0};
        }

        RoutePoint outboundDirection(ApproachId approach) {
            switch (approach) {
                case ApproachId::North:
                    return {0.0, 1.0};
                case ApproachId::South:
                    return {0.0, -1.0};
                case ApproachId::East:
                    return {1.0, 0.0};
                case ApproachId::West:
                    return {-1.0, 0.0};
            }
            return {0.0, 0.0};
        }

        std::vector<RoutePoint> sampleConnectionPath(const IntersectionConfig& config,
                                                     const LaneConnectionConfig& connection) {
            const auto& from_cfg = config.approaches[approachIndex(connection.from_approach)];
            const auto& to_cfg = config.approaches[approachIndex(connection.to_approach)];
            const std::size_t from_count = std::max<std::size_t>(1, from_cfg.lanes.size());
            const std::size_t to_count = std::max<std::size_t>(1, effectiveToLaneCount(to_cfg));

            const RoutePoint p0 = entryPointFor(connection.from_approach, connection.from_lane_index, from_count);
            const RoutePoint p3 = exitPointFor(connection.to_approach, connection.to_lane_index, to_count);

            if (connection.movement == MovementType::Straight) {
                return {p0, p3};
            }

            if (connection.movement == MovementType::Left) {
                return {p0, RoutePoint{0.0, 0.0}, p3};
            }

            const RoutePoint in_dir = inboundDirection(connection.from_approach);
            const RoutePoint out_dir = outboundDirection(connection.to_approach);
            const double control_distance = 0.18;

            const RoutePoint c1 = {p0.x + in_dir.x * control_distance, p0.y + in_dir.y * control_distance};
            const RoutePoint c2 = {p3.x - out_dir.x * control_distance, p3.y - out_dir.y * control_distance};

            constexpr int samples = 18;
            std::vector<RoutePoint> path;
            path.reserve(samples + 1);
            for (int i = 0; i <= samples; ++i) {
                const double t = static_cast<double>(i) / static_cast<double>(samples);
                const double mt = 1.0 - t;
                const double b0 = mt * mt * mt;
                const double b1 = 3.0 * mt * mt * t;
                const double b2 = 3.0 * mt * t * t;
                const double b3 = t * t * t;
                path.push_back(
                    {b0 * p0.x + b1 * c1.x + b2 * c2.x + b3 * p3.x, b0 * p0.y + b1 * c1.y + b2 * c2.y + b3 * p3.y});
            }
            return path;
        }

        double crossProduct(const RoutePoint& a, const RoutePoint& b, const RoutePoint& c) {
            return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
        }

        bool segmentIntersects(const RoutePoint& a, const RoutePoint& b, const RoutePoint& c, const RoutePoint& d) {
            constexpr double eps = 1e-9;

            const double o1 = crossProduct(a, b, c);
            const double o2 = crossProduct(a, b, d);
            const double o3 = crossProduct(c, d, a);
            const double o4 = crossProduct(c, d, b);

            const bool ab_straddles_cd = (o1 > eps && o2 < -eps) || (o1 < -eps && o2 > eps);
            const bool cd_straddles_ab = (o3 > eps && o4 < -eps) || (o3 < -eps && o4 > eps);
            return ab_straddles_cd && cd_straddles_ab;
        }

        bool pointInCore(const RoutePoint& p) {
            return std::abs(p.x) <= 0.55 && std::abs(p.y) <= 0.55;
        }

        bool segmentIntersectsCore(const RoutePoint& a, const RoutePoint& b) {
            if (pointInCore(a) || pointInCore(b)) {
                return true;
            }

            constexpr double core = 0.55;
            constexpr double eps = 1e-12;
            double t0 = 0.0;
            double t1 = 1.0;
            const double dx = b.x - a.x;
            const double dy = b.y - a.y;

            auto clip = [&](double p, double q) {
                if (std::abs(p) <= eps) {
                    return q >= 0.0;
                }

                const double r = q / p;
                if (p < 0.0) {
                    if (r > t1) {
                        return false;
                    }
                    if (r > t0) {
                        t0 = r;
                    }
                } else {
                    if (r < t0) {
                        return false;
                    }
                    if (r < t1) {
                        t1 = r;
                    }
                }

                return true;
            };

            return clip(-dx, a.x + core) && clip(dx, core - a.x) && clip(-dy, a.y + core) && clip(dy, core - a.y);
        }

        bool pathsConflictInCore(const std::vector<RoutePoint>& lhs, const std::vector<RoutePoint>& rhs) {
            if (lhs.size() < 2 || rhs.size() < 2) {
                return false;
            }

            for (std::size_t i = 1; i < lhs.size(); ++i) {
                const RoutePoint a = lhs[i - 1];
                const RoutePoint b = lhs[i];
                if (!segmentIntersectsCore(a, b)) {
                    continue;
                }

                for (std::size_t j = 1; j < rhs.size(); ++j) {
                    const RoutePoint c = rhs[j - 1];
                    const RoutePoint d = rhs[j];
                    if (!segmentIntersectsCore(c, d)) {
                        continue;
                    }

                    if (segmentIntersects(a, b, c, d)) {
                        return true;
                    }
                }
            }
            return false;
        }
    }  // namespace

    SimulatorEngine::SimulatorEngine(double traffic_rate, double ns_duration, double ew_duration)
        : SimulatorEngine(makeDefaultIntersectionConfig(), traffic_rate, ns_duration, ew_duration) {
    }

    SimulatorEngine::SimulatorEngine(const IntersectionConfig& intersection_config,
                                     double traffic_rate,
                                     double ns_duration,
                                     double ew_duration)
        : checker(intersection_config)
        , control_mode(ControlMode::Basic)
        , traffic(intersection_config, traffic_rate)
        , ns_duration(ns_duration)
        , ew_duration(ew_duration)
        , intersection_config(intersection_config)
        , current_time(0.0)
        , safety_violations(0) {
        if (this->intersection_config.signal_groups.empty()) {
            controller = std::make_unique<BasicControllerAdapter>(ns_duration, ew_duration);
        } else {
            controller = std::make_unique<ConfigurableSignalGroupController>(this->intersection_config);
        }

        route_last_served_time.fill(-1.0);
        route_green_started_at.fill(-1.0);
        route_vehicles_started_this_green.fill(0);
        route_initial_waiting_count.fill(0);
        route_crossing_vehicle_count.fill(0);
        route_conflicts_cleared_at.fill(-1.0);
        scheduler_served_this_cycle.fill(false);
        scheduler_clearance_blocked_routes.fill(false);

        refreshEffectiveSignalState(0.0);
        previous_controller_state = controller ? controller->getCurrentState() : IntersectionState{};
        has_previous_controller_state = true;
    }

    void SimulatorEngine::simulate(double duration_seconds, double time_step) {
        reset();
        start();
        while (current_time < duration_seconds)
            tick(time_step);
        stop();
    }

    void SimulatorEngine::tick(double dt) {
        if (!running) {
            return;
        }

        std::array<bool, 4> approach_demand = {!traffic.getQueueByDirection(Direction::North).empty(),
                                               !traffic.getQueueByDirection(Direction::South).empty(),
                                               !traffic.getQueueByDirection(Direction::East).empty(),
                                               !traffic.getQueueByDirection(Direction::West).empty()};
        if (controller) {
            controller->setDemandByDirection(approach_demand);
        }

        advanceController(dt);

        generateTraffic(dt);
        refreshEffectiveSignalState(dt);
        std::array<bool, 4> lane_can_move = {effective_light_state.north == LightState::Green,
                                             effective_light_state.south == LightState::Green,
                                             effective_light_state.east == LightState::Green,
                                             effective_light_state.west == LightState::Green};
        traffic.updateVehicleSpeeds(dt, lane_can_move, [&](Direction dir, const Vehicle& vehicle) {
            const LaneConfig* lane_cfg = findLaneConfigForVehicle(intersection_config, dir, vehicle.lane_id);
            if (lane_cfg && !lane_cfg->connected_to_intersection) {
                return false;
            }
            return signalAllowsVehicle(dir, vehicle, lane_cfg, effective_light_state);
        });
        processVehicleCrossings();
        completeVehicleCrossings();

        IntersectionState current_state = controller ? controller->getCurrentState() : IntersectionState{};
        bool transition_valid = true;
        if (control_mode != ControlMode::NullControl && has_previous_controller_state) {
            transition_valid = !hasRedToOrangeTransition(previous_controller_state, current_state);
        }

        if (!transition_valid || !checker.isSafe(current_state) || !isConfigSignalStateSafe(current_state)) {
            safety_violations++;
            if (control_mode != ControlMode::NullControl) {
                setControlMode(ControlMode::NullControl);
            }
        }

        previous_controller_state = controller ? controller->getCurrentState() : IntersectionState{};
        has_previous_controller_state = true;

        current_time += dt;
    }

    void SimulatorEngine::refreshEffectiveSignalState(double dt_seconds) {
        IntersectionState base_state = controller ? controller->getCurrentState() : IntersectionState{};
        const IntersectionState prev_effective =
            has_previous_effective_light_state ? previous_effective_light_state : IntersectionState{};
        const auto prev_route_green_active = route_green_active;

        if (!route_conflict_matrix_ready) {
            route_conflict_matrix = {};
            route_configured.fill(false);

            std::array<std::vector<std::vector<RoutePoint>>, kRouteCount> route_paths;
            std::array<std::vector<LaneConnectionConfig>, kRouteCount> route_connections;
            for (const auto& connection : intersection_config.lane_connections) {
                const std::size_t idx = routeIndex(connection.from_approach, connection.movement);
                route_paths[idx].push_back(sampleConnectionPath(intersection_config, connection));
                route_connections[idx].push_back(connection);
                route_configured[idx] = true;
            }

            for (std::size_t i = 0; i < kRouteCount; ++i) {
                for (std::size_t j = 0; j < kRouteCount; ++j) {
                    if (i == j || !route_configured[i] || !route_configured[j]) {
                        route_conflict_matrix[i][j] = false;
                        continue;
                    }

                    const ApproachId route_i_approach = static_cast<ApproachId>(i / 3);
                    const ApproachId route_j_approach = static_cast<ApproachId>(j / 3);
                    const MovementType route_i_movement = static_cast<MovementType>(i % 3);
                    const MovementType route_j_movement = static_cast<MovementType>(j % 3);
                    if (route_i_movement == MovementType::Straight && route_j_movement == MovementType::Straight &&
                        areOpposingApproaches(route_i_approach, route_j_approach)) {
                        route_conflict_matrix[i][j] = false;
                        continue;
                    }

                    bool has_conflict = false;

                    for (const auto& lhs_connection : route_connections[i]) {
                        for (const auto& rhs_connection : route_connections[j]) {
                            if (lhs_connection.to_approach == rhs_connection.to_approach &&
                                lhs_connection.to_lane_index == rhs_connection.to_lane_index) {
                                has_conflict = true;
                                break;
                            }
                        }
                        if (has_conflict) {
                            break;
                        }
                    }

                    for (const auto& lhs_path : route_paths[i]) {
                        for (const auto& rhs_path : route_paths[j]) {
                            if (pathsConflictInCore(lhs_path, rhs_path)) {
                                has_conflict = true;
                                break;
                            }
                        }
                        if (has_conflict) {
                            break;
                        }
                    }

                    route_conflict_matrix[i][j] = has_conflict;
                }
            }

            for (std::size_t i = 0; i < kRouteCount; ++i) {
                for (std::size_t j = 0; j < kRouteCount; ++j) {
                    if (!route_configured[i] || !route_configured[j]) {
                        route_conflict_matrix[i][j] = false;
                        continue;
                    }
                    route_conflict_matrix[i][j] = route_conflict_matrix[i][j] || route_conflict_matrix[j][i];
                }
            }

            route_conflict_matrix_ready = true;
        }

        scheduler_anchor_route_index = -1;
        scheduler_parallel_routes.fill(false);
        scheduler_blocked_routes.fill(false);
        scheduler_safety_blocked_routes.fill(false);
        scheduler_clearance_blocked_routes.fill(false);
        route_waiting_demand.fill(false);
        route_wait_seconds.fill(0.0);
        route_priority_score.fill(0.0);
        route_green_active.fill(false);

        std::array<bool, 4> has_left_waiting_demand = {false, false, false, false};
        std::array<bool, 4> has_straight_waiting_demand = {false, false, false, false};
        std::array<bool, 4> has_right_waiting_demand = {false, false, false, false};

        for (const auto& approach_cfg : intersection_config.approaches) {
            const ApproachId approach = approach_cfg.id;
            const Direction dir = directionFromApproach(approach);
            const auto& queue = traffic.getQueueByDirection(dir);
            const std::size_t idx = approachIndex(approach);

            has_left_waiting_demand[idx] = std::any_of(queue.begin(), queue.end(), [&](const Vehicle& vehicle) {
                return vehicle.isWaiting() && isEffectiveLeftTurn(dir, vehicle);
            });

            has_straight_waiting_demand[idx] = std::any_of(queue.begin(), queue.end(), [](const Vehicle& vehicle) {
                return vehicle.isWaiting() && vehicle.movement == MovementType::Straight;
            });

            has_right_waiting_demand[idx] = std::any_of(queue.begin(), queue.end(), [](const Vehicle& vehicle) {
                return vehicle.isWaiting() && vehicle.movement == MovementType::Right;
            });
        }

        if (traffic.getTotalWaiting() == 0) {
            effective_light_state = IntersectionState{};
            previous_effective_light_state = effective_light_state;
            has_previous_effective_light_state = true;
            minimum_green_hold_until_seconds.fill(0.0);
            left_wait_seconds.fill(0.0);
            straight_wait_seconds.fill(0.0);
            right_wait_seconds.fill(0.0);
            return;
        }

        effective_light_state = base_state;

        for (const auto& approach_cfg : intersection_config.approaches) {
            const Direction dir = directionFromApproach(approach_cfg.id);
            const auto& queue = traffic.getQueueByDirection(dir);
            if (queue.empty()) {
                setApproachMainLight(effective_light_state, approach_cfg.id, LightState::Red);
            }
        }

        for (const auto& approach_cfg : intersection_config.approaches) {
            const ApproachId approach = approach_cfg.id;
            const Direction dir = directionFromApproach(approach);
            const auto& queue = traffic.getQueueByDirection(dir);

            std::vector<LaneId> dedicated_right_lane_ids;
            std::vector<LaneId> exclusive_right_lane_ids;
            dedicated_right_lane_ids.reserve(approach_cfg.lanes.size());
            exclusive_right_lane_ids.reserve(approach_cfg.lanes.size());
            for (const auto& lane_cfg : approach_cfg.lanes) {
                if (isDedicatedRightTurnLane(lane_cfg)) {
                    dedicated_right_lane_ids.push_back(lane_cfg.id);
                    if (hasExclusiveLaneConnection(intersection_config, approach, lane_cfg.id, MovementType::Right)) {
                        exclusive_right_lane_ids.push_back(lane_cfg.id);
                    }
                }
            }

            if (dedicated_right_lane_ids.empty()) {
                setRightTurnLight(effective_light_state, approach, LightState::Red);
                continue;
            }

            const bool has_demand = std::any_of(queue.begin(), queue.end(), [&](const Vehicle& vehicle) {
                if (std::find(dedicated_right_lane_ids.begin(), dedicated_right_lane_ids.end(), vehicle.lane_id) ==
                    dedicated_right_lane_ids.end()) {
                    return false;
                }
                return vehicle.isWaiting() || vehicle.isCrossing();
            });

            const bool has_non_right_or_mixed_lane_demand =
                std::any_of(queue.begin(), queue.end(), [&](const Vehicle& vehicle) {
                    if (!vehicle.isWaiting() && !vehicle.isCrossing()) {
                        return false;
                    }
                    if (vehicle.movement != MovementType::Right) {
                        return true;
                    }
                    return std::find(dedicated_right_lane_ids.begin(),
                                     dedicated_right_lane_ids.end(),
                                     vehicle.lane_id) == dedicated_right_lane_ids.end();
                });
            if (!has_non_right_or_mixed_lane_demand) {
                setApproachMainLight(effective_light_state, approach, LightState::Red);
            }

            const bool has_exclusive_demand = std::any_of(queue.begin(), queue.end(), [&](const Vehicle& vehicle) {
                if (std::find(exclusive_right_lane_ids.begin(), exclusive_right_lane_ids.end(), vehicle.lane_id) ==
                    exclusive_right_lane_ids.end()) {
                    return false;
                }
                return vehicle.isWaiting() || vehicle.isCrossing();
            });

            const ApproachId opposing = opposingApproachForRightTurn(approach);
            const bool opposing_red = isApproachMainRed(opposing, effective_light_state);
            const std::size_t approach_idx = approachIndex(approach);

            if (has_demand && opposing_red) {
                right_turn_green_hold_until[approach_idx] =
                    std::max(right_turn_green_hold_until[approach_idx], current_time + right_turn_min_green_seconds);
            }

            const bool hold_active = right_turn_green_hold_until[approach_idx] > current_time;
            const ApproachId conflicting_left_approach = opposingApproachForLeftTurn(approach);
            const bool starved_conflicting_left =
                left_wait_seconds[approachIndex(conflicting_left_approach)] >= left_starvation_threshold_seconds;
            const bool fairness_blocks_right =
                starved_conflicting_left && has_left_waiting_demand[approachIndex(conflicting_left_approach)];

            const bool right_candidate_green =
                has_exclusive_demand ? (has_demand || hold_active) : (opposing_red && (has_demand || hold_active));
            const bool turn_green = right_candidate_green && !fairness_blocks_right;
            setRightTurnLight(effective_light_state, approach, turn_green ? LightState::Green : LightState::Red);
        }

        for (const auto& approach_cfg : intersection_config.approaches) {
            const ApproachId approach = approach_cfg.id;
            const Direction dir = directionFromApproach(approach);
            const auto& queue = traffic.getQueueByDirection(dir);

            std::vector<LaneId> dedicated_left_lane_ids;
            dedicated_left_lane_ids.reserve(approach_cfg.lanes.size());
            for (const auto& lane_cfg : approach_cfg.lanes) {
                if (isDedicatedLeftTurnLane(lane_cfg)) {
                    dedicated_left_lane_ids.push_back(lane_cfg.id);
                }
            }

            if (dedicated_left_lane_ids.empty()) {
                setLeftTurnLight(effective_light_state, approach, LightState::Red);
                continue;
            }

            const bool has_demand = std::any_of(queue.begin(), queue.end(), [&](const Vehicle& vehicle) {
                if (std::find(dedicated_left_lane_ids.begin(), dedicated_left_lane_ids.end(), vehicle.lane_id) ==
                    dedicated_left_lane_ids.end()) {
                    return false;
                }
                return vehicle.isWaiting() || vehicle.isCrossing();
            });

            const ApproachId opposing = opposingApproachForLeftTurn(approach);
            const bool opposing_red = isApproachMainRed(opposing, effective_light_state);
            const bool opposing_right_red =
                rightTurnLightFor(directionFromApproach(opposing), effective_light_state) == LightState::Red;
            bool cross_traffic_main_red = true;
            if (approach == ApproachId::North || approach == ApproachId::South) {
                cross_traffic_main_red =
                    (effective_light_state.east == LightState::Red && effective_light_state.west == LightState::Red);
            } else {
                cross_traffic_main_red =
                    (effective_light_state.north == LightState::Red && effective_light_state.south == LightState::Red);
            }

            const bool opposing_straight_starved =
                straight_wait_seconds[approachIndex(opposing)] >= straight_starvation_threshold_seconds;
            const bool opposing_has_straight_waiting = has_straight_waiting_demand[approachIndex(opposing)];
            const bool fairness_blocks_left = opposing_straight_starved && opposing_has_straight_waiting;

            const bool left_green =
                has_demand && opposing_red && opposing_right_red && cross_traffic_main_red && !fairness_blocks_left;
            setLeftTurnLight(effective_light_state, approach, left_green ? LightState::Green : LightState::Red);

            const bool has_dedicated_left_crossing =
                std::any_of(queue.begin(), queue.end(), [&](const Vehicle& vehicle) {
                    if (!vehicle.isCrossing()) {
                        return false;
                    }
                    if (std::find(dedicated_left_lane_ids.begin(), dedicated_left_lane_ids.end(), vehicle.lane_id) ==
                        dedicated_left_lane_ids.end()) {
                        return false;
                    }
                    return isEffectiveLeftTurn(dir, vehicle);
                });
            if (has_dedicated_left_crossing) {
                setApproachMainLight(effective_light_state, opposing, LightState::Red);
            }

            const bool unprotected_mixed_left = std::any_of(queue.begin(), queue.end(), [&](const Vehicle& vehicle) {
                if (!vehicle.isWaiting() && !vehicle.isCrossing()) {
                    return false;
                }
                if (!isEffectiveLeftTurn(dir, vehicle)) {
                    return false;
                }
                return std::find(dedicated_left_lane_ids.begin(), dedicated_left_lane_ids.end(), vehicle.lane_id) ==
                       dedicated_left_lane_ids.end();
            });

            if (unprotected_mixed_left && !(opposing_red && opposing_right_red)) {
                setApproachMainLight(effective_light_state, approach, LightState::Red);
            }

            const bool has_any_left_demand = std::any_of(queue.begin(), queue.end(), [&](const Vehicle& vehicle) {
                if (!vehicle.isWaiting() && !vehicle.isCrossing()) {
                    return false;
                }
                return isEffectiveLeftTurn(dir, vehicle);
            });
            if (has_any_left_demand) {
                const LightState approach_main = approachMainLight(approach, effective_light_state);
                if (approach_main == LightState::Green) {
                    const bool opposing_straight_starved =
                        straight_wait_seconds[approachIndex(opposing)] >= straight_starvation_threshold_seconds;
                    const bool opposing_has_straight_waiting = has_straight_waiting_demand[approachIndex(opposing)];
                    if (!(opposing_straight_starved && opposing_has_straight_waiting)) {
                        setApproachMainLight(effective_light_state, opposing, LightState::Red);
                    }
                }
            }
        }

        for (const auto& approach_cfg : intersection_config.approaches) {
            const ApproachId approach = approach_cfg.id;
            const Direction dir = directionFromApproach(approach);
            const auto& queue = traffic.getQueueByDirection(dir);
            const std::size_t idx = approachIndex(approach);

            bool left_is_being_served = false;
            bool straight_is_being_served = false;
            bool right_is_being_served = false;

            for (const auto& vehicle : queue) {
                if (vehicle.isCrossing()) {
                    if (isEffectiveLeftTurn(dir, vehicle)) {
                        left_is_being_served = true;
                    }
                    if (vehicle.movement == MovementType::Straight) {
                        straight_is_being_served = true;
                    }
                    if (vehicle.movement == MovementType::Right) {
                        right_is_being_served = true;
                    }
                }
            }

            if (has_left_waiting_demand[idx] && !left_is_being_served) {
                left_wait_seconds[idx] += std::max(0.0, dt_seconds);
            } else {
                left_wait_seconds[idx] = 0.0;
            }

            if (has_straight_waiting_demand[idx] && !straight_is_being_served) {
                straight_wait_seconds[idx] += std::max(0.0, dt_seconds);
            } else {
                straight_wait_seconds[idx] = 0.0;
            }

            if (has_right_waiting_demand[idx] && !right_is_being_served) {
                right_wait_seconds[idx] += std::max(0.0, dt_seconds);
            } else {
                right_wait_seconds[idx] = 0.0;
            }
        }

        struct RouteCandidate {
            ApproachId approach;
            MovementType movement;
            bool waiting_demand;
            std::size_t demand_count;
            double wait_seconds;
        };

        std::array<RouteCandidate, kRouteCount> routes{};
        for (int approach_int = 0; approach_int < 4; ++approach_int) {
            ApproachId approach = static_cast<ApproachId>(approach_int);
            for (int movement_int = 0; movement_int < 3; ++movement_int) {
                MovementType movement = static_cast<MovementType>(movement_int);
                const std::size_t idx = routeIndex(approach, movement);
                routes[idx] = {approach, movement, false, 0u, 0.0};
            }
        }

        for (const auto& approach_cfg : intersection_config.approaches) {
            const ApproachId approach = approach_cfg.id;
            const std::size_t idx = approachIndex(approach);

            auto count_waiting = [&](MovementType movement) {
                const Direction dir = directionFromApproach(approach);
                const auto& queue = traffic.getQueueByDirection(dir);
                return static_cast<std::size_t>(std::count_if(queue.begin(), queue.end(), [&](const Vehicle& vehicle) {
                    if (!vehicle.isWaiting()) {
                        return false;
                    }
                    switch (movement) {
                        case MovementType::Straight:
                            return vehicle.movement == MovementType::Straight;
                        case MovementType::Left:
                            return isEffectiveLeftTurn(dir, vehicle);
                        case MovementType::Right:
                            return vehicle.movement == MovementType::Right;
                    }
                    return false;
                }));
            };

            const std::size_t straight_idx = routeIndex(approach, MovementType::Straight);
            const std::size_t left_idx = routeIndex(approach, MovementType::Left);
            const std::size_t right_idx = routeIndex(approach, MovementType::Right);

            if (route_configured[straight_idx]) {
                routes[straight_idx] = {approach,
                                        MovementType::Straight,
                                        has_straight_waiting_demand[idx],
                                        count_waiting(MovementType::Straight),
                                        straight_wait_seconds[idx]};
            }
            if (route_configured[left_idx]) {
                routes[left_idx] = {approach,
                                    MovementType::Left,
                                    has_left_waiting_demand[idx],
                                    count_waiting(MovementType::Left),
                                    left_wait_seconds[idx]};
            }
            if (route_configured[right_idx]) {
                routes[right_idx] = {approach,
                                     MovementType::Right,
                                     has_right_waiting_demand[idx],
                                     count_waiting(MovementType::Right),
                                     right_wait_seconds[idx]};
            }

            route_waiting_demand[straight_idx] = routes[straight_idx].waiting_demand;
            route_waiting_demand[left_idx] = routes[left_idx].waiting_demand;
            route_waiting_demand[right_idx] = routes[right_idx].waiting_demand;
            route_wait_seconds[straight_idx] = routes[straight_idx].wait_seconds;
            route_wait_seconds[left_idx] = routes[left_idx].wait_seconds;
            route_wait_seconds[right_idx] = routes[right_idx].wait_seconds;
        }

        // Count crossing vehicles per route (vehicles that started crossing but haven't exited)
        route_crossing_vehicle_count.fill(0);
        for (int approach_int = 0; approach_int < 4; ++approach_int) {
            ApproachId approach = static_cast<ApproachId>(approach_int);
            const Direction dir = directionFromApproach(approach);
            const auto& queue = traffic.getQueueByDirection(dir);

            auto count_crossing = [&](MovementType movement) {
                return static_cast<int>(std::count_if(queue.begin(), queue.end(), [&](const Vehicle& vehicle) {
                    if (!vehicle.isCrossing()) {
                        return false;
                    }
                    switch (movement) {
                        case MovementType::Straight:
                            return vehicle.movement == MovementType::Straight;
                        case MovementType::Left:
                            return isEffectiveLeftTurn(dir, vehicle);
                        case MovementType::Right:
                            return vehicle.movement == MovementType::Right;
                    }
                    return false;
                }));
            };

            const std::size_t s_idx = routeIndex(approach, MovementType::Straight);
            const std::size_t l_idx = routeIndex(approach, MovementType::Left);
            const std::size_t r_idx = routeIndex(approach, MovementType::Right);
            if (route_configured[s_idx])
                route_crossing_vehicle_count[s_idx] = count_crossing(MovementType::Straight);
            if (route_configured[l_idx])
                route_crossing_vehicle_count[l_idx] = count_crossing(MovementType::Left);
            if (route_configured[r_idx])
                route_crossing_vehicle_count[r_idx] = count_crossing(MovementType::Right);
        }

        // Helper: check whether all conflicting routes have no crossing vehicles remaining
        auto areConflictsClear = [&](std::size_t route_idx) -> bool {
            for (std::size_t other = 0; other < kRouteCount; ++other) {
                if (other == route_idx)
                    continue;
                if (!route_configured[other])
                    continue;
                if (!route_conflict_matrix[route_idx][other])
                    continue;
                if (route_crossing_vehicle_count[other] > 0)
                    return false;
            }
            return true;
        };

        // Track when conflicts first became clear per route,
        // and require an extra 2-second buffer before activation.
        constexpr double kClearanceBufferSeconds = 2.0;
        for (std::size_t ri = 0; ri < kRouteCount; ++ri) {
            if (!route_configured[ri])
                continue;
            if (areConflictsClear(ri)) {
                if (route_conflicts_cleared_at[ri] < 0.0) {
                    route_conflicts_cleared_at[ri] = current_time;
                }
            } else {
                route_conflicts_cleared_at[ri] = -1.0;
            }
        }

        auto areConflictsClearWithBuffer = [&](std::size_t route_idx) -> bool {
            if (route_conflicts_cleared_at[route_idx] < 0.0)
                return false;
            return (current_time - route_conflicts_cleared_at[route_idx]) >= kClearanceBufferSeconds;
        };

        int anchor_route_index = -1;
        double anchor_score = -1.0;
        for (std::size_t route_idx = 0; route_idx < routes.size(); ++route_idx) {
            if (!route_configured[route_idx]) {
                continue;
            }
            const auto& route = routes[route_idx];
            if (!route.waiting_demand) {
                continue;
            }

            const double aging_seconds = route_last_served_time[route_idx] >= 0.0
                                             ? std::max(0.0, current_time - route_last_served_time[route_idx])
                                             : movement_starvation_max_wait_seconds;
            const double demand_pressure = static_cast<double>(route.demand_count);
            double score = route_priority_wait_weight * route.wait_seconds +
                           route_priority_queue_weight * demand_pressure + route_priority_aging_weight * aging_seconds +
                           (0.75 * route.wait_seconds * demand_pressure);

            constexpr double kMinimumGreenLockBonus = 1'000'000.0;
            constexpr double kServiceContinuationBonus = 25'000.0;
            constexpr double kStarvationBonus = 20'000.0;

            int conflicting_waiting_routes = 0;
            for (std::size_t other_idx = 0; other_idx < routes.size(); ++other_idx) {
                if (other_idx == route_idx) {
                    continue;
                }
                if (!route_configured[other_idx] || !routes[other_idx].waiting_demand) {
                    continue;
                }
                if (route_conflict_matrix[route_idx][other_idx]) {
                    ++conflicting_waiting_routes;
                }
            }

            if (prev_route_green_active[route_idx]) {
                const double green_started_at =
                    route_green_started_at[route_idx] >= 0.0 ? route_green_started_at[route_idx] : current_time;
                const double green_elapsed = std::max(0.0, current_time - green_started_at);
                if (green_elapsed < minimum_green_seconds) {
                    score += kMinimumGreenLockBonus;
                } else {
                    const double started_this_green = static_cast<double>(route_vehicles_started_this_green[route_idx]);
                    const double remaining_queue_estimate = std::max(0.0, demand_pressure - started_this_green);
                    double continuation_bonus = 4000.0 + std::min(26000.0, remaining_queue_estimate * 3200.0);
                    continuation_bonus -= static_cast<double>(conflicting_waiting_routes) * 3500.0;

                    if (conflicting_waiting_routes > 0 && green_elapsed >= route_max_green_seconds) {
                        continuation_bonus -= 9000.0;
                    }

                    score += std::max(0.0, continuation_bonus);
                }

                // Initial-waiters lock: keep green long enough so all vehicles that were
                // waiting when this route turned green have started crossing.
                if (route_initial_waiting_count[route_idx] > 0 &&
                    route_vehicles_started_this_green[route_idx] < route_initial_waiting_count[route_idx] &&
                    green_elapsed < 2.0 * route_max_green_seconds) {
                    constexpr double kInitialWaitersLockBonus = 500'000.0;
                    score += kInitialWaitersLockBonus;
                }
            }

            if (route.wait_seconds >= movement_starvation_max_wait_seconds) {
                score += kStarvationBonus;
            }

            route_priority_score[route_idx] = score;

            if (score > anchor_score) {
                anchor_score = score;
                anchor_route_index = static_cast<int>(route_idx);
            }
        }

        if (anchor_route_index >= 0) {
            scheduler_anchor_route_index = anchor_route_index;

            IntersectionState override_state = effective_light_state;

            for (std::size_t route_idx = 0; route_idx < routes.size(); ++route_idx) {
                if (!route_configured[route_idx]) {
                    continue;
                }
                if (!route_conflict_matrix[static_cast<std::size_t>(anchor_route_index)][route_idx]) {
                    continue;
                }
                scheduler_blocked_routes[route_idx] = true;
                setRouteLight(override_state, routes[route_idx].approach, routes[route_idx].movement, LightState::Red);
            }

            // Clearance gate: a new anchor can only go green when all conflicting
            // routes have no crossing vehicles remaining in the intersection.
            const bool anchor_already_green = prev_route_green_active[static_cast<std::size_t>(anchor_route_index)];
            const bool conflicts_clear = areConflictsClearWithBuffer(static_cast<std::size_t>(anchor_route_index));
            const bool can_activate_anchor = anchor_already_green || conflicts_clear;
            scheduler_clearance_blocked_routes[static_cast<std::size_t>(anchor_route_index)] = !can_activate_anchor;

            if (can_activate_anchor) {
                setRouteLight(override_state,
                              routes[static_cast<std::size_t>(anchor_route_index)].approach,
                              routes[static_cast<std::size_t>(anchor_route_index)].movement,
                              LightState::Green);

                if (!checker.isSafe(override_state)) {
                    scheduler_safety_blocked_routes[static_cast<std::size_t>(anchor_route_index)] = true;
                } else {
                    effective_light_state = override_state;
                }

                std::vector<std::size_t> parallel_candidates;
                parallel_candidates.reserve(routes.size());
                for (std::size_t route_idx = 0; route_idx < routes.size(); ++route_idx) {
                    if (!route_configured[route_idx]) {
                        continue;
                    }
                    if (static_cast<int>(route_idx) == anchor_route_index) {
                        continue;
                    }
                    if (!routes[route_idx].waiting_demand) {
                        continue;
                    }
                    if (route_conflict_matrix[static_cast<std::size_t>(anchor_route_index)][route_idx]) {
                        continue;
                    }
                    parallel_candidates.push_back(route_idx);
                }

                std::sort(
                    parallel_candidates.begin(), parallel_candidates.end(), [&](std::size_t lhs, std::size_t rhs) {
                        const double lhs_score = route_priority_score[lhs];
                        const double rhs_score = route_priority_score[rhs];
                        return lhs_score > rhs_score;
                    });

                std::vector<std::size_t> selected_parallel;
                for (std::size_t candidate_idx : parallel_candidates) {
                    bool conflicts = false;
                    for (std::size_t accepted_idx : selected_parallel) {
                        if (route_conflict_matrix[candidate_idx][accepted_idx]) {
                            conflicts = true;
                            break;
                        }
                    }
                    if (conflicts) {
                        continue;
                    }

                    IntersectionState trial_state = override_state;
                    setRouteLight(
                        trial_state, routes[candidate_idx].approach, routes[candidate_idx].movement, LightState::Green);

                    if (!checker.isSafe(trial_state)) {
                        scheduler_safety_blocked_routes[candidate_idx] = true;
                        continue;
                    }

                    override_state = trial_state;
                    selected_parallel.push_back(candidate_idx);
                    scheduler_parallel_routes[candidate_idx] = true;
                }

                if (checker.isSafe(override_state)) {
                    effective_light_state = override_state;
                }
            } else {
                // Conflicts not yet cleared: apply Red states to help drain crossing vehicles
                if (checker.isSafe(override_state)) {
                    effective_light_state = override_state;
                }
            }
        }

        auto apply_minimum_green_hold = [&](std::size_t index, LightState& current_light, LightState previous_light) {
            const bool was_green = previous_light == LightState::Green;
            const bool is_green = current_light == LightState::Green;

            if (!was_green && is_green) {
                minimum_green_hold_until_seconds[index] = current_time + minimum_green_seconds;
                return;
            }

            if (was_green && !is_green && current_time < minimum_green_hold_until_seconds[index]) {
                current_light = LightState::Green;
            }
        };

        auto apply_transition_discipline =
            [&](std::size_t index, LightState& current_light, LightState previous_light) {
                const bool was_orange = previous_light == LightState::Orange;

                if (previous_light == LightState::Red && current_light == LightState::Orange) {
                    current_light = LightState::Red;
                    return;
                }

                if (previous_light == LightState::Green && current_light == LightState::Red) {
                    current_light = LightState::Orange;
                }

                const bool is_orange = current_light == LightState::Orange;
                if (!was_orange && is_orange) {
                    minimum_orange_hold_until_seconds[index] = current_time + SafetyChecker::ORANGE_DURATION;
                    return;
                }

                if (was_orange) {
                    const double hold_until = minimum_orange_hold_until_seconds[index];
                    if (current_time < hold_until) {
                        current_light = LightState::Orange;
                        return;
                    }

                    if (current_light == LightState::Green) {
                        current_light = LightState::Red;
                    }
                }
            };

        apply_transition_discipline(0, effective_light_state.north, prev_effective.north);
        apply_transition_discipline(1, effective_light_state.east, prev_effective.east);
        apply_transition_discipline(2, effective_light_state.south, prev_effective.south);
        apply_transition_discipline(3, effective_light_state.west, prev_effective.west);
        apply_transition_discipline(4, effective_light_state.turnSouthEast, prev_effective.turnSouthEast);
        apply_transition_discipline(5, effective_light_state.turnNorthWest, prev_effective.turnNorthWest);
        apply_transition_discipline(6, effective_light_state.turnWestSouth, prev_effective.turnWestSouth);
        apply_transition_discipline(7, effective_light_state.turnEastNorth, prev_effective.turnEastNorth);
        apply_transition_discipline(8, effective_light_state.turnNorthEast, prev_effective.turnNorthEast);
        apply_transition_discipline(9, effective_light_state.turnSouthWest, prev_effective.turnSouthWest);
        apply_transition_discipline(10, effective_light_state.turnEastSouth, prev_effective.turnEastSouth);
        apply_transition_discipline(11, effective_light_state.turnWestNorth, prev_effective.turnWestNorth);

        apply_minimum_green_hold(0, effective_light_state.north, prev_effective.north);
        apply_minimum_green_hold(1, effective_light_state.east, prev_effective.east);
        apply_minimum_green_hold(2, effective_light_state.south, prev_effective.south);
        apply_minimum_green_hold(3, effective_light_state.west, prev_effective.west);
        apply_minimum_green_hold(4, effective_light_state.turnSouthEast, prev_effective.turnSouthEast);
        apply_minimum_green_hold(5, effective_light_state.turnNorthWest, prev_effective.turnNorthWest);
        apply_minimum_green_hold(6, effective_light_state.turnWestSouth, prev_effective.turnWestSouth);
        apply_minimum_green_hold(7, effective_light_state.turnEastNorth, prev_effective.turnEastNorth);
        apply_minimum_green_hold(8, effective_light_state.turnNorthEast, prev_effective.turnNorthEast);
        apply_minimum_green_hold(9, effective_light_state.turnSouthWest, prev_effective.turnSouthWest);
        apply_minimum_green_hold(10, effective_light_state.turnEastSouth, prev_effective.turnEastSouth);
        apply_minimum_green_hold(11, effective_light_state.turnWestNorth, prev_effective.turnWestNorth);

        for (std::size_t route_idx = 0; route_idx < routes.size(); ++route_idx) {
            if (!route_configured[route_idx]) {
                route_green_active[route_idx] = false;
                continue;
            }

            const bool is_green =
                routeIsGreen(effective_light_state, routes[route_idx].approach, routes[route_idx].movement);
            const bool was_green = prev_route_green_active[route_idx];
            route_green_active[route_idx] = is_green;

            if (!was_green && is_green) {
                route_green_started_at[route_idx] = current_time;
                route_vehicles_started_this_green[route_idx] = 0;
                route_initial_waiting_count[route_idx] = static_cast<int>(routes[route_idx].demand_count);
            } else if (was_green && !is_green) {
                route_last_served_time[route_idx] = current_time;
                route_green_started_at[route_idx] = -1.0;
                route_vehicles_started_this_green[route_idx] = 0;
                route_initial_waiting_count[route_idx] = 0;
            }
        }

        previous_effective_light_state = effective_light_state;
        has_previous_effective_light_state = true;
    }

    void SimulatorEngine::generateTraffic(double dt) {
        traffic.generateTraffic(dt, current_time);
    }

    void SimulatorEngine::processVehicleCrossings() {
        // Start crossing when vehicle has reached stopzone.
        // Multiple vehicles in the same lane are allowed, but only with safe headway.
        const double STOP_TARGET = 69.5;
        const double MIN_SAME_LANE_HEADWAY_SECONDS = 0.7;

        for (int dir = 0; dir < 4; ++dir) {
            Direction lane = static_cast<Direction>(dir);
            auto& queue = traffic.getQueueByDirection(lane);
            for (size_t i = 0; i < queue.size(); ++i) {
                auto& vehicle = queue[i];
                if (!vehicle.isWaiting() || vehicle.position_in_lane < STOP_TARGET) {
                    continue;
                }

                bool blocked_by_same_lane_front = false;
                for (size_t j = 0; j < i; ++j) {
                    const auto& ahead = queue[j];
                    if (ahead.lane_id != vehicle.lane_id) {
                        continue;
                    }

                    if (ahead.isWaiting()) {
                        blocked_by_same_lane_front = true;
                        break;
                    }

                    if (ahead.isCrossing()) {
                        const double elapsed = std::max(0.0, current_time - ahead.crossing_time);
                        if (elapsed < MIN_SAME_LANE_HEADWAY_SECONDS) {
                            blocked_by_same_lane_front = true;
                            break;
                        }
                    }
                }
                if (blocked_by_same_lane_front) {
                    continue;
                }

                const LaneConfig* lane_cfg = findLaneConfigForVehicle(intersection_config, lane, vehicle.lane_id);
                bool connected = lane_cfg ? lane_cfg->connected_to_intersection : true;
                bool has_traffic_light = lane_cfg ? lane_cfg->has_traffic_light : true;

                if (!connected) {
                    continue;
                }

                bool can_cross = signalAllowsVehicle(lane, vehicle, lane_cfg, effective_light_state);
                if (can_cross) {
                    const ApproachId approach = approachFromDirection(lane);
                    MovementType route_movement = vehicle.movement;
                    if (isEffectiveLeftTurn(lane, vehicle)) {
                        route_movement = MovementType::Left;
                    }
                    const std::size_t route_idx = routeIndex(approach, route_movement);
                    if (route_idx < route_vehicles_started_this_green.size() && route_configured[route_idx] &&
                        route_green_active[route_idx]) {
                        route_vehicles_started_this_green[route_idx] += 1;
                    }
                    vehicle.crossing_time = current_time;
                }
            }
        }
    }

    void SimulatorEngine::completeVehicleCrossings() {
        // Complete all vehicles that have finished crossing, regardless of queue position.
        for (int dir = 0; dir < 4; ++dir) {
            Direction lane = static_cast<Direction>(dir);
            const auto& queue = traffic.getQueueByDirection(lane);
            std::vector<uint32_t> finished_ids;
            finished_ids.reserve(queue.size());

            for (const auto& vehicle : queue) {
                if (!vehicle.isCrossing()) {
                    continue;
                }

                const double crossing_time = vehicle.getCrossingDuration(queue.size());
                if (current_time - vehicle.crossing_time >= crossing_time) {
                    finished_ids.push_back(vehicle.id);
                }
            }

            for (uint32_t id : finished_ids) {
                traffic.completeCrossing(id, current_time);
            }
        }
    }

    void SimulatorEngine::advanceController(double dt) {
        controller->tick(dt);
    }

    IntersectionState SimulatorEngine::getCurrentLightState() const {
        return controller ? controller->getCurrentState() : IntersectionState{};
    }

    SimulatorMetrics SimulatorEngine::getMetrics() const {
        SimulatorMetrics metrics;
        metrics.total_time = current_time;
        metrics.vehicles_generated = traffic.getTotalGenerated();
        metrics.vehicles_crossed = traffic.getTotalCrossed();
        metrics.average_wait_time = traffic.getAverageWaitTime();
        metrics.queue_lengths = {traffic.getQueueLength(Direction::North),
                                 traffic.getQueueLength(Direction::East),
                                 traffic.getQueueLength(Direction::South),
                                 traffic.getQueueLength(Direction::West)};
        metrics.total_queue_length =
            metrics.queue_lengths[0] + metrics.queue_lengths[1] + metrics.queue_lengths[2] + metrics.queue_lengths[3];
        metrics.safety_violations = safety_violations;
        return metrics;
    }

    SimulatorSnapshot SimulatorEngine::getSnapshot() const {
        SimulatorSnapshot snapshot;
        snapshot.sim_time = current_time;
        snapshot.running = running;
        snapshot.metrics = getMetrics();
        snapshot.lights = effective_light_state;
        return snapshot;
    }

    namespace {
        const char* toString(LightState state) {
            switch (state) {
                case LightState::Red:
                    return "red";
                case LightState::Orange:
                    return "orange";
                case LightState::Green:
                    return "green";
            }
            return "red";
        }

        const char* toString(MovementType movement) {
            switch (movement) {
                case MovementType::Straight:
                    return "straight";
                case MovementType::Left:
                    return "left";
                case MovementType::Right:
                    return "right";
            }
            return "straight";
        }

        const char* toString(ApproachId approach) {
            switch (approach) {
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
    }  // namespace

    std::string SimulatorEngine::getSnapshotJson() const {
        auto northVehicles = traffic.getLaneVehicleStates(Direction::North);
        auto eastVehicles = traffic.getLaneVehicleStates(Direction::East);
        auto southVehicles = traffic.getLaneVehicleStates(Direction::South);
        auto westVehicles = traffic.getLaneVehicleStates(Direction::West);

        auto appendLaneVehicles = [](std::ostringstream& out, const std::vector<LaneVehicleState>& vehicles) {
            out << "[";
            for (size_t i = 0; i < vehicles.size(); ++i) {
                const auto& v = vehicles[i];
                if (i > 0) {
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
        out << "\"scheduler\":{";
        out << "\"wmax_seconds\":" << movement_starvation_max_wait_seconds << ",";
        out << "\"anchor_route\":";
        if (scheduler_anchor_route_index >= 0 &&
            route_configured[static_cast<std::size_t>(scheduler_anchor_route_index)]) {
            const std::size_t anchor_idx = static_cast<std::size_t>(scheduler_anchor_route_index);
            ApproachId anchor_approach = static_cast<ApproachId>(anchor_idx / 3);
            MovementType anchor_movement = static_cast<MovementType>(anchor_idx % 3);
            out << "\"" << routeName(anchor_approach, anchor_movement) << "\"";
        } else {
            out << "\"\"";
        }
        out << ",";
        out << "\"routes\":[";
        auto route_activation_reason = [&](std::size_t idx) {
            if (!route_waiting_demand[idx]) {
                return "no_demand";
            }

            if (route_green_active[idx]) {
                return "green_active";
            }

            if (scheduler_blocked_routes[idx]) {
                return "blocked_by_anchor";
            }

            if (scheduler_safety_blocked_routes[idx]) {
                return "blocked_by_safety";
            }

            bool conflicting_green_active = false;
            for (std::size_t other_idx = 0; other_idx < route_green_active.size(); ++other_idx) {
                if (other_idx == idx || !route_configured[other_idx]) {
                    continue;
                }
                if (route_conflict_matrix[idx][other_idx] && route_green_active[other_idx]) {
                    conflicting_green_active = true;
                    break;
                }
            }
            if (conflicting_green_active) {
                return "waiting_conflict_clearance";
            }

            const bool selected =
                (scheduler_anchor_route_index >= 0 && static_cast<std::size_t>(scheduler_anchor_route_index) == idx) ||
                scheduler_parallel_routes[idx];
            if (selected) {
                return "selected_pending_transition";
            }

            if (scheduler_anchor_route_index >= 0) {
                return "waiting_scheduler_turn";
            }

            return "waiting_controller_phase";
        };

        bool first_route = true;
        for (int approach_int = 0; approach_int < 4; ++approach_int) {
            ApproachId approach = static_cast<ApproachId>(approach_int);
            for (int movement_int = 0; movement_int < 3; ++movement_int) {
                MovementType movement = static_cast<MovementType>(movement_int);
                const std::size_t idx = routeIndex(approach, movement);
                if (!route_configured[idx]) {
                    continue;
                }
                if (!first_route) {
                    out << ",";
                }
                first_route = false;

                out << "{";
                out << "\"route\":\"" << routeName(approach, movement) << "\",";
                out << "\"approach\":\"" << toString(approach) << "\",";
                out << "\"movement\":\"" << toString(movement) << "\",";
                out << "\"waiting_demand\":" << (route_waiting_demand[idx] ? "true" : "false") << ",";
                out << "\"wait_seconds\":" << route_wait_seconds[idx] << ",";
                out << "\"priority_score\":" << route_priority_score[idx] << ",";
                out << "\"green_active\":" << (route_green_active[idx] ? "true" : "false") << ",";
                const bool selected = (scheduler_anchor_route_index >= 0 &&
                                       static_cast<std::size_t>(scheduler_anchor_route_index) == idx) ||
                                      scheduler_parallel_routes[idx];
                out << "\"selected\":" << (selected ? "true" : "false") << ",";
                out << "\"parallel_active\":" << (scheduler_parallel_routes[idx] ? "true" : "false") << ",";
                out << "\"blocked_by_anchor\":" << (scheduler_blocked_routes[idx] ? "true" : "false") << ",";
                out << "\"blocked_by_safety\":" << (scheduler_safety_blocked_routes[idx] ? "true" : "false") << ",";
                out << "\"blocked_by_clearance\":" << (scheduler_clearance_blocked_routes[idx] ? "true" : "false")
                    << ",";
                out << "\"vehicles_started_this_green\":" << route_vehicles_started_this_green[idx] << ",";
                out << "\"initial_waiting_count\":" << route_initial_waiting_count[idx] << ",";
                out << "\"crossing_vehicle_count\":" << route_crossing_vehicle_count[idx] << ",";
                out << "\"activation_reason\":\"" << route_activation_reason(idx) << "\",";
                out << "\"conflicts\":[";

                bool first_conflict = true;
                for (int other_approach_int = 0; other_approach_int < 4; ++other_approach_int) {
                    ApproachId other_approach = static_cast<ApproachId>(other_approach_int);
                    for (int other_movement_int = 0; other_movement_int < 3; ++other_movement_int) {
                        MovementType other_movement = static_cast<MovementType>(other_movement_int);
                        const std::size_t other_idx = routeIndex(other_approach, other_movement);
                        if (!route_configured[other_idx]) {
                            continue;
                        }
                        if (!route_conflict_matrix[idx][other_idx]) {
                            continue;
                        }
                        if (!first_conflict) {
                            out << ",";
                        }
                        first_conflict = false;
                        out << "\"" << routeName(other_approach, other_movement) << "\"";
                    }
                }

                out << "]";
                out << "}";
            }
        }
        out << "]";
        out << "},";
        out << "\"spawn\":{";
        out << "\"rate\":" << traffic.getArrivalRate() << ",";
        auto spawn_filter = traffic.getSpawnLaneFilter();
        if (spawn_filter.has_value()) {
            out << "\"focus\":{";
            out << "\"active\":true,";
            out << "\"approach\":\"" << toString(spawn_filter->approach) << "\",";
            out << "\"lane_index\":" << spawn_filter->lane_index;
            out << "}";
        } else {
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

    void SimulatorEngine::reset() {
        current_time = 0.0;
        running = false;
        safety_violations = 0;
        traffic.reset();
        setControlMode(ControlMode::Basic);
        right_turn_green_hold_until = {0.0, 0.0, 0.0, 0.0};
        left_wait_seconds = {0.0, 0.0, 0.0, 0.0};
        straight_wait_seconds = {0.0, 0.0, 0.0, 0.0};
        right_wait_seconds = {0.0, 0.0, 0.0, 0.0};
        route_waiting_demand.fill(false);
        route_wait_seconds.fill(0.0);
        route_priority_score.fill(0.0);
        route_green_active.fill(false);
        route_last_served_time.fill(-1.0);
        route_green_started_at.fill(-1.0);
        route_vehicles_started_this_green.fill(0);
        route_initial_waiting_count.fill(0);
        route_crossing_vehicle_count.fill(0);
        route_conflicts_cleared_at.fill(-1.0);
        scheduler_anchor_route_index = -1;
        scheduler_parallel_routes.fill(false);
        scheduler_blocked_routes.fill(false);
        scheduler_safety_blocked_routes.fill(false);
        scheduler_clearance_blocked_routes.fill(false);
        scheduler_served_this_cycle.fill(false);
        minimum_green_hold_until_seconds.fill(0.0);
        minimum_orange_hold_until_seconds.fill(0.0);
        previous_effective_light_state = IntersectionState{};
        has_previous_effective_light_state = false;
        refreshEffectiveSignalState(0.0);
        previous_controller_state = controller ? controller->getCurrentState() : IntersectionState{};
        has_previous_controller_state = true;
    }

    void SimulatorEngine::start() {
        running = true;
    }

    void SimulatorEngine::stop() {
        running = false;
    }

    bool SimulatorEngine::isRunning() const {
        return running;
    }

    void SimulatorEngine::handleCommand(UICommand command, double dt) {
        switch (command) {
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
                if (!running) {
                    start();
                    tick(dt);
                    stop();
                } else {
                    tick(dt);
                }
                break;
        }
    }

    void SimulatorEngine::setControlMode(ControlMode mode) {
        control_mode = mode;

        if (control_mode == ControlMode::Basic) {
            if (intersection_config.signal_groups.empty()) {
                controller = std::make_unique<BasicControllerAdapter>(ns_duration, ew_duration);
            } else {
                controller = std::make_unique<ConfigurableSignalGroupController>(intersection_config);
            }
        } else {
            controller = std::make_unique<NullControlController>();
        }

        controller->reset();
        previous_controller_state = controller ? controller->getCurrentState() : IntersectionState{};
        has_previous_controller_state = true;
    }

    SimulatorEngine::ControlMode SimulatorEngine::getControlMode() const {
        return control_mode;
    }

    void SimulatorEngine::setController(std::unique_ptr<ITrafficLightController> custom_controller, ControlMode mode) {
        control_mode = mode;
        controller = std::move(custom_controller);
        if (controller) {
            controller->reset();
        }
        previous_controller_state = controller ? controller->getCurrentState() : IntersectionState{};
        has_previous_controller_state = true;
    }

    const IntersectionConfig& SimulatorEngine::getIntersectionConfig() const {
        return intersection_config;
    }

    void SimulatorEngine::setSpawnLaneFilter(const std::optional<TrafficGenerator::SpawnLaneFilter>& filter) {
        traffic.setSpawnLaneFilter(filter);
    }

    std::optional<TrafficGenerator::SpawnLaneFilter> SimulatorEngine::getSpawnLaneFilter() const {
        return traffic.getSpawnLaneFilter();
    }

    void SimulatorEngine::setTrafficRate(double rate) {
        traffic.setArrivalRate(rate);
    }

    double SimulatorEngine::getTrafficRate() const {
        return traffic.getArrivalRate();
    }

    bool SimulatorEngine::isLightGreen(Direction dir) const {
        auto state = getCurrentLightState();
        switch (dir) {
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
                                              const Vehicle& vehicle,
                                              const LaneConfig* lane_cfg,
                                              const IntersectionState& state) const {
        const bool has_traffic_light = lane_cfg ? lane_cfg->has_traffic_light : true;
        const bool effective_left_turn = isEffectiveLeftTurn(dir, vehicle);

        if (lane_cfg && !lane_cfg->allowed_movements.empty()) {
            const bool movement_allowed =
                std::find(lane_cfg->allowed_movements.begin(), lane_cfg->allowed_movements.end(), vehicle.movement) !=
                lane_cfg->allowed_movements.end();
            if (!movement_allowed) {
                return false;
            }
        }

        if (!has_traffic_light) {
            if (effective_left_turn) {
                return false;
            }
            return true;
        }

        bool main_green = false;
        switch (dir) {
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

        switch (vehicle.movement) {
            case MovementType::Straight:
                return main_green;
            case MovementType::Left:
                if (dedicated_left) {
                    return left_turn_green;
                }
                return main_green;
            case MovementType::Right:
                if (dedicated_right) {
                    return turn_green;
                }
                return main_green || turn_green;
        }

        return false;
    }

    std::vector<SignalGroupId> SimulatorEngine::resolveActiveSignalGroups(const IntersectionState& state) const {
        auto is_active = [](LightState value) { return value == LightState::Green || value == LightState::Orange; };

        auto is_movement_active = [&](ApproachId approach, MovementType movement) {
            switch (movement) {
                case MovementType::Straight:
                    return is_active(approachMainLight(approach, state));
                case MovementType::Left:
                    switch (approach) {
                        case ApproachId::North:
                            return is_active(state.turnNorthEast);
                        case ApproachId::South:
                            return is_active(state.turnSouthWest);
                        case ApproachId::East:
                            return is_active(state.turnEastSouth);
                        case ApproachId::West:
                            return is_active(state.turnWestNorth);
                    }
                    return false;
                case MovementType::Right:
                    switch (approach) {
                        case ApproachId::North:
                            return is_active(state.turnNorthWest);
                        case ApproachId::South:
                            return is_active(state.turnSouthEast);
                        case ApproachId::East:
                            return is_active(state.turnEastNorth);
                        case ApproachId::West:
                            return is_active(state.turnWestSouth);
                    }
                    return false;
            }
            return false;
        };

        auto findApproachForLane = [&](LaneId lane_id, ApproachId& approach_out) {
            for (const auto& approach : intersection_config.approaches) {
                auto lane_it = std::find_if(approach.lanes.begin(),
                                            approach.lanes.end(),
                                            [lane_id](const LaneConfig& lane) { return lane.id == lane_id; });
                if (lane_it != approach.lanes.end()) {
                    approach_out = approach.id;
                    return true;
                }
            }
            return false;
        };

        std::unordered_set<SignalGroupId> active_ids;

        for (const auto& group : intersection_config.signal_groups) {
            bool group_active = false;
            for (LaneId lane_id : group.controlled_lanes) {
                ApproachId lane_approach;
                if (!findApproachForLane(lane_id, lane_approach)) {
                    continue;
                }

                for (MovementType movement : group.green_movements) {
                    if (is_movement_active(lane_approach, movement)) {
                        group_active = true;
                        break;
                    }
                }

                if (group_active) {
                    break;
                }
            }

            if (group_active) {
                active_ids.insert(group.id);
            }
        }

        return std::vector<SignalGroupId>(active_ids.begin(), active_ids.end());
    }

    bool SimulatorEngine::isConfigSignalStateSafe(const IntersectionState& state) const {
        if (intersection_config.signal_groups.empty()) {
            return true;
        }

        if (!checker.isConfigValid()) {
            return false;
        }

        std::vector<SignalGroupId> active_groups = resolveActiveSignalGroups(state);
        if (active_groups.empty()) {
            return true;
        }

        return checker.areSignalGroupsConflictFree(active_groups);
    }

}  // namespace crossroads