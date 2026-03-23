#pragma once

#include <array>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "IntersectionConfig.hpp"
#include "SafetyChecker.hpp"
#include "TrafficGenerator.hpp"
#include "TrafficLightControllers.hpp"

namespace crossroads {
    struct SimulatorMetrics {
        double total_time = 0.0;
        size_t vehicles_generated = 0;
        size_t vehicles_crossed = 0;
        double average_wait_time = 0.0;
        std::array<size_t, 4> queue_lengths{};
        size_t total_queue_length = 0;
        size_t safety_violations = 0;
    };

    struct SimulatorSnapshot {
        double sim_time = 0.0;
        bool running = false;
        SimulatorMetrics metrics;
        IntersectionState lights;
    };

    class SimulatorEngine {
       public:
        enum class ControlMode { Basic, NullControl };

        enum class UICommand { Start, Stop, Reset, Step };

        SimulatorEngine(double traffic_rate = 0.5, double ns_duration = 10.0, double ew_duration = 10.0);
        SimulatorEngine(const IntersectionConfig& intersection_config,
                        double traffic_rate,
                        double ns_duration,
                        double ew_duration);

        void simulate(double duration_seconds, double time_step = 0.1);
        void tick(double dt);
        IntersectionState getCurrentLightState() const;
        SimulatorMetrics getMetrics() const;
        SimulatorSnapshot getSnapshot() const;
        std::string getSnapshotJson() const;
        void reset();
        void start();
        void stop();
        bool isRunning() const;
        void handleCommand(UICommand command, double dt = 0.1);
        void setControlMode(ControlMode mode);
        ControlMode getControlMode() const;
        void setController(std::unique_ptr<ITrafficLightController> custom_controller, ControlMode mode);
        const IntersectionConfig& getIntersectionConfig() const;
        void setSpawnLaneFilter(const std::optional<TrafficGenerator::SpawnLaneFilter>& filter);
        std::optional<TrafficGenerator::SpawnLaneFilter> getSpawnLaneFilter() const;
        void setTrafficRate(double rate);
        double getTrafficRate() const;

       private:
        void generateTraffic(double dt);
        void processVehicleCrossings();
        void completeVehicleCrossings();
        void advanceController(double dt);
        void refreshEffectiveSignalState(double dt_seconds);
        bool signalAllowsVehicle(Direction dir,
                                 const Vehicle& vehicle,
                                 const LaneConfig* lane_cfg,
                                 const IntersectionState& state) const;
        bool isLightGreen(Direction dir) const;
        std::vector<SignalGroupId> resolveActiveSignalGroups(const IntersectionState& state) const;
        bool isConfigSignalStateSafe(const IntersectionState& state) const;

        SafetyChecker checker;
        std::unique_ptr<ITrafficLightController> controller;
        ControlMode control_mode;
        TrafficGenerator traffic;
        double ns_duration;
        double ew_duration;
        IntersectionConfig intersection_config;
        double current_time = 0.0;
        bool running = false;
        size_t safety_violations = 0;
        IntersectionState effective_light_state{};
        IntersectionState previous_effective_light_state{};
        IntersectionState previous_controller_state{};
        bool has_previous_effective_light_state = false;
        bool has_previous_controller_state = false;
        std::array<double, 12> minimum_green_hold_until_seconds{};
        std::array<double, 12> minimum_orange_hold_until_seconds{};
        std::array<double, 4> right_turn_green_hold_until{};
        std::array<double, 4> straight_wait_seconds{};
        std::array<double, 4> left_wait_seconds{};
        std::array<double, 4> right_wait_seconds{};
        std::array<bool, 12> route_waiting_demand{};
        std::array<double, 12> route_wait_seconds{};
        std::array<double, 12> route_priority_score{};
        std::array<bool, 12> route_green_active{};
        std::array<bool, 12> route_configured{};
        std::array<double, 12> route_last_served_time{};
        std::array<double, 12> route_green_started_at{};
        std::array<int, 12> route_vehicles_started_this_green{};
        std::array<int, 12> route_initial_waiting_count{};    // Waiting vehicles when green started
        std::array<int, 12> route_crossing_vehicle_count{};   // Currently crossing per route
        std::array<int, 12> route_stopped_waiting_count{};    // Stopped waiting vehicles per route
        std::array<double, 12> route_conflicts_cleared_at{};  // When conflicts first became clear (-1 = not clear)
        std::array<double, 12> route_red_since{};             // When route last turned red (-1 = not red)
        std::array<std::array<bool, 12>, 12> route_conflict_matrix{};
        bool route_conflict_matrix_ready = false;
        int scheduler_anchor_route_index = -1;
        std::array<bool, 12> scheduler_parallel_routes{};
        std::array<bool, 12> scheduler_blocked_routes{};
        std::array<bool, 12> scheduler_safety_blocked_routes{};
        std::array<bool, 12> scheduler_clearance_blocked_routes{};  // Blocked by crossing vehicles
        std::array<bool, 12> scheduler_served_this_cycle{};         // Routes served in current scheduling cycle
        double minimum_green_seconds = 3.0;
        double right_turn_min_green_seconds = 2.0;
        double straight_starvation_threshold_seconds = 2.0;
        double left_starvation_threshold_seconds = 2.0;
        double movement_starvation_max_wait_seconds = 10.0;
        double route_priority_wait_weight = 2.0;
        double route_priority_queue_weight = 3.0;
        double route_priority_aging_weight = 1.5;
        double route_max_green_seconds = 6.0;
        int route_target_vehicles_per_green = 4;
    };

}  // namespace crossroads