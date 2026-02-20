#pragma once

#include "SafetyChecker.hpp"
#include "TrafficGenerator.hpp"
#include "TrafficLightControllers.hpp"
#include <array>
#include <memory>
#include <string>

namespace crossroads
{
    struct SimulatorMetrics
    {
        double total_time = 0.0;
        size_t vehicles_generated = 0;
        size_t vehicles_crossed = 0;
        double average_wait_time = 0.0;
        std::array<size_t, 4> queue_lengths{};
        size_t total_queue_length = 0;
        size_t safety_violations = 0;
    };

    struct SimulatorSnapshot
    {
        double sim_time = 0.0;
        bool running = false;
        SimulatorMetrics metrics;
        IntersectionState lights;
    };

    class SimulatorEngine
    {
    public:
        enum class ControlMode
        {
            Basic,
            NullControl
        };

        enum class UICommand
        {
            Start,
            Stop,
            Reset,
            Step
        };

        SimulatorEngine(double traffic_rate = 0.5,
                        double ns_duration = 10.0,
                        double ew_duration = 10.0);

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

    private:
        void generateTraffic(double dt);
        void processVehicleCrossings();
        void completeVehicleCrossings();
        void advanceController(double dt);
        bool isLightGreen(Direction dir) const;

        SafetyChecker checker;
        std::unique_ptr<ITrafficLightController> controller;
        ControlMode control_mode;
        TrafficGenerator traffic;
        double ns_duration;
        double ew_duration;
        double current_time = 0.0;
        bool running = false;
        size_t safety_violations = 0;
    };

} // namespace crossroads