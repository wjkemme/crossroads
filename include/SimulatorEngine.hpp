#pragma once

#include "SafetyChecker.hpp"
#include "BasicLightController.hpp"
#include "TrafficGenerator.hpp"

namespace crossroads
{
    struct SimulatorMetrics
    {
        double total_time = 0.0;
        size_t vehicles_generated = 0;
        size_t vehicles_crossed = 0;
        double average_wait_time = 0.0;
        double queue_length = 0.0;
        size_t safety_violations = 0;
    };

    class SimulatorEngine
    {
    public:
        SimulatorEngine(double traffic_rate = 0.5,
                       double ns_duration = 10.0,
                       double ew_duration = 10.0);

        void simulate(double duration_seconds, double time_step = 0.1);
        void tick(double dt);
        IntersectionState getCurrentLightState() const;
        SimulatorMetrics getMetrics() const;
        void reset();

    private:
        void generateTraffic(double dt);
        void processVehicleCrossings();
        void completeVehicleCrossings();
        void advanceController(double dt);
        bool isLightGreen(Direction dir) const;

        SafetyChecker checker;
        BasicLightController controller;
        TrafficGenerator traffic;
        double current_time = 0.0;
        size_t safety_violations = 0;
    };

} // namespace crossroads