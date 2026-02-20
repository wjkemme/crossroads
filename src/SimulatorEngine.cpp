#include "SimulatorEngine.hpp"
#include <iostream>

namespace crossroads
{
    SimulatorEngine::SimulatorEngine(double traffic_rate, double ns_duration, double ew_duration)
        : controller(ns_duration, ew_duration), traffic(traffic_rate), current_time(0.0), safety_violations(0)
    {
    }

    void SimulatorEngine::simulate(double duration_seconds, double time_step)
    {
        reset();
        while (current_time < duration_seconds)
            tick(time_step);
    }

    void SimulatorEngine::tick(double dt)
    {
        generateTraffic(dt);
        traffic.updateVehicleSpeeds(dt);
        processVehicleCrossings();
        completeVehicleCrossings();
        advanceController(dt);

        if (!checker.isSafe(getCurrentLightState()))
            safety_violations++;

        current_time += dt;
    }

    void SimulatorEngine::generateTraffic(double dt)
    {
        traffic.generateTraffic(dt, current_time);
    }

    void SimulatorEngine::processVehicleCrossings()
    {
        for (int dir = 0; dir < 4; ++dir)
        {
            Direction lane = static_cast<Direction>(dir);
            if (isLightGreen(lane))
            {
                Vehicle *v = traffic.peekNextVehicle(lane);
                if (v && v->isWaiting())
                {
                    traffic.startCrossing(lane, v->id, current_time);
                }
            }
        }
    }

    void SimulatorEngine::completeVehicleCrossings()
    {
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
        controller.tick(dt);
    }

    IntersectionState SimulatorEngine::getCurrentLightState() const
    {
        return controller.getCurrentState();
    }

    SimulatorMetrics SimulatorEngine::getMetrics() const
    {
        SimulatorMetrics metrics;
        metrics.total_time = current_time;
        metrics.vehicles_generated = traffic.getTotalGenerated();
        metrics.vehicles_crossed = traffic.getTotalCrossed();
        metrics.average_wait_time = traffic.getAverageWaitTime();
        metrics.queue_length = traffic.getQueueLength(Direction::North);
        metrics.safety_violations = safety_violations;
        return metrics;
    }

    void SimulatorEngine::reset()
    {
        current_time = 0.0;
        safety_violations = 0;
        traffic.reset();
        controller.reset();
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

} // namespace crossroads