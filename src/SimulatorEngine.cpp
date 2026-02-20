#include "SimulatorEngine.hpp"
#include <array>
#include <sstream>
#include <iostream>
#include <utility>

namespace crossroads
{
    SimulatorEngine::SimulatorEngine(double traffic_rate, double ns_duration, double ew_duration)
        : control_mode(ControlMode::Basic),
          traffic(traffic_rate),
          ns_duration(ns_duration),
          ew_duration(ew_duration),
          current_time(0.0),
          safety_violations(0)
    {
        controller = std::make_unique<BasicControllerAdapter>(ns_duration, ew_duration);
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

        generateTraffic(dt);
        traffic.updateVehicleSpeeds(dt);
        processVehicleCrossings();
        completeVehicleCrossings();
        advanceController(dt);

        if (!checker.isSafe(getCurrentLightState()))
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
    }

    std::string SimulatorEngine::getSnapshotJson() const
    {
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
            controller = std::make_unique<BasicControllerAdapter>(ns_duration, ew_duration);
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