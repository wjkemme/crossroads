#pragma once

#include "Intersection.hpp"
#include "BasicLightController.hpp"
#include "IntersectionConfig.hpp"

#include <algorithm>
#include <unordered_map>
#include <vector>

namespace crossroads
{
    class ITrafficLightController
    {
    public:
        virtual ~ITrafficLightController() = default;
        virtual void tick(double dt_seconds) = 0;
        virtual IntersectionState getCurrentState() const = 0;
        virtual void reset() = 0;
    };

    class BasicControllerAdapter : public ITrafficLightController
    {
    public:
        BasicControllerAdapter(double ns_green_duration, double ew_green_duration)
            : basic_controller(ns_green_duration, ew_green_duration)
        {
        }

        void tick(double dt_seconds) override
        {
            basic_controller.tick(dt_seconds);
        }

        IntersectionState getCurrentState() const override
        {
            return basic_controller.getCurrentState();
        }

        void reset() override
        {
            basic_controller.reset();
        }

    private:
        BasicLightController basic_controller;
    };

    class NullControlController : public ITrafficLightController
    {
    public:
        NullControlController()
            : elapsed(0.0), orange_on(true)
        {
            reset();
        }

        void tick(double dt_seconds) override
        {
            elapsed += dt_seconds;
            while (elapsed >= 1.0)
            {
                elapsed -= 1.0;
                orange_on = !orange_on;
                applyPattern();
            }
        }

        IntersectionState getCurrentState() const override
        {
            return state;
        }

        void reset() override
        {
            elapsed = 0.0;
            orange_on = true;
            applyPattern();
        }

    private:
        void applyPattern()
        {
            LightState active = orange_on ? LightState::Orange : LightState::Red;
            state.north = active;
            state.south = active;
            state.east = active;
            state.west = active;
            state.turnSouthEast = active;
            state.turnNorthWest = active;
            state.turnWestSouth = active;
            state.turnEastNorth = active;
        }

        IntersectionState state{};
        double elapsed;
        bool orange_on;
    };

    class ConfigurableSignalGroupController : public ITrafficLightController
    {
    public:
        explicit ConfigurableSignalGroupController(const IntersectionConfig &config)
            : intersection_config(config), phase_index(0), in_orange(false), phase_elapsed(0.0)
        {
            for (const auto &group : intersection_config.signal_groups)
            {
                phase_order.push_back(group.id);
            }
            rebuildLaneApproachMap();
            reset();
        }

        void tick(double dt_seconds) override
        {
            if (phase_order.empty())
            {
                return;
            }

            phase_elapsed += dt_seconds;
            while (true)
            {
                const SignalGroupConfig *group = currentGroup();
                if (!group)
                {
                    return;
                }

                double phase_duration = in_orange ? group->orange_seconds : group->min_green_seconds;
                if (phase_elapsed < phase_duration)
                {
                    break;
                }

                phase_elapsed -= phase_duration;
                if (!in_orange)
                {
                    in_orange = true;
                }
                else
                {
                    in_orange = false;
                    phase_index = (phase_index + 1) % phase_order.size();
                }
                applyCurrentPhase();
            }
        }

        IntersectionState getCurrentState() const override
        {
            return state;
        }

        void reset() override
        {
            phase_index = 0;
            in_orange = false;
            phase_elapsed = 0.0;
            applyCurrentPhase();
        }

    private:
        void rebuildLaneApproachMap()
        {
            lane_to_approach.clear();
            for (const auto &approach : intersection_config.approaches)
            {
                for (const auto &lane : approach.lanes)
                {
                    lane_to_approach[lane.id] = approach.id;
                }
            }
        }

        const SignalGroupConfig *currentGroup() const
        {
            if (phase_order.empty())
            {
                return nullptr;
            }

            SignalGroupId id = phase_order[phase_index];
            auto it = std::find_if(intersection_config.signal_groups.begin(),
                                   intersection_config.signal_groups.end(),
                                   [id](const SignalGroupConfig &group)
                                   { return group.id == id; });
            return it == intersection_config.signal_groups.end() ? nullptr : &(*it);
        }

        static void applyMovement(IntersectionState &s, ApproachId approach, MovementType movement, LightState color)
        {
            if (movement == MovementType::Right)
            {
                switch (approach)
                {
                case ApproachId::North:
                    s.turnNorthWest = color;
                    return;
                case ApproachId::East:
                    s.turnEastNorth = color;
                    return;
                case ApproachId::South:
                    s.turnSouthEast = color;
                    return;
                case ApproachId::West:
                    s.turnWestSouth = color;
                    return;
                }
            }

            switch (approach)
            {
            case ApproachId::North:
                s.north = color;
                break;
            case ApproachId::East:
                s.east = color;
                break;
            case ApproachId::South:
                s.south = color;
                break;
            case ApproachId::West:
                s.west = color;
                break;
            }
        }

        void applyCurrentPhase()
        {
            state = IntersectionState{};
            const SignalGroupConfig *group = currentGroup();
            if (!group)
            {
                return;
            }

            LightState color = in_orange ? LightState::Orange : LightState::Green;
            for (LaneId lane_id : group->controlled_lanes)
            {
                auto approach_it = lane_to_approach.find(lane_id);
                if (approach_it == lane_to_approach.end())
                {
                    continue;
                }

                for (MovementType movement : group->green_movements)
                {
                    applyMovement(state, approach_it->second, movement, color);
                }
            }
        }

        IntersectionConfig intersection_config;
        std::vector<SignalGroupId> phase_order;
        std::unordered_map<LaneId, ApproachId> lane_to_approach;
        size_t phase_index;
        bool in_orange;
        double phase_elapsed;
        IntersectionState state{};
    };
}
