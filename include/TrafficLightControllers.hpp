#pragma once

#include "Intersection.hpp"
#include "BasicLightController.hpp"

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
}
