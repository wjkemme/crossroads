#include "BasicLightController.hpp"

namespace crossroads
{

    BasicLightController::BasicLightController(double ns_green_duration, double ew_green_duration)
        : ns_duration(ns_green_duration), ew_duration(ew_green_duration), phase_elapsed(0.0),
          current_phase(NS_GREEN)
    {
        reset();
    }

    void BasicLightController::reset()
    {
        current_state = IntersectionState{};
        phase_elapsed = 0.0;
        current_phase = NS_GREEN;
        demand_by_direction = {false, false, false, false};
        ns_red_elapsed = 0.0;
        ew_red_elapsed = 0.0;
        applyPhasePattern(NS_GREEN, current_state);
    }

    void BasicLightController::setDemandByDirection(const std::array<bool, 4> &demand)
    {
        demand_by_direction = demand;
    }

    void BasicLightController::applyPhasePattern(Phase phase, IntersectionState &state)
    {
        // Initialize all lights to red
        state.north = LightState::Red;
        state.south = LightState::Red;
        state.east = LightState::Red;
        state.west = LightState::Red;
        state.turnSouthEast = LightState::Red;
        state.turnNorthWest = LightState::Red;
        state.turnWestSouth = LightState::Red;
        state.turnEastNorth = LightState::Red;

        // Apply phase-specific pattern
        switch (phase)
        {
        case NS_GREEN:
            state.north = LightState::Green;
            state.south = LightState::Green;
            break;
        case NS_ORANGE:
            state.north = LightState::Orange;
            state.south = LightState::Orange;
            break;
        case EW_GREEN:
            state.east = LightState::Green;
            state.west = LightState::Green;
            break;
        case EW_ORANGE:
            state.east = LightState::Orange;
            state.west = LightState::Orange;
            break;
        }
    }

    void BasicLightController::transitionToNextPhase()
    {
        IntersectionState next_state = current_state;
        Phase next_phase = current_phase;

        // Determine next phase
        switch (current_phase)
        {
        case NS_GREEN:
            next_phase = NS_ORANGE;
            break;
        case NS_ORANGE:
            next_phase = EW_GREEN;
            break;
        case EW_GREEN:
            next_phase = EW_ORANGE;
            break;
        case EW_ORANGE:
            next_phase = NS_GREEN;
            break;
        }

        // Apply the next phase pattern
        applyPhasePattern(next_phase, next_state);

        // Validate transition with SafetyChecker
        if (checker.isValidTransition(current_state, next_state, SafetyChecker::ORANGE_DURATION))
        {
            current_state = next_state;
            current_phase = next_phase;
            phase_elapsed = 0.0;
        }
        // If validation fails, stay in current phase (shouldn't happen if logic is correct)
    }

    void BasicLightController::tick(double dt_seconds)
    {
        phase_elapsed += dt_seconds;
        updateRedTimers(dt_seconds);

        const bool ns_demand = demand_by_direction[0] || demand_by_direction[1];
        const bool ew_demand = demand_by_direction[2] || demand_by_direction[3];

        // Get phase duration
        double phase_duration = 0.0;
        switch (current_phase)
        {
        case NS_GREEN:
            phase_duration = ns_duration;
            break;
        case NS_ORANGE:
            phase_duration = SafetyChecker::ORANGE_DURATION;
            break;
        case EW_GREEN:
            phase_duration = ew_duration;
            break;
        case EW_ORANGE:
            phase_duration = SafetyChecker::ORANGE_DURATION;
            break;
        }

        const bool hold_current_green = (current_phase == NS_GREEN && ns_demand && !ew_demand) ||
                                        (current_phase == EW_GREEN && ew_demand && !ns_demand);
        if (hold_current_green && phase_elapsed >= phase_duration)
        {
            phase_elapsed = phase_duration - 1e-6;
        }

        if (shouldEndCurrentGreenEarly())
        {
            phase_elapsed = phase_duration;
        }

        // Handle multiple phase transitions in a single tick if needed
        while (phase_elapsed >= phase_duration)
        {
            phase_elapsed -= phase_duration;
            transitionToNextPhase();

            // Get new phase duration after transition
            switch (current_phase)
            {
            case NS_GREEN:
                phase_duration = ns_duration;
                break;
            case NS_ORANGE:
                phase_duration = SafetyChecker::ORANGE_DURATION;
                break;
            case EW_GREEN:
                phase_duration = ew_duration;
                break;
            case EW_ORANGE:
                phase_duration = SafetyChecker::ORANGE_DURATION;
                break;
            }
        }
    }

    IntersectionState BasicLightController::getCurrentState() const
    {
        return current_state;
    }

    bool BasicLightController::shouldEndCurrentGreenEarly() const
    {
        if ((current_phase != NS_GREEN && current_phase != EW_GREEN) || phase_elapsed < min_green_seconds)
        {
            return false;
        }

        const bool ns_demand = demand_by_direction[0] || demand_by_direction[1];
        const bool ew_demand = demand_by_direction[2] || demand_by_direction[3];

        if (current_phase == NS_GREEN)
        {
            const bool ew_starving = ew_demand && ew_red_elapsed >= max_red_seconds;
            return ew_starving || (!ns_demand && ew_demand);
        }

        const bool ns_starving = ns_demand && ns_red_elapsed >= max_red_seconds;
        return ns_starving || (!ew_demand && ns_demand);
    }

    void BasicLightController::updateRedTimers(double dt_seconds)
    {
        const bool ns_red = (current_phase == EW_GREEN || current_phase == EW_ORANGE);
        const bool ew_red = (current_phase == NS_GREEN || current_phase == NS_ORANGE);

        if (ns_red)
        {
            ns_red_elapsed += dt_seconds;
        }
        else
        {
            ns_red_elapsed = 0.0;
        }

        if (ew_red)
        {
            ew_red_elapsed += dt_seconds;
        }
        else
        {
            ew_red_elapsed = 0.0;
        }
    }

} // namespace crossroads
