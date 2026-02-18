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
        applyPhasePattern(NS_GREEN, current_state);
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

} // namespace crossroads
