#pragma once

#include "Intersection.hpp"
#include "SafetyChecker.hpp"

namespace crossroads
{

    class BasicLightController
    {
    public:
        // Duration (in seconds) for NS and EW green phases
        BasicLightController(double ns_green_duration = 10.0, double ew_green_duration = 10.0);

        // Advance time by dt_seconds; may trigger state transitions
        void tick(double dt_seconds);

        // Get current intersection state
        IntersectionState getCurrentState() const;

        // Reset to initial state (NS green, all turning lights red)
        void reset();

    private:
        IntersectionState current_state;
        SafetyChecker checker;

        double ns_duration;   // How long NS stays green
        double ew_duration;   // How long EW stays green
        double phase_elapsed; // Time spent in current phase

        enum Phase
        {
            NS_GREEN,
            NS_ORANGE,
            EW_GREEN,
            EW_ORANGE
        };

        Phase current_phase;

        // Helper to transition to next phase
        void transitionToNextPhase();

        // Helper to apply a phase's light pattern
        void applyPhasePattern(Phase phase, IntersectionState &state);
    };

} // namespace crossroads
