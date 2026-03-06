#pragma once

#include <array>

#include "Intersection.hpp"
#include "SafetyChecker.hpp"

namespace crossroads {

    class BasicLightController {
       public:
        // Duration (in seconds) for NS and EW green phases
        BasicLightController(double ns_green_duration = 10.0, double ew_green_duration = 10.0);

        // Advance time by dt_seconds; may trigger state transitions
        void tick(double dt_seconds);

        // Get current intersection state
        IntersectionState getCurrentState() const;

        // Reset to initial state (NS green, all turning lights red)
        void reset();

        // Provide per-direction demand flags in order: North, South, East, West
        void setDemandByDirection(const std::array<bool, 4>& demand);

       private:
        IntersectionState current_state;
        SafetyChecker checker;

        double ns_duration;    // How long NS stays green
        double ew_duration;    // How long EW stays green
        double phase_elapsed;  // Time spent in current phase
        std::array<bool, 4> demand_by_direction{{false, false, false, false}};
        double min_green_seconds = 1.0;
        double max_red_seconds = 8.0;
        double ns_red_elapsed = 0.0;
        double ew_red_elapsed = 0.0;

        enum Phase { NS_GREEN, NS_ORANGE, EW_GREEN, EW_ORANGE };

        Phase current_phase;

        // Helper to transition to next phase
        void transitionToNextPhase();

        // Helper to apply a phase's light pattern
        void applyPhasePattern(Phase phase, IntersectionState& state);

        bool shouldEndCurrentGreenEarly() const;
        void updateRedTimers(double dt_seconds);
    };

}  // namespace crossroads
