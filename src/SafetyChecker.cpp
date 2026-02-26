#include "SafetyChecker.hpp"

#include <algorithm>
#include <unordered_set>

namespace crossroads
{

    SafetyChecker::SafetyChecker()
        : SafetyChecker(makeDefaultIntersectionConfig())
    {
    }

    SafetyChecker::SafetyChecker(const IntersectionConfig &config)
        : intersection_config(config),
          config_valid(validateConfig(config))
    {
    }

    bool SafetyChecker::isConfigValid() const
    {
        return config_valid;
    }

    bool SafetyChecker::validateConfig(const IntersectionConfig &config) const
    {
        std::unordered_set<LaneId> seen_lanes;
        std::unordered_set<SignalGroupId> seen_groups;
        for (const auto &approach : config.approaches)
        {
            if (approach.lanes.empty())
            {
                return false;
            }

            for (const auto &lane : approach.lanes)
            {
                if (lane.allowed_movements.empty())
                {
                    return false;
                }
                if (!seen_lanes.insert(lane.id).second)
                {
                    return false;
                }
            }
        }

        for (const auto &group : config.signal_groups)
        {
            if (!seen_groups.insert(group.id).second)
            {
                return false;
            }

            if (group.controlled_lanes.empty() || group.green_movements.empty())
            {
                return false;
            }

            for (LaneId lane : group.controlled_lanes)
            {
                if (seen_lanes.find(lane) == seen_lanes.end())
                {
                    return false;
                }
            }
        }

        return true;
    }

    bool SafetyChecker::tryFindApproachForLane(LaneId lane_id, ApproachId &approach) const
    {
        for (const auto &entry : intersection_config.approaches)
        {
            auto found = std::find_if(entry.lanes.begin(), entry.lanes.end(),
                                      [lane_id](const LaneConfig &lane)
                                      { return lane.id == lane_id; });
            if (found != entry.lanes.end())
            {
                approach = entry.id;
                return true;
            }
        }

        return false;
    }

    ApproachId SafetyChecker::destinationFor(ApproachId from, MovementType movement)
    {
        switch (from)
        {
        case ApproachId::North:
            if (movement == MovementType::Straight)
                return ApproachId::South;
            if (movement == MovementType::Left)
                return ApproachId::East;
            return ApproachId::West;
        case ApproachId::East:
            if (movement == MovementType::Straight)
                return ApproachId::West;
            if (movement == MovementType::Left)
                return ApproachId::South;
            return ApproachId::North;
        case ApproachId::South:
            if (movement == MovementType::Straight)
                return ApproachId::North;
            if (movement == MovementType::Left)
                return ApproachId::West;
            return ApproachId::East;
        case ApproachId::West:
            if (movement == MovementType::Straight)
                return ApproachId::East;
            if (movement == MovementType::Left)
                return ApproachId::North;
            return ApproachId::South;
        }

        return from;
    }

    bool SafetyChecker::isInNorthSouthCorridor(ApproachId from, MovementType movement)
    {
        ApproachId to = destinationFor(from, movement);
        bool from_ns = (from == ApproachId::North || from == ApproachId::South);
        bool to_ns = (to == ApproachId::North || to == ApproachId::South);
        return from_ns && to_ns;
    }

    bool SafetyChecker::hasMovementConflict(ApproachId from_a, MovementType move_a,
                                            ApproachId from_b, MovementType move_b) const
    {
        if (from_a == from_b && move_a == move_b)
        {
            return false;
        }

        if (from_a == from_b)
        {
            return true;
        }

        if (destinationFor(from_a, move_a) == destinationFor(from_b, move_b))
        {
            return true;
        }

        bool opposite_pair =
            ((from_a == ApproachId::North && from_b == ApproachId::South) ||
             (from_a == ApproachId::South && from_b == ApproachId::North) ||
             (from_a == ApproachId::East && from_b == ApproachId::West) ||
             (from_a == ApproachId::West && from_b == ApproachId::East));

        if (opposite_pair && move_a == MovementType::Left && move_b == MovementType::Left)
        {
            return true;
        }

        bool a_ns = isInNorthSouthCorridor(from_a, move_a);
        bool b_ns = isInNorthSouthCorridor(from_b, move_b);

        if (a_ns != b_ns)
        {
            if (move_a == MovementType::Straight || move_b == MovementType::Straight)
            {
                return true;
            }
        }

        return false;
    }

    bool SafetyChecker::areSignalGroupsConflictFree(const std::vector<SignalGroupId> &active_group_ids) const
    {
        if (!config_valid)
        {
            return false;
        }

        struct MovementRef
        {
            ApproachId from;
            MovementType movement;
        };

        std::vector<MovementRef> active_movements;
        for (SignalGroupId group_id : active_group_ids)
        {
            auto group_it = std::find_if(intersection_config.signal_groups.begin(),
                                         intersection_config.signal_groups.end(),
                                         [group_id](const SignalGroupConfig &g)
                                         { return g.id == group_id; });
            if (group_it == intersection_config.signal_groups.end())
            {
                return false;
            }

            for (LaneId lane_id : group_it->controlled_lanes)
            {
                ApproachId approach;
                if (!tryFindApproachForLane(lane_id, approach))
                {
                    return false;
                }

                for (MovementType movement : group_it->green_movements)
                {
                    active_movements.push_back({approach, movement});
                }
            }
        }

        for (size_t i = 0; i < active_movements.size(); ++i)
        {
            for (size_t j = i + 1; j < active_movements.size(); ++j)
            {
                if (hasMovementConflict(active_movements[i].from, active_movements[i].movement,
                                        active_movements[j].from, active_movements[j].movement))
                {
                    return false;
                }
            }
        }

        return true;
    }

    bool SafetyChecker::isSafe(const IntersectionState &state) const
    {
        return hasConflictingGreens(state) && checkTurningLightSafety(state);
    }

    bool SafetyChecker::hasConflictingGreens(const IntersectionState &state) const
    {
        bool nsGreen = (state.north == LightState::Green) || (state.south == LightState::Green);
        bool ewGreen = (state.east == LightState::Green) || (state.west == LightState::Green);
        return !(nsGreen && ewGreen);
    }

    bool SafetyChecker::checkTurningLightSafety(const IntersectionState &state) const
    {
        auto is_active = [](LightState s)
        { return s == LightState::Green || s == LightState::Orange; };

        // turnSouthEast cannot be green if West is active
        if (state.turnSouthEast == LightState::Green && is_active(state.west))
            return false;

        // turnNorthWest cannot be green if East is active
        if (state.turnNorthWest == LightState::Green && is_active(state.east))
            return false;

        // turnWestSouth cannot be green if North is active
        if (state.turnWestSouth == LightState::Green && is_active(state.north))
            return false;

        // turnEastNorth cannot be green if South is active
        if (state.turnEastNorth == LightState::Green && is_active(state.south))
            return false;

        return true;
    }

    bool SafetyChecker::isValidTransition(const IntersectionState &prev, const IntersectionState &next, double dt_seconds) const
    {
        return checkPerLightTransitions(prev, next) &&
               checkOrangeTiming(prev, next, dt_seconds) &&
               isSafe(next) &&
               checkCrossingLightSafety(prev, next) &&
               checkTurningLightTransitions(next);
    }

    bool SafetyChecker::checkPerLightTransitions(const IntersectionState &prev, const IntersectionState &next) const
    {
        auto valid_for_light = [](LightState p, LightState n)
        {
            if (p == n)
                return true;
            if (p == LightState::Green && n == LightState::Orange)
                return true;
            if (p == LightState::Orange && n == LightState::Red)
                return true;
            if (p == LightState::Red && n == LightState::Green)
                return true;
            return false;
        };

        // Check all 8 lights for valid transitions
        return valid_for_light(prev.north, next.north) &&
               valid_for_light(prev.east, next.east) &&
               valid_for_light(prev.south, next.south) &&
               valid_for_light(prev.west, next.west) &&
               valid_for_light(prev.turnSouthEast, next.turnSouthEast) &&
               valid_for_light(prev.turnNorthWest, next.turnNorthWest) &&
               valid_for_light(prev.turnWestSouth, next.turnWestSouth) &&
               valid_for_light(prev.turnEastNorth, next.turnEastNorth);
    }

    bool SafetyChecker::checkOrangeTiming(const IntersectionState &prev, const IntersectionState &next, double dt_seconds) const
    {
        auto check_orange_duration = [dt_seconds](LightState p, LightState n)
        {
            return !((p == LightState::Orange && n == LightState::Red) && dt_seconds < ORANGE_DURATION);
        };

        // Verify orange lasted long enough for all 8 lights
        return check_orange_duration(prev.north, next.north) &&
               check_orange_duration(prev.east, next.east) &&
               check_orange_duration(prev.south, next.south) &&
               check_orange_duration(prev.west, next.west) &&
               check_orange_duration(prev.turnSouthEast, next.turnSouthEast) &&
               check_orange_duration(prev.turnNorthWest, next.turnNorthWest) &&
               check_orange_duration(prev.turnWestSouth, next.turnWestSouth) &&
               check_orange_duration(prev.turnEastNorth, next.turnEastNorth);
    }

    bool SafetyChecker::checkCrossingLightSafety(const IntersectionState &prev, const IntersectionState &next) const
    {
        auto is_active = [](LightState s)
        { return s == LightState::Green || s == LightState::Orange; };

        // Detect transitions to green for NS and EW routes
        bool north_going_green = (prev.north != LightState::Green && next.north == LightState::Green);
        bool south_going_green = (prev.south != LightState::Green && next.south == LightState::Green);
        bool east_going_green = (prev.east != LightState::Green && next.east == LightState::Green);
        bool west_going_green = (prev.west != LightState::Green && next.west == LightState::Green);

        // If NS route is transitioning to green, EW must be inactive
        if ((north_going_green || south_going_green) && (is_active(next.east) || is_active(next.west)))
            return false;

        // If EW route is transitioning to green, NS must be inactive
        if ((east_going_green || west_going_green) && (is_active(next.north) || is_active(next.south)))
            return false;

        return true;
    }

    bool SafetyChecker::checkTurningLightTransitions(const IntersectionState &next) const
    {
        auto is_active = [](LightState s)
        { return s == LightState::Green || s == LightState::Orange; };

        // turnSouthEast cannot go green if West is active
        if (next.turnSouthEast == LightState::Green && is_active(next.west))
            return false;

        // turnNorthWest cannot go green if East is active
        if (next.turnNorthWest == LightState::Green && is_active(next.east))
            return false;

        // turnWestSouth cannot go green if North is active
        if (next.turnWestSouth == LightState::Green && is_active(next.north))
            return false;

        // turnEastNorth cannot go green if South is active
        if (next.turnEastNorth == LightState::Green && is_active(next.south))
            return false;

        return true;
    }

} // namespace crossroads
