#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace crossroads
{
    enum class ApproachId : uint8_t
    {
        North = 0,
        East = 1,
        South = 2,
        West = 3
    };

    enum class MovementType : uint8_t
    {
        Straight = 0,
        Left = 1,
        Right = 2
    };

    using LaneId = uint16_t;
    using SignalGroupId = uint16_t;

    inline LaneId laneIdFor(ApproachId approach, std::size_t lane_index)
    {
        return static_cast<LaneId>(static_cast<uint16_t>(approach) * 100 + lane_index);
    }

    inline std::size_t approachIndex(ApproachId approach)
    {
        return static_cast<std::size_t>(static_cast<uint8_t>(approach));
    }

    inline ApproachId destinationApproachFor(ApproachId from, MovementType movement)
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
        return ApproachId::North;
    }

    struct LaneConfig
    {
        LaneId id = 0;
        std::string name;
        std::vector<MovementType> allowed_movements;
        bool supports_lane_change = true;
        bool connected_to_intersection = true;
        bool has_traffic_light = true;
    };

    struct ApproachConfig
    {
        ApproachId id = ApproachId::North;
        std::string name;
        std::vector<LaneConfig> lanes;
        uint16_t to_lane_count = 1;
    };

    inline std::size_t effectiveToLaneCount(const ApproachConfig &approach)
    {
        if (approach.to_lane_count > 0)
        {
            return approach.to_lane_count;
        }
        if (!approach.lanes.empty())
        {
            return approach.lanes.size();
        }
        return 1;
    }

    struct SignalGroupConfig
    {
        SignalGroupId id = 0;
        std::string name;
        std::vector<LaneId> controlled_lanes;
        std::vector<MovementType> green_movements;
        double min_green_seconds = 10.0;
        double orange_seconds = 2.0;
    };

    struct LaneConnectionConfig
    {
        ApproachId from_approach = ApproachId::North;
        uint16_t from_lane_index = 0;
        MovementType movement = MovementType::Straight;
        ApproachId to_approach = ApproachId::South;
        uint16_t to_lane_index = 0;
    };

    struct IntersectionConfig
    {
        std::array<ApproachConfig, 4> approaches;
        std::vector<SignalGroupConfig> signal_groups;
        std::vector<LaneConnectionConfig> lane_connections;
    };

    inline IntersectionConfig makeDefaultIntersectionConfig()
    {
        IntersectionConfig config;

        config.approaches[0] = {ApproachId::North, "North", {{laneIdFor(ApproachId::North, 0), "N-0", {MovementType::Straight}, true, true, true}, {laneIdFor(ApproachId::North, 1), "N-1", {MovementType::Straight}, true, true, true}, {laneIdFor(ApproachId::North, 2), "N-2", {MovementType::Right}, true, true, true}}};
        config.approaches[1] = {ApproachId::East, "East", {{laneIdFor(ApproachId::East, 0), "E-0", {MovementType::Straight}, true, true, true}, {laneIdFor(ApproachId::East, 1), "E-1", {MovementType::Straight}, true, true, true}, {laneIdFor(ApproachId::East, 2), "E-2", {MovementType::Right}, true, true, true}}};
        config.approaches[2] = {ApproachId::South, "South", {{laneIdFor(ApproachId::South, 0), "S-0", {MovementType::Straight}, true, true, true}, {laneIdFor(ApproachId::South, 1), "S-1", {MovementType::Straight}, true, true, true}, {laneIdFor(ApproachId::South, 2), "S-2", {MovementType::Right}, true, true, true}}};
        config.approaches[3] = {ApproachId::West, "West", {{laneIdFor(ApproachId::West, 0), "W-0", {MovementType::Straight}, true, true, true}, {laneIdFor(ApproachId::West, 1), "W-1", {MovementType::Straight}, true, true, true}, {laneIdFor(ApproachId::West, 2), "W-2", {MovementType::Right}, true, true, true}}};

        for (auto &approach : config.approaches)
        {
            approach.to_lane_count = static_cast<uint16_t>(approach.lanes.size());
        }

        for (const auto &approach : config.approaches)
        {
            for (std::size_t lane_idx = 0; lane_idx < approach.lanes.size(); ++lane_idx)
            {
                const auto &lane = approach.lanes[lane_idx];
                for (MovementType movement : lane.allowed_movements)
                {
                    ApproachId to = destinationApproachFor(approach.id, movement);
                    std::size_t to_idx = approachIndex(to);
                    std::size_t to_count = effectiveToLaneCount(config.approaches[to_idx]);
                    std::size_t max_target_lane = to_count == 0 ? 0 : (to_count - 1);
                    std::size_t target_lane = lane_idx <= max_target_lane ? lane_idx : max_target_lane;
                    config.lane_connections.push_back({approach.id,
                                                       static_cast<uint16_t>(lane_idx),
                                                       movement,
                                                       to,
                                                       static_cast<uint16_t>(target_lane)});
                }
            }
        }

        return config;
    }

} // namespace crossroads
