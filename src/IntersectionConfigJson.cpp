#include "IntersectionConfigJson.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cctype>
#include <unordered_set>

namespace crossroads
{
    namespace
    {
        using nlohmann::json;

        std::string approachToString(ApproachId id)
        {
            switch (id)
            {
            case ApproachId::North:
                return "north";
            case ApproachId::East:
                return "east";
            case ApproachId::South:
                return "south";
            case ApproachId::West:
                return "west";
            }
            return "north";
        }

        bool approachFromString(const std::string &value, ApproachId &id)
        {
            if (value == "north")
            {
                id = ApproachId::North;
                return true;
            }
            if (value == "east")
            {
                id = ApproachId::East;
                return true;
            }
            if (value == "south")
            {
                id = ApproachId::South;
                return true;
            }
            if (value == "west")
            {
                id = ApproachId::West;
                return true;
            }
            return false;
        }

        std::string movementToString(MovementType movement)
        {
            switch (movement)
            {
            case MovementType::Straight:
                return "straight";
            case MovementType::Left:
                return "left";
            case MovementType::Right:
                return "right";
            }
            return "straight";
        }

        bool movementFromString(const std::string &value, MovementType &movement)
        {
            if (value == "straight")
            {
                movement = MovementType::Straight;
                return true;
            }
            if (value == "left")
            {
                movement = MovementType::Left;
                return true;
            }
            if (value == "right")
            {
                movement = MovementType::Right;
                return true;
            }
            return false;
        }

        size_t approachIndexLocal(ApproachId id)
        {
            switch (id)
            {
            case ApproachId::North:
                return 0;
            case ApproachId::East:
                return 1;
            case ApproachId::South:
                return 2;
            case ApproachId::West:
                return 3;
            }
            return 0;
        }

        bool laneIndexValid(const IntersectionConfig &config, ApproachId approach, uint16_t lane_index)
        {
            const std::size_t idx = approachIndexLocal(approach);
            if (idx >= config.approaches.size())
            {
                return false;
            }
            return lane_index < config.approaches[idx].lanes.size();
        }

        bool toLaneIndexValid(const IntersectionConfig &config, ApproachId approach, uint16_t lane_index)
        {
            const std::size_t idx = approachIndexLocal(approach);
            if (idx >= config.approaches.size())
            {
                return false;
            }
            return lane_index < effectiveToLaneCount(config.approaches[idx]);
        }

        bool resolveLaneIdToApproachIndex(const IntersectionConfig &config,
                                          LaneId lane_id,
                                          ApproachId &approach,
                                          uint16_t &lane_index)
        {
            for (const auto &entry : config.approaches)
            {
                for (uint16_t i = 0; i < entry.lanes.size(); ++i)
                {
                    if (entry.lanes[i].id == lane_id)
                    {
                        approach = entry.id;
                        lane_index = i;
                        return true;
                    }
                }
            }
            return false;
        }
    }

    std::string intersectionConfigToJson(const IntersectionConfig &config)
    {
        json root;
        root["approaches"] = json::array();

        for (const auto &approach : config.approaches)
        {
            json approach_json;
            approach_json["id"] = approachToString(approach.id);
            approach_json["name"] = approach.name;
            approach_json["to_lane_count"] = approach.to_lane_count;
            approach_json["lanes"] = json::array();

            for (const auto &lane : approach.lanes)
            {
                json lane_json;
                lane_json["id"] = lane.id;
                lane_json["index"] = lane.id - laneIdFor(approach.id, 0);
                lane_json["name"] = lane.name;
                lane_json["supports_lane_change"] = lane.supports_lane_change;
                lane_json["connected_to_intersection"] = lane.connected_to_intersection;
                lane_json["has_traffic_light"] = lane.has_traffic_light;
                lane_json["allowed_movements"] = json::array();
                for (MovementType movement : lane.allowed_movements)
                {
                    lane_json["allowed_movements"].push_back(movementToString(movement));
                }
                approach_json["lanes"].push_back(lane_json);
            }

            root["approaches"].push_back(approach_json);
        }

        root["signal_groups"] = json::array();
        for (const auto &group : config.signal_groups)
        {
            json group_json;
            group_json["id"] = group.id;
            group_json["name"] = group.name;
            group_json["controlled_lanes"] = group.controlled_lanes;
            group_json["green_movements"] = json::array();
            for (MovementType movement : group.green_movements)
            {
                group_json["green_movements"].push_back(movementToString(movement));
            }
            group_json["min_green_seconds"] = group.min_green_seconds;
            group_json["orange_seconds"] = group.orange_seconds;
            root["signal_groups"].push_back(group_json);
        }

        root["lane_connections"] = json::array();
        for (const auto &connection : config.lane_connections)
        {
            json connection_json;
            connection_json["from_approach"] = approachToString(connection.from_approach);
            connection_json["from_lane_index"] = connection.from_lane_index;
            connection_json["from_lane_id"] = laneIdFor(connection.from_approach, connection.from_lane_index);
            connection_json["movement"] = movementToString(connection.movement);
            connection_json["to_approach"] = approachToString(connection.to_approach);
            connection_json["to_lane_index"] = connection.to_lane_index;
            connection_json["to_lane_id"] = laneIdFor(connection.to_approach, connection.to_lane_index);
            root["lane_connections"].push_back(connection_json);
        }

        return root.dump();
    }

    ConfigParseResult intersectionConfigFromJson(const std::string &json_text)
    {
        ConfigParseResult result;

        json root;
        try
        {
            root = json::parse(json_text);
        }
        catch (const std::exception &e)
        {
            result.errors.push_back(std::string("invalid JSON: ") + e.what());
            return result;
        }

        if (!root.is_object())
        {
            result.errors.push_back("root must be an object");
            return result;
        }

        if (!root.contains("approaches") || !root["approaches"].is_array())
        {
            result.errors.push_back("approaches must be an array");
            return result;
        }

        const json &approaches_json = root["approaches"];
        if (approaches_json.size() != 4)
        {
            result.errors.push_back("approaches must contain exactly 4 entries");
            return result;
        }

        std::array<bool, 4> seen_approaches = {false, false, false, false};
        std::unordered_set<LaneId> seen_lanes;

        for (const auto &approach_json : approaches_json)
        {
            if (!approach_json.is_object())
            {
                result.errors.push_back("each approach entry must be an object");
                continue;
            }

            if (!approach_json.contains("id") || !approach_json["id"].is_string())
            {
                result.errors.push_back("approach.id must be a string");
                continue;
            }

            ApproachId approach_id{};
            const std::string id_value = approach_json["id"].get<std::string>();
            if (!approachFromString(id_value, approach_id))
            {
                result.errors.push_back("unknown approach id: " + id_value);
                continue;
            }

            const size_t idx = approachIndexLocal(approach_id);
            if (seen_approaches[idx])
            {
                result.errors.push_back("duplicate approach id: " + id_value);
                continue;
            }
            seen_approaches[idx] = true;

            ApproachConfig approach;
            approach.id = approach_id;
            approach.name = approach_json.value("name", id_value);
            if (approach_json.contains("to_lane_count") && approach_json["to_lane_count"].is_number_unsigned())
            {
                approach.to_lane_count = std::min<uint16_t>(approach_json["to_lane_count"].get<uint16_t>(), 64);
            }
            else
            {
                approach.to_lane_count = 0;
            }

            if (!approach_json.contains("lanes") || !approach_json["lanes"].is_array())
            {
                result.errors.push_back("approach " + id_value + " lanes must be an array");
                continue;
            }

            size_t lane_index = 0;
            for (const auto &lane_json : approach_json["lanes"])
            {
                if (!lane_json.is_object())
                {
                    result.errors.push_back("lane entries must be objects");
                    continue;
                }

                LaneConfig lane;
                lane.id = laneIdFor(approach_id, lane_index);
                const std::string default_lane_name =
                    std::string(1, static_cast<char>(std::toupper(id_value[0]))) + "-" + std::to_string(lane_index);
                lane.name = lane_json.value("name", default_lane_name);
                lane.supports_lane_change = lane_json.value("supports_lane_change", true);
                lane.connected_to_intersection = lane_json.value("connected_to_intersection", true);
                lane.has_traffic_light = lane_json.value("has_traffic_light", true);
                if (!lane.connected_to_intersection)
                {
                    lane.has_traffic_light = false;
                }

                if (!seen_lanes.insert(lane.id).second)
                {
                    result.errors.push_back("duplicate lane id: " + std::to_string(lane.id));
                }

                if (!lane_json.contains("allowed_movements") || !lane_json["allowed_movements"].is_array())
                {
                    result.errors.push_back("lane " + std::to_string(lane.id) + " allowed_movements must be an array");
                    continue;
                }

                for (const auto &movement_json : lane_json["allowed_movements"])
                {
                    if (!movement_json.is_string())
                    {
                        result.errors.push_back("lane " + std::to_string(lane.id) + " movement must be a string");
                        continue;
                    }

                    MovementType movement;
                    const std::string movement_value = movement_json.get<std::string>();
                    if (!movementFromString(movement_value, movement))
                    {
                        result.errors.push_back("lane " + std::to_string(lane.id) + " unknown movement: " + movement_value);
                        continue;
                    }

                    if (std::find(lane.allowed_movements.begin(), lane.allowed_movements.end(), movement) == lane.allowed_movements.end())
                    {
                        lane.allowed_movements.push_back(movement);
                    }
                }

                approach.lanes.push_back(lane);
                lane_index += 1;
            }

            result.config.approaches[idx] = approach;
        }

        for (auto &approach : result.config.approaches)
        {
            if (approach.to_lane_count == 0)
            {
                if (!approach.lanes.empty())
                {
                    approach.to_lane_count = static_cast<uint16_t>(approach.lanes.size());
                }
                else
                {
                    approach.to_lane_count = 1;
                }
            }
        }

        for (size_t i = 0; i < seen_approaches.size(); ++i)
        {
            if (!seen_approaches[i])
            {
                result.errors.push_back("missing approach entry");
            }
        }

        if (root.contains("signal_groups"))
        {
            if (!root["signal_groups"].is_array())
            {
                result.errors.push_back("signal_groups must be an array");
            }
            else
            {
                std::unordered_set<SignalGroupId> seen_groups;
                for (const auto &group_json : root["signal_groups"])
                {
                    if (!group_json.is_object())
                    {
                        result.errors.push_back("signal_group entries must be objects");
                        continue;
                    }

                    if (!group_json.contains("id") || !group_json["id"].is_number_unsigned())
                    {
                        result.errors.push_back("signal_group.id must be an unsigned number");
                        continue;
                    }

                    SignalGroupConfig group;
                    group.id = group_json["id"].get<SignalGroupId>();
                    if (!seen_groups.insert(group.id).second)
                    {
                        result.errors.push_back("duplicate signal_group id: " + std::to_string(group.id));
                    }
                    group.name = group_json.value("name", std::string("group-" + std::to_string(group.id)));
                    group.min_green_seconds = group_json.value("min_green_seconds", 10.0);
                    group.orange_seconds = group_json.value("orange_seconds", 2.0);

                    if (!group_json.contains("controlled_lanes") || !group_json["controlled_lanes"].is_array())
                    {
                        result.errors.push_back("signal_group " + std::to_string(group.id) + " controlled_lanes must be an array");
                        continue;
                    }

                    for (const auto &lane_json : group_json["controlled_lanes"])
                    {
                        if (!lane_json.is_number_unsigned())
                        {
                            result.errors.push_back("signal_group " + std::to_string(group.id) + " controlled lane ids must be unsigned numbers");
                            continue;
                        }
                        group.controlled_lanes.push_back(lane_json.get<LaneId>());
                    }

                    if (!group_json.contains("green_movements") || !group_json["green_movements"].is_array())
                    {
                        result.errors.push_back("signal_group " + std::to_string(group.id) + " green_movements must be an array");
                        continue;
                    }

                    for (const auto &movement_json : group_json["green_movements"])
                    {
                        if (!movement_json.is_string())
                        {
                            result.errors.push_back("signal_group " + std::to_string(group.id) + " movement must be a string");
                            continue;
                        }

                        MovementType movement;
                        const std::string movement_value = movement_json.get<std::string>();
                        if (!movementFromString(movement_value, movement))
                        {
                            result.errors.push_back("signal_group " + std::to_string(group.id) + " unknown movement: " + movement_value);
                            continue;
                        }

                        if (std::find(group.green_movements.begin(), group.green_movements.end(), movement) == group.green_movements.end())
                        {
                            group.green_movements.push_back(movement);
                        }
                    }

                    result.config.signal_groups.push_back(group);
                }
            }
        }

        if (root.contains("lane_connections"))
        {
            if (!root["lane_connections"].is_array())
            {
                result.errors.push_back("lane_connections must be an array");
            }
            else
            {
                for (const auto &connection_json : root["lane_connections"])
                {
                    if (!connection_json.is_object())
                    {
                        result.errors.push_back("lane_connection entries must be objects");
                        continue;
                    }

                    LaneConnectionConfig connection;
                    bool from_ok = false;
                    bool to_ok = false;

                    if (connection_json.contains("from_approach") && connection_json.contains("from_lane_index") &&
                        connection_json["from_approach"].is_string() && connection_json["from_lane_index"].is_number_unsigned())
                    {
                        from_ok = approachFromString(connection_json["from_approach"].get<std::string>(), connection.from_approach);
                        connection.from_lane_index = connection_json["from_lane_index"].get<uint16_t>();
                    }
                    else if (connection_json.contains("from_lane_id") && connection_json["from_lane_id"].is_number_unsigned())
                    {
                        from_ok = resolveLaneIdToApproachIndex(result.config,
                                                               connection_json["from_lane_id"].get<LaneId>(),
                                                               connection.from_approach,
                                                               connection.from_lane_index);
                    }

                    if (!from_ok)
                    {
                        result.errors.push_back("lane_connection has invalid source lane reference");
                        continue;
                    }

                    if (!connection_json.contains("movement") || !connection_json["movement"].is_string())
                    {
                        result.errors.push_back("lane_connection.movement must be a string");
                        continue;
                    }

                    if (!movementFromString(connection_json["movement"].get<std::string>(), connection.movement))
                    {
                        result.errors.push_back("lane_connection has unknown movement");
                        continue;
                    }

                    if (connection_json.contains("to_approach") && connection_json.contains("to_lane_index") &&
                        connection_json["to_approach"].is_string() && connection_json["to_lane_index"].is_number_unsigned())
                    {
                        to_ok = approachFromString(connection_json["to_approach"].get<std::string>(), connection.to_approach);
                        connection.to_lane_index = connection_json["to_lane_index"].get<uint16_t>();
                    }
                    else if (connection_json.contains("to_lane_id") && connection_json["to_lane_id"].is_number_unsigned())
                    {
                        to_ok = resolveLaneIdToApproachIndex(result.config,
                                                             connection_json["to_lane_id"].get<LaneId>(),
                                                             connection.to_approach,
                                                             connection.to_lane_index);
                    }

                    if (!to_ok)
                    {
                        result.errors.push_back("lane_connection has invalid target lane reference");
                        continue;
                    }

                    if (!laneIndexValid(result.config, connection.from_approach, connection.from_lane_index) ||
                        !toLaneIndexValid(result.config, connection.to_approach, connection.to_lane_index))
                    {
                        result.errors.push_back("lane_connection references lane index outside configured range");
                        continue;
                    }

                    result.config.lane_connections.push_back(connection);
                }
            }
        }
        else
        {
            for (const auto &approach : result.config.approaches)
            {
                for (uint16_t lane_idx = 0; lane_idx < approach.lanes.size(); ++lane_idx)
                {
                    const auto &lane = approach.lanes[lane_idx];
                    for (MovementType movement : lane.allowed_movements)
                    {
                        const ApproachId to = destinationApproachFor(approach.id, movement);
                        const std::size_t to_idx = approachIndexLocal(to);
                        const std::size_t to_count = effectiveToLaneCount(result.config.approaches[to_idx]);
                        const std::size_t max_target_lane = to_count == 0 ? 0 : (to_count - 1);
                        const std::size_t target_lane = lane_idx <= max_target_lane ? lane_idx : max_target_lane;
                        result.config.lane_connections.push_back({approach.id,
                                                                  lane_idx,
                                                                  movement,
                                                                  to,
                                                                  static_cast<uint16_t>(target_lane)});
                    }
                }
            }
        }

        result.ok = result.errors.empty();
        return result;
    }

    std::string validationErrorsToJson(const std::vector<std::string> &errors)
    {
        nlohmann::json root;
        root["ok"] = false;
        root["errors"] = errors;
        return root.dump();
    }
} // namespace crossroads
