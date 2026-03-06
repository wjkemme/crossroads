#include <algorithm>
#include <atomic>
#include <cctype>
#include <chrono>
#include <csignal>
#include <iostream>
#include <mutex>
#include <nlohmann/json.hpp>
#include <optional>
#include <sstream>
#include <thread>
#include <vector>

#include "IntersectionConfigJson.hpp"
#include "SafetyChecker.hpp"
#include "SimpleHttpUiServer.hpp"
#include "SimulatorEngine.hpp"
#include "db/Database.hpp"

namespace {
    std::atomic<bool> g_keep_running{true};

    std::string trimCopy(std::string value) {
        value.erase(value.begin(),
                    std::find_if(value.begin(), value.end(), [](unsigned char ch) { return !std::isspace(ch); }));
        value.erase(
            std::find_if(value.rbegin(), value.rend(), [](unsigned char ch) { return !std::isspace(ch); }).base(),
            value.end());
        return value;
    }

    void handleSignal(int) {
        g_keep_running = false;
    }
}  // namespace

int main() {
    std::signal(SIGINT, handleSignal);
    std::signal(SIGTERM, handleSignal);

    std::cout << "=== Crossroads Traffic Simulator UI ===" << std::endl;
    std::cout << std::endl;

    constexpr double kDefaultTrafficRate = 0.8;
    constexpr double kNorthSouthDuration = 10.0;
    constexpr double kEastWestDuration = 10.0;
    double traffic_rate = kDefaultTrafficRate;

    crossroads::db::Database database("crossroads.db");
    std::string db_error;
    if (!database.initialize(&db_error)) {
        std::cerr << "Warning: failed to initialize config database: " << db_error << std::endl;
    }

    crossroads::IntersectionConfig initial_config = crossroads::makeDefaultIntersectionConfig();
    if (auto stored = database.loadActiveIntersectionConfigJson(&db_error); stored.has_value()) {
        crossroads::ConfigParseResult parsed = crossroads::intersectionConfigFromJson(*stored);
        crossroads::SafetyChecker checker(parsed.config);
        if (parsed.ok && checker.isConfigValid()) {
            initial_config = parsed.config;
        } else {
            std::cerr << "Warning: stored config is invalid, using defaults" << std::endl;
        }
    } else if (!db_error.empty()) {
        std::cerr << "Warning: failed to load config from database: " << db_error << std::endl;
    } else {
        database.saveActiveIntersectionConfigJson(crossroads::intersectionConfigToJson(initial_config), &db_error);
    }

    std::string initial_config_json = crossroads::intersectionConfigToJson(initial_config);
    if (!database.loadMostRecentNamedConfigName(&db_error).has_value()) {
        std::string seed_error;
        if (!database.saveNamedIntersectionConfigJson("Standaard", initial_config_json, &seed_error)) {
            std::cerr << "Warning: failed to seed named config: " << seed_error << std::endl;
        } else {
            database.touchNamedIntersectionConfig("Standaard", &seed_error);
        }
    }

    crossroads::SimulatorEngine engine(initial_config, traffic_rate, kNorthSouthDuration, kEastWestDuration);
    std::mutex engine_mutex;
    std::atomic<bool> app_running{true};
    std::optional<crossroads::IntersectionConfig> pending_config;
    std::optional<crossroads::TrafficGenerator::SpawnLaneFilter> active_spawn_filter;

    crossroads::SimpleHttpUiServer server(
        8080,
        [&]() {
            std::lock_guard<std::mutex> lock(engine_mutex);
            return engine.getSnapshotJson();
        },
        [&](const std::string& cmd) {
            std::lock_guard<std::mutex> lock(engine_mutex);

            auto applyPendingConfigIfNeeded = [&]() {
                if (!pending_config.has_value()) {
                    return;
                }
                engine =
                    crossroads::SimulatorEngine(*pending_config, traffic_rate, kNorthSouthDuration, kEastWestDuration);
                engine.setTrafficRate(traffic_rate);
                engine.setSpawnLaneFilter(active_spawn_filter);
                pending_config.reset();
            };

            if (cmd == "start") {
                if (!engine.isRunning()) {
                    applyPendingConfigIfNeeded();
                }
                engine.handleCommand(crossroads::SimulatorEngine::UICommand::Start);
            } else if (cmd == "stop")
                engine.handleCommand(crossroads::SimulatorEngine::UICommand::Stop);
            else if (cmd == "reset") {
                applyPendingConfigIfNeeded();
                engine.handleCommand(crossroads::SimulatorEngine::UICommand::Reset);
            } else if (cmd == "step")
                engine.handleCommand(crossroads::SimulatorEngine::UICommand::Step, 0.1);
            else if (cmd == "spawn_focus:all") {
                const bool was_running = engine.isRunning();
                active_spawn_filter.reset();
                engine.setSpawnLaneFilter(active_spawn_filter);
                engine.handleCommand(crossroads::SimulatorEngine::UICommand::Reset);
                if (was_running) {
                    engine.handleCommand(crossroads::SimulatorEngine::UICommand::Start);
                }
            } else if (cmd.rfind("spawn_focus:", 0) == 0) {
                const std::string payload = cmd.substr(std::string("spawn_focus:").size());
                std::istringstream parser(payload);
                std::string approach_text;
                std::string lane_text;
                if (std::getline(parser, approach_text, ':') && std::getline(parser, lane_text)) {
                    auto normalizeToken = [](std::string value) {
                        value.erase(value.begin(), std::find_if(value.begin(), value.end(), [](unsigned char ch) {
                                        return !std::isspace(ch);
                                    }));
                        value.erase(
                            std::find_if(
                                value.rbegin(), value.rend(), [](unsigned char ch) { return !std::isspace(ch); })
                                .base(),
                            value.end());
                        std::transform(value.begin(), value.end(), value.begin(), [](unsigned char ch) {
                            return static_cast<char>(std::tolower(ch));
                        });
                        return value;
                    };

                    approach_text = normalizeToken(approach_text);
                    lane_text = normalizeToken(lane_text);

                    crossroads::ApproachId approach = crossroads::ApproachId::North;
                    if (approach_text == "east")
                        approach = crossroads::ApproachId::East;
                    else if (approach_text == "south")
                        approach = crossroads::ApproachId::South;
                    else if (approach_text == "west")
                        approach = crossroads::ApproachId::West;

                    try {
                        int lane_index_raw = std::stoi(lane_text);
                        if (lane_index_raw >= 0) {
                            const bool was_running = engine.isRunning();
                            crossroads::TrafficGenerator::SpawnLaneFilter filter;
                            filter.approach = approach;
                            filter.lane_index = static_cast<uint16_t>(lane_index_raw);
                            active_spawn_filter = filter;
                            engine.setSpawnLaneFilter(active_spawn_filter);
                            engine.handleCommand(crossroads::SimulatorEngine::UICommand::Reset);
                            if (was_running) {
                                engine.handleCommand(crossroads::SimulatorEngine::UICommand::Start);
                            }
                        }
                    } catch (...) {
                    }
                }
            } else if (cmd.rfind("spawn_rate:", 0) == 0) {
                const std::string rate_text = cmd.substr(std::string("spawn_rate:").size());
                try {
                    const double parsed = std::stod(rate_text);
                    traffic_rate = std::max(0.0, parsed);
                    engine.setTrafficRate(traffic_rate);
                } catch (...) {
                }
            }
        },
        [&]() {
            std::lock_guard<std::mutex> lock(engine_mutex);
            if (pending_config.has_value()) {
                return crossroads::intersectionConfigToJson(*pending_config);
            }
            return crossroads::intersectionConfigToJson(engine.getIntersectionConfig());
        },
        [&](const std::string& body) {
            std::lock_guard<std::mutex> lock(engine_mutex);

            nlohmann::json request = nlohmann::json::parse(body, nullptr, false);
            if (request.is_object() && request.contains("action") && request["action"].is_string()) {
                const std::string action = request["action"].get<std::string>();
                std::string error;

                if (action == "list_named") {
                    const auto entries = database.listNamedIntersectionConfigs(&error);
                    if (!error.empty()) {
                        return crossroads::SimpleHttpUiServer::ConfigMutationResult{
                            500, crossroads::validationErrorsToJson({"database error: " + error})};
                    }

                    nlohmann::json resp;
                    resp["ok"] = true;
                    resp["items"] = nlohmann::json::array();
                    for (const auto& entry : entries) {
                        resp["items"].push_back({{"name", entry.name},
                                                 {"updated_at", entry.updated_at},
                                                 {"last_used_at", entry.last_used_at}});
                    }
                    return crossroads::SimpleHttpUiServer::ConfigMutationResult{200, resp.dump()};
                }

                if (action == "last_used_name") {
                    nlohmann::json resp;
                    resp["ok"] = true;
                    if (auto name = database.loadMostRecentNamedConfigName(&error); name.has_value()) {
                        resp["name"] = *name;
                    } else {
                        resp["name"] = "";
                    }
                    return crossroads::SimpleHttpUiServer::ConfigMutationResult{200, resp.dump()};
                }

                if (action == "load_named") {
                    if (!request.contains("name") || !request["name"].is_string()) {
                        return crossroads::SimpleHttpUiServer::ConfigMutationResult{
                            400, crossroads::validationErrorsToJson({"name is required"})};
                    }
                    const std::string name = trimCopy(request["name"].get<std::string>());
                    if (name.empty()) {
                        return crossroads::SimpleHttpUiServer::ConfigMutationResult{
                            400, crossroads::validationErrorsToJson({"name is required"})};
                    }

                    auto json_text = database.loadNamedIntersectionConfigJson(name, &error);
                    if (!json_text.has_value()) {
                        const std::string msg =
                            !error.empty() ? ("database error: " + error) : "named configuration not found";
                        return crossroads::SimpleHttpUiServer::ConfigMutationResult{
                            404, crossroads::validationErrorsToJson({msg})};
                    }

                    crossroads::ConfigParseResult parsed = crossroads::intersectionConfigFromJson(*json_text);
                    crossroads::SafetyChecker checker(parsed.config);
                    if (!parsed.ok || !checker.isConfigValid()) {
                        return crossroads::SimpleHttpUiServer::ConfigMutationResult{
                            400, crossroads::validationErrorsToJson({"stored config is invalid"})};
                    }

                    database.touchNamedIntersectionConfig(name, &error);

                    nlohmann::json resp;
                    resp["ok"] = true;
                    resp["name"] = name;
                    resp["config"] = nlohmann::json::parse(crossroads::intersectionConfigToJson(parsed.config));
                    return crossroads::SimpleHttpUiServer::ConfigMutationResult{200, resp.dump()};
                }

                if (action == "save_named") {
                    if (!request.contains("name") || !request["name"].is_string()) {
                        return crossroads::SimpleHttpUiServer::ConfigMutationResult{
                            400, crossroads::validationErrorsToJson({"name is required"})};
                    }
                    if (!request.contains("config")) {
                        return crossroads::SimpleHttpUiServer::ConfigMutationResult{
                            400, crossroads::validationErrorsToJson({"config is required"})};
                    }

                    const std::string name = trimCopy(request["name"].get<std::string>());
                    if (name.empty()) {
                        return crossroads::SimpleHttpUiServer::ConfigMutationResult{
                            400, crossroads::validationErrorsToJson({"name is required"})};
                    }

                    std::string config_json;
                    if (request["config"].is_string()) {
                        config_json = request["config"].get<std::string>();
                    } else {
                        config_json = request["config"].dump();
                    }

                    crossroads::ConfigParseResult parsed = crossroads::intersectionConfigFromJson(config_json);
                    if (!parsed.ok) {
                        return crossroads::SimpleHttpUiServer::ConfigMutationResult{
                            400, crossroads::validationErrorsToJson(parsed.errors)};
                    }

                    crossroads::SafetyChecker checker(parsed.config);
                    if (!checker.isConfigValid()) {
                        return crossroads::SimpleHttpUiServer::ConfigMutationResult{
                            400, crossroads::validationErrorsToJson({"config failed safety validation rules"})};
                    }

                    const std::string normalized_json = crossroads::intersectionConfigToJson(parsed.config);
                    if (!database.saveNamedIntersectionConfigJson(name, normalized_json, &error)) {
                        return crossroads::SimpleHttpUiServer::ConfigMutationResult{
                            500, crossroads::validationErrorsToJson({"database error: " + error})};
                    }
                    database.touchNamedIntersectionConfig(name, &error);

                    nlohmann::json resp;
                    resp["ok"] = true;
                    resp["name"] = name;
                    return crossroads::SimpleHttpUiServer::ConfigMutationResult{200, resp.dump()};
                }

                if (action == "delete_named") {
                    if (!request.contains("name") || !request["name"].is_string()) {
                        return crossroads::SimpleHttpUiServer::ConfigMutationResult{
                            400, crossroads::validationErrorsToJson({"name is required"})};
                    }

                    const std::string name = trimCopy(request["name"].get<std::string>());
                    if (name.empty()) {
                        return crossroads::SimpleHttpUiServer::ConfigMutationResult{
                            400, crossroads::validationErrorsToJson({"name is required"})};
                    }

                    auto existing = database.loadNamedIntersectionConfigJson(name, &error);
                    if (!existing.has_value()) {
                        const std::string msg =
                            !error.empty() ? ("database error: " + error) : "named configuration not found";
                        return crossroads::SimpleHttpUiServer::ConfigMutationResult{
                            404, crossroads::validationErrorsToJson({msg})};
                    }

                    if (!database.deleteNamedIntersectionConfig(name, &error)) {
                        return crossroads::SimpleHttpUiServer::ConfigMutationResult{
                            500, crossroads::validationErrorsToJson({"database error: " + error})};
                    }

                    nlohmann::json resp;
                    resp["ok"] = true;
                    resp["name"] = name;
                    return crossroads::SimpleHttpUiServer::ConfigMutationResult{200, resp.dump()};
                }

                return crossroads::SimpleHttpUiServer::ConfigMutationResult{
                    400, crossroads::validationErrorsToJson({"unknown action"})};
            }

            crossroads::ConfigParseResult parsed = crossroads::intersectionConfigFromJson(body);
            if (!parsed.ok) {
                return crossroads::SimpleHttpUiServer::ConfigMutationResult{
                    400, crossroads::validationErrorsToJson(parsed.errors)};
            }

            crossroads::SafetyChecker checker(parsed.config);
            if (!checker.isConfigValid()) {
                return crossroads::SimpleHttpUiServer::ConfigMutationResult{
                    400, crossroads::validationErrorsToJson({"config failed safety validation rules"})};
            }

            const std::string normalized_json = crossroads::intersectionConfigToJson(parsed.config);
            std::string error;
            if (!database.saveActiveIntersectionConfigJson(normalized_json, &error)) {
                return crossroads::SimpleHttpUiServer::ConfigMutationResult{
                    500, crossroads::validationErrorsToJson({"database error: " + error})};
            }

            pending_config = parsed.config;
            return crossroads::SimpleHttpUiServer::ConfigMutationResult{
                200, "{\"ok\":true,\"state\":\"pending\",\"apply_on\":\"start_or_reset\"}"};
        });

    if (!server.start()) {
        std::cerr << "Failed to start UI server on port 8080" << std::endl;
        return 1;
    }

    std::thread sim_thread([&]() {
        while (app_running) {
            {
                std::lock_guard<std::mutex> lock(engine_mutex);
                engine.tick(0.1);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    });

    std::cout << "Open UI at: http://localhost:8080" << std::endl;
    std::cout << "Press Ctrl+C to stop server..." << std::endl;

    while (g_keep_running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    app_running = false;
    if (sim_thread.joinable()) {
        sim_thread.join();
    }
    server.stop();

    std::cout << "UI server stopped." << std::endl;
    return 0;
}
