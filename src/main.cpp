#include <iostream>
#include <atomic>
#include <csignal>
#include <chrono>
#include <mutex>
#include <thread>
#include <optional>
#include <vector>
#include "SimulatorEngine.hpp"
#include "SimpleHttpUiServer.hpp"
#include "IntersectionConfigJson.hpp"
#include "SafetyChecker.hpp"
#include "db/Database.hpp"

namespace
{
    std::atomic<bool> g_keep_running{true};

    void handleSignal(int)
    {
        g_keep_running = false;
    }
}

int main()
{
    std::signal(SIGINT, handleSignal);
    std::signal(SIGTERM, handleSignal);

    std::cout << "=== Crossroads Traffic Simulator UI ===" << std::endl;
    std::cout << std::endl;

    constexpr double kTrafficRate = 0.8;
    constexpr double kNorthSouthDuration = 10.0;
    constexpr double kEastWestDuration = 10.0;

    crossroads::db::Database database("crossroads.db");
    std::string db_error;
    if (!database.initialize(&db_error))
    {
        std::cerr << "Warning: failed to initialize config database: " << db_error << std::endl;
    }

    crossroads::IntersectionConfig initial_config = crossroads::makeDefaultIntersectionConfig();
    if (auto stored = database.loadActiveIntersectionConfigJson(&db_error); stored.has_value())
    {
        crossroads::ConfigParseResult parsed = crossroads::intersectionConfigFromJson(*stored);
        crossroads::SafetyChecker checker(parsed.config);
        if (parsed.ok && checker.isConfigValid())
        {
            initial_config = parsed.config;
        }
        else
        {
            std::cerr << "Warning: stored config is invalid, using defaults" << std::endl;
        }
    }
    else if (!db_error.empty())
    {
        std::cerr << "Warning: failed to load config from database: " << db_error << std::endl;
    }
    else
    {
        database.saveActiveIntersectionConfigJson(crossroads::intersectionConfigToJson(initial_config), &db_error);
    }

    crossroads::SimulatorEngine engine(initial_config, kTrafficRate, kNorthSouthDuration, kEastWestDuration);
    std::mutex engine_mutex;
    std::atomic<bool> app_running{true};
    std::optional<crossroads::IntersectionConfig> pending_config;

    crossroads::SimpleHttpUiServer server(
        8080,
        [&]()
        {
            std::lock_guard<std::mutex> lock(engine_mutex);
            return engine.getSnapshotJson();
        },
        [&](const std::string &cmd)
        {
            std::lock_guard<std::mutex> lock(engine_mutex);

            auto applyPendingConfigIfNeeded = [&]()
            {
                if (!pending_config.has_value())
                {
                    return;
                }
                engine = crossroads::SimulatorEngine(*pending_config, kTrafficRate, kNorthSouthDuration, kEastWestDuration);
                pending_config.reset();
            };

            if (cmd == "start")
            {
                if (!engine.isRunning())
                {
                    applyPendingConfigIfNeeded();
                }
                engine.handleCommand(crossroads::SimulatorEngine::UICommand::Start);
            }
            else if (cmd == "stop")
                engine.handleCommand(crossroads::SimulatorEngine::UICommand::Stop);
            else if (cmd == "reset")
            {
                applyPendingConfigIfNeeded();
                engine.handleCommand(crossroads::SimulatorEngine::UICommand::Reset);
            }
            else if (cmd == "step")
                engine.handleCommand(crossroads::SimulatorEngine::UICommand::Step, 0.1);
        },
        [&]()
        {
            std::lock_guard<std::mutex> lock(engine_mutex);
            if (pending_config.has_value())
            {
                return crossroads::intersectionConfigToJson(*pending_config);
            }
            return crossroads::intersectionConfigToJson(engine.getIntersectionConfig());
        },
        [&](const std::string &body)
        {
            std::lock_guard<std::mutex> lock(engine_mutex);

            crossroads::ConfigParseResult parsed = crossroads::intersectionConfigFromJson(body);
            if (!parsed.ok)
            {
                return crossroads::SimpleHttpUiServer::ConfigMutationResult{400, crossroads::validationErrorsToJson(parsed.errors)};
            }

            crossroads::SafetyChecker checker(parsed.config);
            if (!checker.isConfigValid())
            {
                return crossroads::SimpleHttpUiServer::ConfigMutationResult{400, crossroads::validationErrorsToJson({"config failed safety validation rules"})};
            }

            const std::string normalized_json = crossroads::intersectionConfigToJson(parsed.config);
            std::string error;
            if (!database.saveActiveIntersectionConfigJson(normalized_json, &error))
            {
                return crossroads::SimpleHttpUiServer::ConfigMutationResult{500, crossroads::validationErrorsToJson({"database error: " + error})};
            }

            pending_config = parsed.config;
            return crossroads::SimpleHttpUiServer::ConfigMutationResult{200, "{\"ok\":true,\"state\":\"pending\",\"apply_on\":\"start_or_reset\"}"};
        });

    if (!server.start())
    {
        std::cerr << "Failed to start UI server on port 8080" << std::endl;
        return 1;
    }

    std::thread sim_thread([&]()
                           {
        while (app_running)
        {
            {
                std::lock_guard<std::mutex> lock(engine_mutex);
                engine.tick(0.1);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        } });

    std::cout << "Open UI at: http://localhost:8080" << std::endl;
    std::cout << "Press Ctrl+C to stop server..." << std::endl;

    while (g_keep_running)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    app_running = false;
    if (sim_thread.joinable())
    {
        sim_thread.join();
    }
    server.stop();

    std::cout << "UI server stopped." << std::endl;
    return 0;
}
