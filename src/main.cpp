#include <iostream>
#include <atomic>
#include <csignal>
#include <chrono>
#include <mutex>
#include <thread>
#include "SimulatorEngine.hpp"
#include "SimpleHttpUiServer.hpp"

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

    crossroads::SimulatorEngine engine(0.8, 10.0, 10.0);
    std::mutex engine_mutex;
    std::atomic<bool> app_running{true};

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
            if (cmd == "start")
                engine.handleCommand(crossroads::SimulatorEngine::UICommand::Start);
            else if (cmd == "stop")
                engine.handleCommand(crossroads::SimulatorEngine::UICommand::Stop);
            else if (cmd == "reset")
                engine.handleCommand(crossroads::SimulatorEngine::UICommand::Reset);
            else if (cmd == "step")
                engine.handleCommand(crossroads::SimulatorEngine::UICommand::Step, 0.1);
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
