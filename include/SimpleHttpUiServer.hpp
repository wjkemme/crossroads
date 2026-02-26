#pragma once

#include <atomic>
#include <functional>
#include <string>
#include <thread>

namespace crossroads
{
    class SimpleHttpUiServer
    {
    public:
        struct ConfigMutationResult
        {
            int status_code = 200;
            std::string body;
        };

        using SnapshotProvider = std::function<std::string()>;
        using CommandHandler = std::function<void(const std::string &)>;
        using ConfigProvider = std::function<std::string()>;
        using ConfigMutationHandler = std::function<ConfigMutationResult(const std::string &)>;

        SimpleHttpUiServer(int port,
                           SnapshotProvider snapshot_provider,
                           CommandHandler command_handler,
                           ConfigProvider config_provider,
                           ConfigMutationHandler config_mutation_handler);
        ~SimpleHttpUiServer();

        bool start();
        void stop();

    private:
        void acceptLoop();
        void handleClient(int client_fd);
        std::string buildHttpResponse(const std::string &status,
                                      const std::string &content_type,
                                      const std::string &body) const;

        int port;
        int server_fd;
        std::atomic<bool> running;
        std::thread accept_thread;
        SnapshotProvider snapshot_provider;
        CommandHandler command_handler;
        ConfigProvider config_provider;
        ConfigMutationHandler config_mutation_handler;
    };
} // namespace crossroads
