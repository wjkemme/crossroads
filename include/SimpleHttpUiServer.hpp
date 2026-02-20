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
        using SnapshotProvider = std::function<std::string()>;
        using CommandHandler = std::function<void(const std::string &)>;

        SimpleHttpUiServer(int port,
                           SnapshotProvider snapshot_provider,
                           CommandHandler command_handler);
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
    };
} // namespace crossroads
