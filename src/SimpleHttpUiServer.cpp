#include "SimpleHttpUiServer.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstring>
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>

namespace crossroads
{
    namespace
    {
        std::string decodePath(const std::string &path)
        {
            if (path == "/" || path == "/index.html")
            {
                return "index";
            }
            if (path.rfind("/snapshot", 0) == 0)
            {
                return "snapshot";
            }
            if (path.rfind("/command", 0) == 0)
            {
                return "command";
            }
            if (path == "/config" || path == "/config/")
            {
                return "config_page";
            }
            if (path.rfind("/config/api", 0) == 0 || path == "/config.json")
            {
                return "config_api";
            }
            return "unknown";
        }

        std::string statusTextFromCode(int status_code)
        {
            switch (status_code)
            {
            case 200:
                return "200 OK";
            case 400:
                return "400 Bad Request";
            case 404:
                return "404 Not Found";
            case 405:
                return "405 Method Not Allowed";
            case 500:
                return "500 Internal Server Error";
            default:
                return std::to_string(status_code) + " Unknown";
            }
        }

        std::string extractCmd(const std::string &path)
        {
            std::size_t q = path.find("cmd=");
            if (q == std::string::npos)
            {
                return "";
            }
            std::string cmd = path.substr(q + 4);
            std::size_t amp = cmd.find('&');
            if (amp != std::string::npos)
            {
                cmd = cmd.substr(0, amp);
            }
            return cmd;
        }

        std::string readFileIfExists(const std::string &path)
        {
            std::ifstream in(path, std::ios::in | std::ios::binary);
            if (!in)
            {
                return "";
            }
            std::ostringstream out;
            out << in.rdbuf();
            return out.str();
        }

        std::string readWebFile(const std::string &relative_path)
        {
            if (relative_path.empty() || relative_path.find("..") != std::string::npos)
            {
                return "";
            }

            const std::vector<std::string> roots = {"./web/", "../web/"};
            for (const auto &root : roots)
            {
                std::string body = readFileIfExists(root + relative_path);
                if (!body.empty())
                {
                    return body;
                }
            }
            return "";
        }

        std::string contentTypeForPath(const std::string &path)
        {
            if (path.size() >= 5 && path.substr(path.size() - 5) == ".html")
            {
                return "text/html; charset=utf-8";
            }
            if (path.size() >= 4 && path.substr(path.size() - 4) == ".css")
            {
                return "text/css; charset=utf-8";
            }
            if (path.size() >= 3 && path.substr(path.size() - 3) == ".js")
            {
                return "application/javascript; charset=utf-8";
            }
            if (path.size() >= 5 && path.substr(path.size() - 5) == ".json")
            {
                return "application/json; charset=utf-8";
            }
            return "text/plain; charset=utf-8";
        }

        const char *kIndexHtml = R"HTML(
<!doctype html>
<html lang="en"><head><meta charset="UTF-8" /><title>Crossroads UI</title></head>
<body><p>UI assets missing. Expected web/index.html and web/assets/index.css.</p></body></html>
)HTML";

        const char *kConfigHtml = R"HTML(
<!doctype html>
<html lang="en"><head><meta charset="UTF-8" /><title>Crossroads Config</title></head>
<body><p>Config assets missing. Expected web/config.html and web/assets/config.css.</p></body></html>
)HTML";
    } // namespace

    SimpleHttpUiServer::SimpleHttpUiServer(int port,
                                           SnapshotProvider snapshot_provider,
                                           CommandHandler command_handler,
                                           ConfigProvider config_provider,
                                           ConfigMutationHandler config_mutation_handler)
        : port(port),
          server_fd(-1),
          running(false),
          snapshot_provider(std::move(snapshot_provider)),
          command_handler(std::move(command_handler)),
          config_provider(std::move(config_provider)),
          config_mutation_handler(std::move(config_mutation_handler))
    {
    }

    SimpleHttpUiServer::~SimpleHttpUiServer()
    {
        stop();
    }

    bool SimpleHttpUiServer::start()
    {
        server_fd = socket(AF_INET, SOCK_STREAM, 0);
        if (server_fd < 0)
        {
            std::cerr << "UI server: failed to create socket\n";
            return false;
        }

        int opt = 1;
        setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = htonl(INADDR_ANY);
        addr.sin_port = htons(static_cast<uint16_t>(port));

        if (bind(server_fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0)
        {
            std::cerr << "UI server: bind failed on port " << port << "\n";
            close(server_fd);
            server_fd = -1;
            return false;
        }

        if (listen(server_fd, 16) < 0)
        {
            std::cerr << "UI server: listen failed\n";
            close(server_fd);
            server_fd = -1;
            return false;
        }

        running = true;
        accept_thread = std::thread(&SimpleHttpUiServer::acceptLoop, this);
        return true;
    }

    void SimpleHttpUiServer::stop()
    {
        if (!running)
        {
            return;
        }

        running = false;
        if (server_fd >= 0)
        {
            shutdown(server_fd, SHUT_RDWR);
            close(server_fd);
            server_fd = -1;
        }

        if (accept_thread.joinable())
        {
            accept_thread.join();
        }
    }

    void SimpleHttpUiServer::acceptLoop()
    {
        while (running)
        {
            sockaddr_in client_addr{};
            socklen_t len = sizeof(client_addr);
            int client_fd = accept(server_fd, reinterpret_cast<sockaddr *>(&client_addr), &len);
            if (client_fd < 0)
            {
                if (running)
                {
                    continue;
                }
                break;
            }

            handleClient(client_fd);
            close(client_fd);
        }
    }

    void SimpleHttpUiServer::handleClient(int client_fd)
    {
        char buffer[8192];
        std::memset(buffer, 0, sizeof(buffer));
        int n = recv(client_fd, buffer, sizeof(buffer) - 1, 0);
        if (n <= 0)
        {
            return;
        }

        std::string req(buffer, n);
        std::istringstream input(req);
        std::string method, path, version;
        input >> method >> path >> version;
        std::string body;
        std::size_t body_start = req.find("\r\n\r\n");
        if (body_start != std::string::npos)
        {
            body = req.substr(body_start + 4);
        }

        std::size_t qmark = path.find('?');
        std::string clean_path = qmark == std::string::npos ? path : path.substr(0, qmark);

        if (method == "GET" && clean_path.rfind("/assets/", 0) == 0)
        {
            const std::string rel = clean_path[0] == '/' ? clean_path.substr(1) : clean_path;
            std::string asset = readWebFile(rel);
            if (asset.empty())
            {
                std::string resp = buildHttpResponse("404 Not Found", "text/plain", "not found");
                send(client_fd, resp.c_str(), resp.size(), 0);
                return;
            }

            std::string resp = buildHttpResponse("200 OK", contentTypeForPath(rel), asset);
            send(client_fd, resp.c_str(), resp.size(), 0);
            return;
        }

        std::string route = decodePath(clean_path);
        if (route == "index")
        {
            std::string body = readWebFile("index.html");
            if (body.empty())
            {
                body = kIndexHtml;
            }
            std::string resp = buildHttpResponse("200 OK", "text/html; charset=utf-8", body);
            send(client_fd, resp.c_str(), resp.size(), 0);
            return;
        }

        if (route == "snapshot")
        {
            std::string body = snapshot_provider();
            std::string resp = buildHttpResponse("200 OK", "application/json", body);
            send(client_fd, resp.c_str(), resp.size(), 0);
            return;
        }

        if (route == "command")
        {
            std::string cmd = extractCmd(path);
            command_handler(cmd);
            std::string resp = buildHttpResponse("200 OK", "text/plain", "ok");
            send(client_fd, resp.c_str(), resp.size(), 0);
            return;
        }

        if (route == "config_page")
        {
            if (method != "GET")
            {
                std::string resp = buildHttpResponse("405 Method Not Allowed", "application/json", "{\"ok\":false,\"error\":\"method not allowed\"}");
                send(client_fd, resp.c_str(), resp.size(), 0);
                return;
            }

            std::string page = readWebFile("config.html");
            if (page.empty())
            {
                page = kConfigHtml;
            }
            std::string resp = buildHttpResponse("200 OK", "text/html; charset=utf-8", page);
            send(client_fd, resp.c_str(), resp.size(), 0);
            return;
        }

        if (route == "config_api")
        {
            if (method == "GET")
            {
                std::string config_json = config_provider();
                std::string resp = buildHttpResponse("200 OK", "application/json", config_json);
                send(client_fd, resp.c_str(), resp.size(), 0);
                return;
            }

            if (method == "POST")
            {
                ConfigMutationResult result = config_mutation_handler(body);
                std::string status = statusTextFromCode(result.status_code);
                std::string resp = buildHttpResponse(status, "application/json", result.body);
                send(client_fd, resp.c_str(), resp.size(), 0);
                return;
            }

            std::string resp = buildHttpResponse("405 Method Not Allowed", "application/json", "{\"ok\":false,\"error\":\"method not allowed\"}");
            send(client_fd, resp.c_str(), resp.size(), 0);
            return;
        }

        std::string not_found = buildHttpResponse("404 Not Found", "text/plain", "not found");
        send(client_fd, not_found.c_str(), not_found.size(), 0);
    }

    std::string SimpleHttpUiServer::buildHttpResponse(const std::string &status,
                                                      const std::string &content_type,
                                                      const std::string &body) const
    {
        std::ostringstream out;
        out << "HTTP/1.1 " << status << "\r\n";
        out << "Content-Type: " << content_type << "\r\n";
        out << "Cache-Control: no-store, no-cache, must-revalidate, max-age=0\r\n";
        out << "Pragma: no-cache\r\n";
        out << "Expires: 0\r\n";
        out << "Content-Length: " << body.size() << "\r\n";
        out << "Connection: close\r\n\r\n";
        out << body;
        return out.str();
    }
} // namespace crossroads
