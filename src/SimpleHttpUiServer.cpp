#include "SimpleHttpUiServer.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstring>
#include <iostream>
#include <sstream>

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
            return "unknown";
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

        const char *kIndexHtml = R"HTML(
<!doctype html>
<html lang="en">
<head>
  <meta charset="UTF-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1.0" />
  <title>Crossroads UI</title>
  <style>
    :root { color-scheme: dark; }
    body { font-family: Arial, sans-serif; margin: 0; background: #111827; color: #e5e7eb; }
    .wrap { max-width: 980px; margin: 24px auto; padding: 0 16px; }
    .row { display: flex; gap: 12px; flex-wrap: wrap; }
    .card { background: #1f2937; border: 1px solid #374151; border-radius: 10px; padding: 12px; }
    .kpi { min-width: 180px; }
    button { background: #2563eb; color: white; border: 0; border-radius: 8px; padding: 8px 12px; cursor: pointer; }
    button:hover { background: #1d4ed8; }
    .cross { margin-top: 14px; display: grid; grid-template-columns: 1fr 1fr 1fr; grid-template-rows: 1fr 1fr 1fr; gap: 8px; }
    .cell { background: #1f2937; border: 1px solid #374151; border-radius: 10px; min-height: 90px; display: flex; align-items: center; justify-content: center; }
    .light { width: 20px; height: 20px; border-radius: 999px; border: 2px solid #9ca3af; margin-left: 8px; display: inline-block; }
    .red { background: #ef4444; }
    .orange { background: #f59e0b; }
    .green { background: #22c55e; }
    .off { background: #374151; }
    .small { font-size: 12px; color: #9ca3af; }
  </style>
</head>
<body>
  <div class="wrap">
    <h1>Crossroads Simulator</h1>
    <div class="row">
      <button onclick="cmd('start')">Start</button>
      <button onclick="cmd('stop')">Stop</button>
      <button onclick="cmd('reset')">Reset</button>
      <button onclick="cmd('step')">Step</button>
      <div class="small" id="status">status</div>
    </div>

    <div class="row" style="margin-top: 12px;">
      <div class="card kpi">Time: <b id="time">0</b></div>
      <div class="card kpi">Generated: <b id="gen">0</b></div>
      <div class="card kpi">Crossed: <b id="crossed">0</b></div>
      <div class="card kpi">Avg Wait: <b id="wait">0</b></div>
      <div class="card kpi">Safety Violations: <b id="viol">0</b></div>
      <div class="card kpi">Queue Total: <b id="qtotal">0</b></div>
      <div class="card kpi">Queue N/E/S/W: <b id="qnesw">0/0/0/0</b></div>
    </div>

    <div class="cross">
      <div class="cell"></div>
      <div class="cell">North <span class="light off" id="north"></span></div>
      <div class="cell"></div>
      <div class="cell">West <span class="light off" id="west"></span></div>
      <div class="cell">Center</div>
      <div class="cell">East <span class="light off" id="east"></span></div>
      <div class="cell"></div>
      <div class="cell">South <span class="light off" id="south"></span></div>
      <div class="cell"></div>
    </div>

    <div class="row" style="margin-top: 10px;">
      <div class="card">Turn S→E <span class="light off" id="turnSouthEast"></span></div>
      <div class="card">Turn N→W <span class="light off" id="turnNorthWest"></span></div>
      <div class="card">Turn W→S <span class="light off" id="turnWestSouth"></span></div>
      <div class="card">Turn E→N <span class="light off" id="turnEastNorth"></span></div>
    </div>
  </div>

  <script>
    function setLight(id, value) {
      const el = document.getElementById(id);
      el.className = 'light ' + (value || 'off');
    }

    async function cmd(name) {
      await fetch(`/command?cmd=${name}`);
      await refresh();
    }

    async function refresh() {
      const res = await fetch('/snapshot');
      const s = await res.json();

      document.getElementById('status').textContent = s.running ? 'running' : 'stopped';
      document.getElementById('time').textContent = s.sim_time.toFixed(1) + 's';
      document.getElementById('gen').textContent = s.metrics.vehicles_generated;
      document.getElementById('crossed').textContent = s.metrics.vehicles_crossed;
      document.getElementById('wait').textContent = Number(s.metrics.average_wait_time).toFixed(2) + 's';
      document.getElementById('viol').textContent = s.metrics.safety_violations;
      const q = s.metrics.queues;
      const total = q.north + q.east + q.south + q.west;
      document.getElementById('qtotal').textContent = total;
      document.getElementById('qnesw').textContent = `${q.north}/${q.east}/${q.south}/${q.west}`;

      setLight('north', s.lights.north);
      setLight('east', s.lights.east);
      setLight('south', s.lights.south);
      setLight('west', s.lights.west);
      setLight('turnSouthEast', s.lights.turnSouthEast);
      setLight('turnNorthWest', s.lights.turnNorthWest);
      setLight('turnWestSouth', s.lights.turnWestSouth);
      setLight('turnEastNorth', s.lights.turnEastNorth);
    }

    refresh();
    setInterval(refresh, 200);
  </script>
</body>
</html>
)HTML";
    } // namespace

    SimpleHttpUiServer::SimpleHttpUiServer(int port,
                                           SnapshotProvider snapshot_provider,
                                           CommandHandler command_handler)
        : port(port),
          server_fd(-1),
          running(false),
          snapshot_provider(std::move(snapshot_provider)),
          command_handler(std::move(command_handler))
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

        std::string route = decodePath(path);
        if (route == "index")
        {
            std::string body = kIndexHtml;
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
        out << "Content-Length: " << body.size() << "\r\n";
        out << "Connection: close\r\n\r\n";
        out << body;
        return out.str();
    }
} // namespace crossroads
