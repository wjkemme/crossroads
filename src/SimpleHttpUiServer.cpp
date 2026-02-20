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
        .wrap { max-width: 1160px; margin: 20px auto; padding: 0 16px; }
        .row { display: flex; gap: 12px; flex-wrap: wrap; }
        .card { background: #1f2937; border: 1px solid #374151; border-radius: 10px; padding: 12px; }
        .kpi { min-width: 180px; }
        button { background: #2563eb; color: white; border: 0; border-radius: 8px; padding: 8px 12px; cursor: pointer; }
        button:hover { background: #1d4ed8; }
        .small { font-size: 12px; color: #9ca3af; }

        .light { width: 18px; height: 18px; border-radius: 999px; border: 2px solid #9ca3af; margin-left: 8px; display: inline-block; vertical-align: middle; }
        .red { background: #ef4444; }
        .orange { background: #f59e0b; }
        .green { background: #22c55e; }
        .off { background: #374151; }

        .road-wrap {
            margin-top: 14px;
            position: relative;
            height: 620px;
            border-radius: 14px;
            border: 1px solid #374151;
            background: #0f172a;
            overflow: hidden;
        }
        .road-h { position: absolute; left: 0; right: 0; top: 50%; height: 100px; transform: translateY(-50%); background: #111827; border-top: 2px solid #475569; border-bottom: 2px solid #475569; }
        .road-v { position: absolute; top: 0; bottom: 0; left: 50%; width: 100px; transform: translateX(-50%); background: #111827; border-left: 2px solid #475569; border-right: 2px solid #475569; }
        .junction { position: absolute; width: 100px; height: 100px; left: 50%; top: 50%; transform: translate(-50%, -50%); background: rgba(30, 41, 59, 0.45); border: 2px dashed #334155; }

        .center-line-h { position: absolute; left: 0; right: 0; top: 50%; height: 2px; transform: translateY(-1px); background: #fbbf24; z-index: 3; }
        .center-line-v { position: absolute; top: 0; bottom: 0; left: 50%; width: 2px; transform: translateX(-1px); background: #fbbf24; z-index: 3; }

        .lane {
            position: absolute;
            background: rgba(148, 163, 184, 0.08);
            border: 1px solid rgba(148, 163, 184, 0.2);
            overflow: visible;
        }

        .lane.west-east {
            left: 0; right: 0; top: 50%; height: 48px;
            background: linear-gradient(to bottom,
                rgba(148, 163, 184, 0.10) 0 16px,
                rgba(148, 163, 184, 0.10) 16px 32px,
                rgba(245, 158, 11, 0.20) 32px 48px);
        }
        .lane.east-west {
            left: 0; right: 0; top: calc(50% - 48px); height: 48px;
            background: linear-gradient(to bottom,
                rgba(245, 158, 11, 0.20) 0 16px,
                rgba(148, 163, 184, 0.10) 16px 32px,
                rgba(148, 163, 184, 0.10) 32px 48px);
        }
        .lane.north-south {
            top: 0; bottom: 0; left: calc(50% - 48px); width: 48px;
            background: linear-gradient(to right,
                rgba(245, 158, 11, 0.20) 0 16px,
                rgba(148, 163, 184, 0.10) 16px 32px,
                rgba(148, 163, 184, 0.10) 32px 48px);
        }
        .lane.south-north {
            top: 0; bottom: 0; left: 50%; width: 48px;
            background: linear-gradient(to right,
                rgba(148, 163, 184, 0.10) 0 16px,
                rgba(148, 163, 184, 0.10) 16px 32px,
                rgba(245, 158, 11, 0.20) 32px 48px);
        }

        .stop-line { position: absolute; background: #f8fafc; opacity: 0.95; z-index: 5; }
        .stop-line.west-east { left: calc(50% - 52px); top: 50%; width: 3px; height: 48px; }
        .stop-line.east-west { left: calc(50% + 50px); top: calc(50% - 48px); width: 3px; height: 48px; }
        .stop-line.north-south { left: calc(50% - 48px); top: calc(50% - 52px); width: 48px; height: 3px; }
        .stop-line.south-north { left: 50%; top: calc(50% + 50px); width: 48px; height: 3px; }

        .turn-cut {
            position: absolute;
            background: #111827;
            z-index: 4;
            opacity: 0.95;
        }
        .turn-cut.we { left: calc(50% - 50px); right: 0; top: calc(50% + 32px); height: 16px; }
        .turn-cut.ew { left: 0; right: calc(50% - 50px); top: calc(50% - 48px); height: 16px; }
        .turn-cut.ns { left: calc(50% - 48px); top: calc(50% - 50px); bottom: 0; width: 16px; }
        .turn-cut.sn { left: calc(50% + 32px); top: 0; bottom: calc(50% - 50px); width: 16px; }

        .turn-link {
            position: absolute;
            z-index: 4;
            pointer-events: none;
            border: 13px solid rgba(245, 158, 11, 0.30);
        }
        .turn-link.ws {
            left: calc(50% - 60px);
            top: calc(50% - 8px);
            width: 56px;
            height: 56px;
            border-top: 0;
            border-left: 0;
            border-radius: 0 0 56px 0;
        }
        .turn-link.en {
            left: calc(50% + 2px);
            top: calc(50% - 60px);
            width: 56px;
            height: 56px;
            border-right: 0;
            border-bottom: 0;
            border-radius: 56px 0 0 0;
        }
        .turn-link.nw {
            left: calc(50% - 60px);
            top: calc(50% - 60px);
            width: 56px;
            height: 56px;
            border-top: 0;
            border-right: 0;
            border-radius: 0 0 0 56px;
        }
        .turn-link.se {
            left: calc(50% + 2px);
            top: calc(50% - 8px);
            width: 56px;
            height: 56px;
            border-left: 0;
            border-bottom: 0;
            border-radius: 0 56px 0 0;
        }

        .dir-arrow {
            position: absolute;
            color: #93c5fd;
            font-size: 16px;
            font-weight: bold;
            letter-spacing: 2px;
            opacity: 0.85;
            user-select: none;
            pointer-events: none;
        }
        .arrow-we { left: calc(50% - 150px); top: calc(50% + 12px); }
        .arrow-ew { left: calc(50% + 70px); top: calc(50% - 36px); }
        .arrow-ns { left: calc(50% - 36px); top: calc(50% - 150px); }
        .arrow-sn { left: calc(50% + 12px); top: calc(50% + 70px); }

        .approach-signal {
            position: absolute;
            width: 11px;
            height: 11px;
            border-radius: 999px;
            border: 2px solid #cbd5e1;
            background: #374151;
            z-index: 6;
            box-shadow: 0 0 0 2px rgba(15, 23, 42, 0.9);
        }
        .approach-signal.green {
            background: #22c55e;
            border-color: #bbf7d0;
            box-shadow: 0 0 0 2px rgba(15, 23, 42, 0.9), 0 0 10px rgba(34, 197, 94, 0.85);
        }
        .approach-signal.orange {
            background: #f59e0b;
            border-color: #fde68a;
            box-shadow: 0 0 0 2px rgba(15, 23, 42, 0.9), 0 0 10px rgba(245, 158, 11, 0.85);
        }
        .approach-signal.red {
            background: #ef4444;
            border-color: #fecaca;
            box-shadow: 0 0 0 2px rgba(15, 23, 42, 0.9), 0 0 8px rgba(239, 68, 68, 0.65);
        }
        .sig-north-0 { left: calc(50% - 44px); top: calc(50% - 68px); }
        .sig-north-1 { left: calc(50% - 28px); top: calc(50% - 68px); }
        .sig-north-2 { left: calc(50% - 12px); top: calc(50% - 68px); }

        .sig-south-0 { left: calc(50% + 8px); top: calc(50% + 56px); }
        .sig-south-1 { left: calc(50% + 24px); top: calc(50% + 56px); }
        .sig-south-2 { left: calc(50% + 40px); top: calc(50% + 56px); }

        .sig-west-0  { left: calc(50% - 68px); top: calc(50% + 8px); }
        .sig-west-1  { left: calc(50% - 68px); top: calc(50% + 24px); }
        .sig-west-2  { left: calc(50% - 68px); top: calc(50% + 40px); }

        .sig-east-0  { left: calc(50% + 56px); top: calc(50% - 44px); }
        .sig-east-1  { left: calc(50% + 56px); top: calc(50% - 28px); }
        .sig-east-2  { left: calc(50% + 56px); top: calc(50% - 12px); }

        .lane::before, .lane::after {
            content: "";
            position: absolute;
            background: transparent;
        }
        .lane.west-east::before, .lane.west-east::after,
        .lane.east-west::before, .lane.east-west::after {
            left: 0; right: 0; height: 1px;
            background: repeating-linear-gradient(
                to right,
                rgba(226, 232, 240, 0.8) 0 5px,
                transparent 5px 10px
            );
        }
        .lane.west-east::before, .lane.east-west::before { top: 16px; }
        .lane.west-east::after, .lane.east-west::after { top: 32px; }

        .lane.north-south::before, .lane.north-south::after,
        .lane.south-north::before, .lane.south-north::after {
            top: 0; bottom: 0; width: 1px;
            background: repeating-linear-gradient(
                to bottom,
                rgba(226, 232, 240, 0.8) 0 5px,
                transparent 5px 10px
            );
        }
        .lane.north-south::before, .lane.south-north::before { left: 16px; }
        .lane.north-south::after, .lane.south-north::after { left: 32px; }

        .lane-label { position: absolute; font-size: 11px; color: #94a3b8; background: rgba(15, 23, 42, 0.65); padding: 2px 6px; border-radius: 6px; }
        .lbl-we { left: 12px; top: calc(50% + 52px); }
        .lbl-ew { right: 12px; top: calc(50% - 68px); }
        .lbl-ns { left: calc(50% - 148px); top: 12px; }
        .lbl-sn { left: calc(50% + 56px); bottom: 12px; }

        .lane-arrow {
            position: absolute;
            color: #bfdbfe;
            font-size: 13px;
            font-weight: 700;
            z-index: 7;
            user-select: none;
            pointer-events: none;
            text-shadow: 0 0 6px rgba(15, 23, 42, 1);
        }
        .turn-arrow {
            color: #fbbf24;
            font-size: 14px;
        }
        .arr-we-1 { left: calc(50% - 100px); top: calc(50% + 5px); }
        .arr-we-2 { left: calc(50% - 100px); top: calc(50% + 21px); }
        .arr-we-t { left: calc(50% - 100px); top: calc(50% + 37px); }

        .arr-ew-t { left: calc(50% + 80px); top: calc(50% - 43px); }
        .arr-ew-1 { left: calc(50% + 80px); top: calc(50% - 27px); }
        .arr-ew-2 { left: calc(50% + 80px); top: calc(50% - 11px); }

        .arr-ns-1 { left: calc(50% - 43px); top: calc(50% - 100px); }
        .arr-ns-2 { left: calc(50% - 27px); top: calc(50% - 100px); }
        .arr-ns-t { left: calc(50% - 11px); top: calc(50% - 100px); }

        .arr-sn-1 { left: calc(50% + 5px); top: calc(50% + 80px); }
        .arr-sn-2 { left: calc(50% + 21px); top: calc(50% + 80px); }
        .arr-sn-t { left: calc(50% + 37px); top: calc(50% + 80px); }

        .car {
            position: absolute;
            width: 12px;
            height: 7px;
            border-radius: 2px;
            background: #60a5fa;
            box-shadow: 0 0 0 2px rgba(30, 41, 59, 0.9);
        }
        .car.crossing { background: #a78bfa; }
        .car.turning {
            background: #f59e0b;
            animation: turnBlink 0.7s linear infinite;
        }

        @keyframes turnBlink {
            0% { opacity: 1; }
            50% { opacity: 0.45; }
            100% { opacity: 1; }
        }

        .lights-overlay { margin-top: 10px; display: flex; gap: 10px; flex-wrap: wrap; }
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

        <div class="road-wrap">
            <div class="road-h"></div>
            <div class="road-v"></div>
            <div class="center-line-h"></div>
            <div class="center-line-v"></div>
            <div class="junction"></div>

            <div class="lane west-east" id="lane-west"></div>
            <div class="lane east-west" id="lane-east"></div>
            <div class="lane north-south" id="lane-north"></div>
            <div class="lane south-north" id="lane-south"></div>

            <div class="turn-cut we"></div>
            <div class="turn-cut ew"></div>
            <div class="turn-cut ns"></div>
            <div class="turn-cut sn"></div>

            <div class="turn-link ws"></div>
            <div class="turn-link en"></div>
            <div class="turn-link nw"></div>
            <div class="turn-link se"></div>

            <div class="stop-line west-east"></div>
            <div class="stop-line east-west"></div>
            <div class="stop-line north-south"></div>
            <div class="stop-line south-north"></div>

            <div class="dir-arrow arrow-we">→ → →</div>
            <div class="dir-arrow arrow-ew">← ← ←</div>
            <div class="dir-arrow arrow-ns">↓ ↓ ↓</div>
            <div class="dir-arrow arrow-sn">↑ ↑ ↑</div>

            <div class="approach-signal sig-north-0" id="sig-north-0"></div>
            <div class="approach-signal sig-north-1" id="sig-north-1"></div>
            <div class="approach-signal sig-north-2" id="sig-north-2"></div>
            <div class="approach-signal sig-east-0" id="sig-east-0"></div>
            <div class="approach-signal sig-east-1" id="sig-east-1"></div>
            <div class="approach-signal sig-east-2" id="sig-east-2"></div>
            <div class="approach-signal sig-south-0" id="sig-south-0"></div>
            <div class="approach-signal sig-south-1" id="sig-south-1"></div>
            <div class="approach-signal sig-south-2" id="sig-south-2"></div>
            <div class="approach-signal sig-west-0" id="sig-west-0"></div>
            <div class="approach-signal sig-west-1" id="sig-west-1"></div>
            <div class="approach-signal sig-west-2" id="sig-west-2"></div>

            <div class="lane-arrow arr-we-1">→</div>
            <div class="lane-arrow arr-we-2">→</div>
            <div class="lane-arrow turn-arrow arr-we-t">↘</div>

            <div class="lane-arrow turn-arrow arr-ew-t">↖</div>
            <div class="lane-arrow arr-ew-1">←</div>
            <div class="lane-arrow arr-ew-2">←</div>

            <div class="lane-arrow arr-ns-1">↓</div>
            <div class="lane-arrow arr-ns-2">↓</div>
            <div class="lane-arrow turn-arrow arr-ns-t">↙</div>

            <div class="lane-arrow arr-sn-1">↑</div>
            <div class="lane-arrow arr-sn-2">↑</div>
            <div class="lane-arrow turn-arrow arr-sn-t">↗</div>

            <div class="lane-label lbl-we">West→East (2x straight + 1x turn)</div>
            <div class="lane-label lbl-ew">East→West (2x straight + 1x turn)</div>
            <div class="lane-label lbl-ns">North→South (2x straight + 1x turn)</div>
            <div class="lane-label lbl-sn">South→North (2x straight + 1x turn)</div>
        </div>

        <div class="lights-overlay">
            <div class="card">North <span class="light off" id="north"></span></div>
            <div class="card">East <span class="light off" id="east"></span></div>
            <div class="card">South <span class="light off" id="south"></span></div>
            <div class="card">West <span class="light off" id="west"></span></div>
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

        function setLaneSignal(id, value) {
            const el = document.getElementById(id);
            if (!el) return;
            const parts = ['approach-signal'];
            for (const name of el.classList) {
                if (name.startsWith('sig-')) {
                    parts.push(name);
                }
            }
            parts.push(value || 'off');
            el.className = parts.join(' ');
        }

        function laneProgress(v, simTime, laneLen) {
            const stopLinePx = Math.max(40, Math.floor(laneLen * 0.42));
            const queuePx = Math.min(stopLinePx, Math.max(0, Number(v.position || 0) * 3.0));
            if (!v.crossing) {
                return queuePx;
            }

            const ctime = Number(v.crossing_time ?? -1);
            const cdur = Math.max(0.1, Number(v.crossing_duration ?? 2.0));
            const t = ctime >= 0 ? (simTime - ctime) / cdur : 0;
            const progress = Math.max(0, Math.min(1, t));

            if (v.turning) {
                // Turning cars travel into intersection before curving out (smaller intersection = 70px)
                return Math.min(laneLen, stopLinePx + progress * 70);
            }

            // Straight-through cars traverse full remaining lane length
            return Math.min(laneLen, stopLinePx + progress * (laneLen - stopLinePx));
        }

        function crossingProgress(v, simTime) {
            if (!v.crossing) return 0;
            const ctime = Number(v.crossing_time ?? -1);
            const cdur = Math.max(0.1, Number(v.crossing_duration ?? 2.0));
            // Turning cars need full duration to complete visual arc
            const t = ctime >= 0 ? (simTime - ctime) / cdur : 0;
            return Math.max(0, Math.min(1, t));
        }

        function cubicBezier01(t, p1, p2) {
            const u = 1 - t;
            return 3 * u * u * t * p1 + 3 * u * t * t * p2 + t * t * t;
        }

        function turnCurveShift(direction, progress) {
            // Bezier curve for smooth turn animation
            // Progress 0 = at stop line, progress 1 = completed turn
            const arc = cubicBezier01(progress, 0.2, 0.8);
            
            // Calculate pixel offsets to move car into destination lane
            // Intersection is ~100px, need to move ~50px to reach other lane
            const turnRadius = 50 * arc;
            
            // W->S: car enters from west, turns right (clockwise) to go south
            // Needs to move: forward (east/+x) into intersection, then down (+y) into N->S lane
            if (direction === 'west') {
                const forwardProgress = Math.min(1, progress * 2);  // First half: go forward
                const turnProgress = Math.max(0, (progress - 0.3) / 0.7);  // Last 70%: turn
                return { 
                    x: 40 * cubicBezier01(forwardProgress, 0.3, 0.7),
                    y: turnRadius * cubicBezier01(turnProgress, 0.2, 0.9)
                };
            }
            
            // E->N: car enters from east, turns right (clockwise from its perspective) to go north
            // Needs to move: forward (west/-x) into intersection, then up (-y) into S->N lane
            if (direction === 'east') {
                const forwardProgress = Math.min(1, progress * 2);
                const turnProgress = Math.max(0, (progress - 0.3) / 0.7);
                return { 
                    x: -40 * cubicBezier01(forwardProgress, 0.3, 0.7),
                    y: -turnRadius * cubicBezier01(turnProgress, 0.2, 0.9)
                };
            }
            
            // N->W: car enters from north, turns right to go west
            // Needs to move: forward (south/+y) into intersection, then left (-x) into E->W lane
            if (direction === 'north') {
                const forwardProgress = Math.min(1, progress * 2);
                const turnProgress = Math.max(0, (progress - 0.3) / 0.7);
                return { 
                    x: -turnRadius * cubicBezier01(turnProgress, 0.2, 0.9),
                    y: 40 * cubicBezier01(forwardProgress, 0.3, 0.7)
                };
            }
            
            // S->E: car enters from south, turns right to go east
            // Needs to move: forward (north/-y) into intersection, then right (+x) into W->E lane
            const forwardProgress = Math.min(1, progress * 2);
            const turnProgress = Math.max(0, (progress - 0.3) / 0.7);
            return { 
                x: turnRadius * cubicBezier01(turnProgress, 0.2, 0.9),
                y: -40 * cubicBezier01(forwardProgress, 0.3, 0.7)
            };
        }

        function laneOffsetIndex(direction, v) {
            // Turn lane is always on the RIGHT side of movement direction.
            if (v.turning) {
                if (direction === 'west') return 2;  // W->E, right is down
                if (direction === 'east') return 0;  // E->W, right is up
                if (direction === 'north') return 0; // N->S, right is left
                return 2;                            // S->N, right is right
            }

            if (direction === 'west') return Number(v.id) % 2;          // 0/1
            if (direction === 'east') return 1 + (Number(v.id) % 2);    // 1/2
            if (direction === 'north') return 1 + (Number(v.id) % 2);   // 1/2
            return Number(v.id) % 2;                                     // 0/1
        }

        function drawLane(direction, vehicles, simTime) {
            const lane = document.getElementById('lane-' + direction);
            lane.innerHTML = '';

            const horizontal = (direction === 'west' || direction === 'east');
            const laneLen = horizontal ? Math.max(1, lane.clientWidth - 10) : Math.max(1, lane.clientHeight - 10);
            const laneOffsets = [5, 21, 37];

            for (const v of vehicles) {
                const car = document.createElement('div');
                car.className = 'car' + (v.crossing ? ' crossing' : '') + (v.turning ? ' turning' : '');

                const idx = laneOffsetIndex(direction, v);
                const dist = Math.min(laneLen, laneProgress(v, simTime, laneLen));
                const cprog = crossingProgress(v, simTime);

                let axis = dist;
                if (direction === 'east' || direction === 'south') {
                    axis = laneLen - dist;
                }

                let extraX = 0;
                let extraY = 0;
                if (v.turning && v.crossing) {
                    const curve = turnCurveShift(direction, cprog);
                    extraX = curve.x;
                    extraY = curve.y;
                }

                if (horizontal) {
                    car.style.left = `${axis + extraX}px`;
                    car.style.top = `${laneOffsets[idx] + extraY}px`;
                } else {
                    car.style.left = `${laneOffsets[idx] + extraX}px`;
                    car.style.top = `${axis + extraY}px`;
                }

                let baseAngle = 0;
                if (direction === 'west') baseAngle = 0;      // W -> E
                if (direction === 'east') baseAngle = 180;    // E -> W
                if (direction === 'north') baseAngle = 90;    // N -> S
                if (direction === 'south') baseAngle = -90;   // S -> N

                let heading = baseAngle;
                if (v.turning && v.crossing) {
                    heading = baseAngle + (90 * cprog);
                }

                car.style.transformOrigin = 'center center';
                car.style.transform = `rotate(${heading}deg)`;

                const laneType = v.turning ? 'turn' : (idx === 0 ? 'straight-1' : 'straight-2');
                car.title = `id=${v.id}, lane=${laneType}, speed=${Number(v.speed).toFixed(1)}m/s`;
                lane.appendChild(car);
            }
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
            ['0', '1', '2'].forEach(i => {
                setLaneSignal(`sig-north-${i}`, s.lights.north);
                setLaneSignal(`sig-east-${i}`, s.lights.east);
                setLaneSignal(`sig-south-${i}`, s.lights.south);
                setLaneSignal(`sig-west-${i}`, s.lights.west);
            });
            setLight('turnSouthEast', s.lights.turnSouthEast);
            setLight('turnNorthWest', s.lights.turnNorthWest);
            setLight('turnWestSouth', s.lights.turnWestSouth);
            setLight('turnEastNorth', s.lights.turnEastNorth);

            drawLane('west', s.lanes.west || [], s.sim_time);
            drawLane('east', s.lanes.east || [], s.sim_time);
            drawLane('north', s.lanes.north || [], s.sim_time);
            drawLane('south', s.lanes.south || [], s.sim_time);
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
