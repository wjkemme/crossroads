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

        /* Main lanes: 2 straight + 1 turn lane (48px total, 16px each) */
        .lane.west-east {
            left: 0; right: 0; top: 50%; height: 48px;
            background: rgba(148, 163, 184, 0.10);
        }
        .lane.east-west {
            left: 0; right: 0; top: calc(50% - 48px); height: 48px;
            background: rgba(148, 163, 184, 0.10);
        }
        .lane.north-south {
            top: 0; bottom: 0; left: calc(50% - 48px); width: 48px;
            background: rgba(148, 163, 184, 0.10);
        }
        .lane.south-north {
            top: 0; bottom: 0; left: 50%; width: 48px;
            background: rgba(148, 163, 184, 0.10);
        }

        /* Separate uitvoeg/invoeg elements no longer needed - integrated in 48px lane */
        /* Uitvoeg/invoeg CSS no longer needed - integrated into 48px lanes */

        .stop-line { position: absolute; background: #f8fafc; opacity: 0.95; z-index: 5; }
        .stop-line.west-east { left: calc(50% - 72px); top: 50%; width: 3px; height: 48px; }
        .stop-line.east-west { left: calc(50% + 70px); top: calc(50% - 48px); width: 3px; height: 48px; }
        .stop-line.north-south { left: calc(50% - 48px); top: calc(50% - 72px); width: 48px; height: 3px; }
        .stop-line.south-north { left: 50%; top: calc(50% + 70px); width: 48px; height: 3px; }

        /* No turn-cut needed anymore - slip roads are separate */

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
        .arrow-ew { left: calc(50% + 70px); top: calc(50% - 32px); }
        .arrow-ns { left: calc(50% - 32px); top: calc(50% - 150px); }
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
        /* 3 signals per direction (2 straight + 1 turn) */
        .sig-north-0 { left: calc(50% - 48px); top: calc(50% - 68px); }
        .sig-north-1 { left: calc(50% - 32px); top: calc(50% - 68px); }
        .sig-north-2 { left: calc(50% - 16px); top: calc(50% - 68px); }

        .sig-south-0 { left: calc(50% + 4px);  top: calc(50% + 56px); }
        .sig-south-1 { left: calc(50% + 20px); top: calc(50% + 56px); }
        .sig-south-2 { left: calc(50% + 36px); top: calc(50% + 56px); }

        .sig-west-0  { left: calc(50% - 68px); top: calc(50% + 4px); }
        .sig-west-1  { left: calc(50% - 68px); top: calc(50% + 20px); }
        .sig-west-2  { left: calc(50% - 68px); top: calc(50% + 36px); }

        .sig-east-0  { left: calc(50% + 56px); top: calc(50% - 48px); }
        .sig-east-1  { left: calc(50% + 56px); top: calc(50% - 32px); }
        .sig-east-2  { left: calc(50% + 56px); top: calc(50% - 16px); }

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
        /* Two dashed lines per lane direction (between 3 lanes) */
        .lane.west-east::before, .lane.east-west::before { top: 16px; }
        .lane.west-east::after, .lane.east-west::after { top: 32px; }

        .lane.north-south::before, .lane.south-north::before {
            top: 0; bottom: 0; width: 1px; left: 16px;
            background: repeating-linear-gradient(
                to bottom,
                rgba(226, 232, 240, 0.8) 0 5px,
                transparent 5px 10px
            );
        }
        .lane.north-south::after, .lane.south-north::after {
            top: 0; bottom: 0; width: 1px; left: 32px;
            background: repeating-linear-gradient(
                to bottom,
                rgba(226, 232, 240, 0.8) 0 5px,
                transparent 5px 10px
            );
        }

        .lane-label { position: absolute; font-size: 11px; color: #94a3b8; background: rgba(15, 23, 42, 0.65); padding: 2px 6px; border-radius: 6px; }
        .lbl-we { left: 12px; top: calc(50% + 52px); }
        .lbl-ew { right: 12px; top: calc(50% - 68px); }
        .lbl-ns { left: calc(50% - 164px); top: 12px; }
        .lbl-sn { left: calc(50% + 52px); bottom: 12px; }

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
        /* Arrow positions: centered in each 16px lane (3 lanes per direction) */
        /* W->E: lane top=50%, lanes at +8px, +24px, +40px */
        .arr-we-1 { left: calc(50% - 100px); top: calc(50% + 4px); }
        .arr-we-2 { left: calc(50% - 100px); top: calc(50% + 20px); }
        .arr-we-t { left: calc(50% - 100px); top: calc(50% + 36px); }

        /* E->W: lane top=50%-48px, lanes at -40px, -24px, -8px */
        .arr-ew-t { left: calc(50% + 80px); top: calc(50% - 44px); }
        .arr-ew-1 { left: calc(50% + 80px); top: calc(50% - 28px); }
        .arr-ew-2 { left: calc(50% + 80px); top: calc(50% - 12px); }

        /* N->S: lane left=50%-48px, lanes at -40px, -24px, -8px */
        .arr-ns-t { left: calc(50% - 44px); top: calc(50% - 100px); }
        .arr-ns-1 { left: calc(50% - 28px); top: calc(50% - 100px); }
        .arr-ns-2 { left: calc(50% - 12px); top: calc(50% - 100px); }

        /* S->N: lane left=50%, lanes at +8px, +24px, +40px */
        .arr-sn-1 { left: calc(50% + 4px); top: calc(50% + 80px); }
        .arr-sn-2 { left: calc(50% + 20px); top: calc(50% + 80px); }
        .arr-sn-t { left: calc(50% + 36px); top: calc(50% + 80px); }

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

            <div class="stop-line west-east"></div>
            <div class="stop-line east-west"></div>
            <div class="stop-line north-south"></div>
            <div class="stop-line south-north"></div>

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

            <div class="lane-arrow turn-arrow arr-ns-t">↙</div>
            <div class="lane-arrow arr-ns-1">↓</div>
            <div class="lane-arrow arr-ns-2">↓</div>

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

        function stopLinePx(direction, laneLen) {
            // Align pixel stoplijn per richting vóór de kruising (incoming side)
            const mid = laneLen * 0.5;
            const off = 60; // afstand vanaf het midden naar de stoplijn
            switch (direction) {
                case 'west':  return mid - off; // W->E: voor het midden
                case 'east':  return mid - off; // E->W: spiegel, axis wordt later omgedraaid
                case 'north': return mid - off; // N->S: boven het midden
                case 'south': return mid - off; // S->N: spiegel, axis wordt later omgedraaid
                default:      return Math.max(40, Math.floor(laneLen * 0.33));
            }
        }

        function laneProgress(v, simTime, laneLen, direction) {
            const stopPx = stopLinePx(direction, laneLen);
            const metersToPx = stopPx / 70.0; // STOP_LINE_POSITION = 70m
            const queuePx = Math.min(stopPx, Math.max(0, Number(v.position || 0) * metersToPx));
            if (!v.crossing) {
                return queuePx;
            }

            const ctime = Number(v.crossing_time ?? -1);
            const cdur = Math.max(0.1, Number(v.crossing_duration ?? 2.0));
            const t = ctime >= 0 ? (simTime - ctime) / cdur : 0;
            const progress = Math.max(0, Math.min(1, t));

            if (v.turning) {
                // Turning cars travel into intersection before curving out (100px intersection)
                return Math.min(laneLen, stopPx + progress * 70);
            }

            // Straight-through cars traverse full remaining lane length
            return Math.min(laneLen, stopPx + progress * (laneLen - stopPx));
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
            // Force turning vehicles into the dedicated turn lane (logical index 2)
            if (v.turning) return 2;
            if (typeof v.queue_index === 'number') return v.queue_index;
            return Number(v.id) % 2; // straight lanes alternate if missing
        }

        function laneOffset(direction, laneIndex, turning) {
            // Offsets per direction so the turn lane is op de rechterkant van de rijrichting
            // West->East, South->North: right side is bottom/right
            // East->West, North->South: right side is top/left
            if (direction === 'east' || direction === 'north') {
                // Top/left = right-hand side for these directions: turn lane first
                if (turning) return 4;        // turn lane buitenkant
                if (laneIndex === 0) return 20; // straight inner
                return 36;                     // straight outer
            }
            // Default: right-hand side is bottom/right
            if (turning) return 36;            // turn lane buitenkant
            if (laneIndex === 0) return 4;     // straight inner
            return 20;                         // straight outer
        }

        function drawLane(direction, vehicles, simTime) {
            const lane = document.getElementById('lane-' + direction);
            lane.innerHTML = '';

            const horizontal = (direction === 'west' || direction === 'east');
            const laneLen = horizontal ? Math.max(1, lane.clientWidth - 10) : Math.max(1, lane.clientHeight - 10);
            // 3 lanes in 48px (each 16px wide) handled via laneOffset()

            for (const v of vehicles) {
                const car = document.createElement('div');
                car.className = 'car' + (v.crossing ? ' crossing' : '') + (v.turning ? ' turning' : '');

            const idx = laneOffsetIndex(direction, v);
            const dist = Math.min(laneLen, laneProgress(v, simTime, laneLen, direction));
            const cprog = crossingProgress(v, simTime);

            // Default lane offset (perp axis) for current direction/lane
            let laneOff = laneOffset(direction, idx, v.turning);

            let drawHorizontal = horizontal;
            let axis = dist;
            let perp = laneOff;
            let headingBase = 0;
            if (direction === 'west') headingBase = 0;      // W -> E
            if (direction === 'east') headingBase = 180;    // E -> W
            if (direction === 'north') headingBase = 90;    // N -> S
            if (direction === 'south') headingBase = -90;   // S -> N

            if (v.turning && v.crossing) {
                // Hard snap to destination leg, outer straight lane (rightmost of target direction)
                const destDir = (direction === 'west') ? 'south' :
                                (direction === 'south') ? 'east' :
                                (direction === 'east') ? 'north' : 'west';
                drawHorizontal = (destDir === 'west' || destDir === 'east');
                const destLaneLen = drawHorizontal ? Math.max(1, lane.clientWidth - 10) : Math.max(1, lane.clientHeight - 10);
                const destStop = stopLinePx(destDir, destLaneLen);
                perp = laneOffset(destDir, 1, false); // outer straight lane of destination

                // Move lineair vanaf de stoplijn van de doelrichting naar het einde van die rijstrook
                axis = destStop + cprog * (destLaneLen - destStop);
                if (destDir === 'east' || destDir === 'south') {
                    axis = destLaneLen - axis;
                }

                headingBase = 0;
                if (destDir === 'west') headingBase = 0;
                if (destDir === 'east') headingBase = 180;
                if (destDir === 'north') headingBase = 90;
                if (destDir === 'south') headingBase = -90;
            }

            if (direction === 'east' || direction === 'south') {
                axis = laneLen - axis;
            }

            if (drawHorizontal) {
                car.style.left = `${axis}px`;
                car.style.top = `${perp}px`;
            } else {
                car.style.left = `${perp}px`;
                car.style.top = `${axis}px`;
            }

            let heading = headingBase;

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
