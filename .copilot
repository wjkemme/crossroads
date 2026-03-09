# Crossroads Project Planning

## Project Overview
- **Project Name**: Crossroads
- **Language**: C++20
- **Code standard**: clean code (Robert C. Martin)
- **Complexity**: Medium to low complexity, with a focus on modular design and maintainability. The project is structured to allow for incremental development, starting with a basic traffic generator and control algorithm, and evolving towards more sophisticated features such as safety checks, multiple control algorithms, and advanced visualization. The use of C++20 enables modern programming practices while maintaining performance and efficiency. Keep the codebase clean and well-documented to facilitate future enhancements and ensure that new contributors can easily understand the architecture and design decisions. Methods should be concise and focused on a single responsibility, adhering to the principles of clean code to promote readability and maintainability throughout the project lifecycle.
- **Build System**: cmake
- **Database**: ORM: sqlite_orm
- **Purpose**:
    - **Simple Traffic Generator**: This module is responsible for simulating the flow of vehicles approaching and entering a crossroad. It creates virtual traffic patterns, such as cars arriving at random intervals and from different directions, to mimic real-world conditions at an intersection.
    - **Basic Traffic Light Control Algorithm**: The application includes a straightforward logic for managing the traffic lights at the crossroad. This algorithm determines when each light turns green or red, coordinating the movement of vehicles through the intersection to ensure safety. This control algorithm is purely clock driven and does not consider any traffic information.
    - **Results Visualization on a Map**: The outcome of the simulation—such as vehicle movement, stops, and light changes—is displayed on a map within the application interface. This visualization allows users to observe how traffic flows and how the algorithm affects congestion and wait times at the crossroad.

## Basic Project Structure
crossroads/
├── main.cpp # Main application entry point
├── build/ # Compiled binary output directory
│ └── crossroads # Executable
├── .vscode/
│ ├── tasks.json # Build and run task configuration
│ └── launch.json # Debugger configuration
└── .copilot # Project planning file

## Features & Functionality
- 

## Build & Development Commands
- **Build & Run**: `Ctrl+Shift+B` (or `g++ main.cpp -o build/crossroads && ./build/crossroads`)
- **Debug**: `F5` (with gdb)

## Notes
- Dependencies required: g++, gdb
- **Constraints**: To ensure crossroad layouts are reusable and accessible, each configuration must be saved in a database. This approach not only provides flexibility for experimentation and deployment but also establishes a structured way to manage multiple intersection designs within the application.
- **Safety**: Before we start working on smarter control algorithms, we need to ensure that the crossroads are always in a safe state. Therefore, in between the control algorithm and the interface to the traffic lights, a safety checker will be added. The safety checker warrants the consistency of commands to the traffic lights. The safety checker implements a series of rules and rigorously checks that none of the rules are violated at any time.
- **Safety**: Before we start working on smarter control algorithms, we need to ensure that the crossroads are always in a safe state. Therefore, in between the control algorithm and the interface to the traffic lights, a safety checker will be added. The safety checker warrants the consistency of commands to the traffic lights. The safety checker implements a series of rules and rigorously checks that none of the rules are violated at any time.
- This approach ensures that conflicting signals—such as multiple directions showing green at the same time—are systematically prevented. The safety checker acts as a safeguard layer, monitoring all light changes and enforcing critical safety constraints regardless of the logic produced by the control algorithm. By introducing this component, the integrity and reliability of the intersection control system are significantly strengthened, reducing the risk of accidents caused by software errors or unforeseen circumstances.
- **Architectural boundary (mandatory)**: Safety logic validates and constrains **traffic light states/transitions only**. Safety must **never** alter road layout semantics, lane connectivity, or allowed movements. Those are defined exclusively by the intersection configuration and interpreted by routing/simulation modules.
- ORM: sqlite_orm
- gebruik tussen server en client UI communicatie WebSocket/REST
- CMake

## Traffic light states and transitions
- Add `Orange` (amber) light state in addition to `Red` and `Green`.
- Allowed transitions per light: `Green -> Orange -> Red`, and `Red -> Green` only.
- `Red -> Orange` is invalid and must never occur.
- The `Orange` state is always held for a fixed time period: **2 seconds**.
- Minimum green lock: once a light turns green, it remains green for at least **3 seconds** unless safety would be violated.
- **Crossing-light safety rule**: A light cannot transition to `Green` if the other route is still active (has any light in `Green` or `Orange` state).
  - North/South are the same route; East/West are the same route.
  - To switch from NS to EW (or vice versa), all lights of the moving-from route must first reach `Red`.
- The `SafetyChecker` will validate safety invariants (no simultaneous NS and EW greens), transition validity (per-light rules and orange duration), and crossing-light constraints.
- **Red-hold + clearance**: A route (anchor or parallel) may turn Green only if every conflicting route (a) is Red for at least **2 seconds** (post-transition state) and (b) has been free of crossing vehicles for at least **2 seconds**. Orange blocks activation.

## Turning Lights (Afslagindicatoren)
- **4 turning lights** for safe turns at the intersection:
  - `turnSouthEast` (South→East): allowed only when West is Red
  - `turnNorthWest` (North→West): allowed only when East is Red
  - `turnWestSouth` (West→South): allowed only when North is Red
  - `turnEastNorth` (East→North): allowed only when South is Red
- Each turn light follows the same transition rules: `Red -> Green -> Orange -> Red`
- Orange duration: 2 seconds (same as main lights)

## Vehicle Spacing & Speed
**Implementation Details (current):**
- Each vehicle tracks `current_speed` (0-10 m/s) and `position_in_lane` (meters).
- Accel/decel capped at **3.0 m/s²**.
- Stop target: **66 m** (4 m before the 70 m stop line); front cars clamp there on red.
- Stopped gap: **2 m** bumper-to-bumper; minimum front-to-front spacing **6 m** (4 m car + 2 m gap).
- Moving gap: **1.5 s** time headway + car length; gaps enforced against the nearest non-crossing vehicle (crossing cars no longer block the platoon).
- Crossing duration scales with density: `2.5 + (density × 2.0)` seconds; turning multiplies by **1.6**.
- `updateVehicleSpeeds()` runs per tick and maintains gaps while allowing all waiting cars to proceed on green.

## SimulatorEngine Architecture

**Purpose:** Orchestrates all components into a unified simulation.

**Components:**
- `SafetyChecker`: Validates all state transitions
- `BasicLightController`: Drives 4-phase light cycling
- `TrafficGenerator`: Manages vehicle queues + speed updates
- `SimulatorMetrics`: Tracks performance (vehicles crossed, avg wait, violations)

**Simulation Loop (per tick):**
1. Generate traffic (Poisson-like arrivals)
2. Update vehicle speeds (0.5 m/s² acceleration/deceleration)
3. Start crossing vehicles (if light green)
4. Complete crossing vehicles (based on density-scaled duration)
5. Advance light controller
6. Validate state with SafetyChecker (count violations)

## Scheduler Contract (Route-based)

**Goal:** groen als het kan, rood als het moet; maximale doorstroming zonder starvation.

### 1) Route conflict matrix is source of truth
- Build a fixed conflict matrix per route (`from_lane + movement`).
- Conflict matrix determines which routes must be red before a target route can become green.
- Policy: when one lane in a route-family conflicts, the full conflicting route-family is blocked.

### 2) Anchor route + optional parallel activation
- Scheduler selects one anchor route by priority (queue pressure + wait aging).
- Conflicting routes are moved to red first.
- Non-conflicting routes are **left untouched** (not forced green, not forced red).
- Optional optimization: if additional routes can safely go green in parallel with the anchor, activate them in the same cycle.

### 3) Fairness and starvation guarantees
- Per-route adaptive priority with bounded-wait guarantee.
- Hard anti-starvation bound: `Wmax = 20s` under sustained demand.
- Scheduler balances throughput and fairness; no route with sustained demand may remain permanently red.

### 4) Transition and phase discipline
- Transition order per light remains: `Green -> Orange -> Red -> Green`.
- Never perform `Red -> Orange`.
- Replanning occurs at controlled decision moments to avoid oscillation/flapping.

### 5) SafetyChecker role
- Scheduler performs primary route selection and conflict handling.
- `SafetyChecker` remains the **final guardrail** for state/transition validity.
- SafetyChecker does not define route priority and does not mutate layout/routing semantics.

### 6) Runtime observability
- Expose conflict matrix and scheduler decisions in snapshot/debug data.
- UI must show a dedicated debug section with:
  - route -> conflicting routes,
  - active anchor route,
  - parallel activated routes,
  - blocked-by-safety reasons,
  - current wait/priority indicators.

**Key Methods:**
- `simulate(duration, dt)`: Run full simulation
- `tick(dt)`: Single time step
- `getMetrics()`: Query performance data

## Iteration: Different Control Algorithms
To enable smarter traffic management, the system needs to support multiple control algorithms and provide a mechanism for switching between them seamlessly. This begins with designing the application architecture so that different algorithms—such as the basic clock-driven control, the “null-control” (amber flashing mode), and more advanced logic—can coexist and be selected as needed. The ability to switch algorithms dynamically is crucial for both operational flexibility and safety.

Implementing a “null-control” mode, which causes all lights to flash amber, serves as a vital fallback. This mode should be triggered automatically if the safety checker detects any rule violation or inconsistency, ensuring the intersection never enters an unsafe or ambiguous state. When switching between algorithms, it is essential to introduce transitional logic that places all lights in a safe, well-defined state (such as all red or all flashing amber) before activating the next algorithm. This transitional period prevents conflicting signals and avoids illegal states, such as multiple greens in different directions.

Once robust algorithm management and safe switching are established, the system can evolve to include smarter algorithms that utilize input from traffic counters placed upstream in the lanes. These counters provide real-time data on approaching vehicles, enabling the control logic to optimize green light durations and reduce congestion proactively. By layering these enhancements, the intersection control system becomes both safer and more responsive to actual traffic conditions.

## Iteration: Key Performance Indicators
A simulation scenario includes three main components:
- The intersection itself, featuring its traffic lights and sensors
- The algorithm that manages control
- Configuration options for the traffic generators

## Iteration: Recording and Playback
The ability to record and replay scenarios is needed. This functionality allows for the preservation of specific simulation runs, facilitating thorough analysis and comparison of results. Playback capabilities ensure that scenarios can be revisited, helping refine algorithms and optimize intersection performance based on recorded data.

## Iteration: ORM (with a relational database)
The application becomes more and more dependent on its database. Traditional methods, accessing the database directly using query language statements such as SQL, ignore the fact that there is major commonality between the data structure in the database and the object model in the application. Enter Object Relation Mapping (ORM).
An Object-Relational Mapping (ORM) is a programming technique that creates a bridge between object-oriented programming languages and relational databases. An ORM is a layer of abstraction that converts data between incompatible type systems - specifically between your application's objects and your database's tables, rows, and columns.
Key Benefits:
- Abstraction of SQL: Write database operations using your programming language instead of SQL
Object-Oriented Interface: Interact with your database using familiar object-oriented patterns
- Reduced Boilerplate: Eliminate repetitive CRUD (Create, Read, Update, Delete) code
- Database Agnosticism: Switch database systems with minimal code changes
- Data Validation: Built-in validation before persisting data
ORMs provide a convenient way to work with databases while maintaining the object-oriented paradigm in your application code.
In this iteration, the app is to be refactored to use an ORM instead of direct access to the database.

## Iteration: Crossroads viewer and editor
To support applying the program to various road crossings, the application should incorporate a crossroads editor that enables users to design, modify, and select different intersection layouts. This editor will allow for the creation and customization of road crossing maps, which can be tailored to reflect real-world or hypothetical scenarios.

## Iteration: Decoupling of UI
Up till now, the UI has been an integral part of the application. Moving forward, we want to split the UI from the simulator and achieve the following:
- We want to close and restart the UI while the simulation keeps running. Ideally, we want to alter the UI without losing the state of the simulator.
- We want to instantiate multiple UIs, connected to the same simulation.
- We want to run the UI on a different computer than the simulation.
As usual, take care of security when connecting and running applications via a network.

## Iteration: Live Maps
To enhance the user experience, it is beneficial to load actual map data into the UI. Maps can be used from OpenSteetMaps or Google Maps.
This iteration is also suited to review and refine the traffic generators to make scenarios more realistic. E.g. trucks have different characteristics than passenger cars and thus behave differently in traffic.