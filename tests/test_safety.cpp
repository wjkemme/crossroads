#define CATCH_CONFIG_MAIN
#include <catch2/catch_all.hpp>
#include <memory>
#include "SafetyChecker.hpp"
#include "BasicLightController.hpp"
#include "TrafficGenerator.hpp"
#include "SimulatorEngine.hpp"
#include "TrafficLightControllers.hpp"
#include "IntersectionConfigJson.hpp"
#include <nlohmann/json.hpp>

using namespace crossroads;

TEST_CASE("SafetyChecker rejects conflicting greens", "safety")
{
    SafetyChecker c;
    IntersectionState s;
    s.north = LightState::Green;
    s.south = LightState::Red;
    s.east = LightState::Green; // conflict with north
    s.west = LightState::Red;
    REQUIRE(c.isSafe(s) == false);
}

TEST_CASE("SafetyChecker accepts single direction green", "safety")
{
    SafetyChecker c;
    IntersectionState s;
    s.north = LightState::Green;
    s.east = LightState::Red;
    s.south = LightState::Red;
    s.west = LightState::Red;
    REQUIRE(c.isSafe(s) == true);
}

TEST_CASE("Transitions enforce G->O->R and R->G with orange timing", "transition")
{
    SafetyChecker c;
    IntersectionState prev;
    prev.north = LightState::Green;
    prev.east = LightState::Red;
    prev.south = LightState::Red;
    prev.west = LightState::Red;

    IntersectionState toOrange = prev;
    toOrange.north = LightState::Orange;
    REQUIRE(c.isValidTransition(prev, toOrange, 0.0) == true);

    IntersectionState toRed = toOrange;
    toRed.north = LightState::Red;
    // orange -> red requires at least ORANGE_DURATION seconds
    REQUIRE(c.isValidTransition(toOrange, toRed, 1.0) == false);
    REQUIRE(c.isValidTransition(toOrange, toRed, SafetyChecker::ORANGE_DURATION) == true);

    // direct green->red is invalid
    IntersectionState direct = prev;
    direct.north = LightState::Red;
    REQUIRE(c.isValidTransition(prev, direct, 0.1) == false);

    // red->green is valid
    IntersectionState red;
    red.north = LightState::Red;
    red.east = LightState::Red;
    red.south = LightState::Red;
    red.west = LightState::Red;
    IntersectionState green = red;
    green.north = LightState::Green;
    REQUIRE(c.isValidTransition(red, green, 0.1) == true);
}
TEST_CASE("Cannot go green if other route is green or orange", "crossing")
{
    SafetyChecker c;

    // Test North-South route: north cannot go green if east OR west is still active
    IntersectionState prev;
    prev.north = LightState::Red;
    prev.south = LightState::Red;
    prev.east = LightState::Green; // EW route is active
    prev.west = LightState::Red;

    IntersectionState attemptGreen = prev;
    attemptGreen.north = LightState::Green;
    REQUIRE(c.isValidTransition(prev, attemptGreen, 0.1) == false);

    // Test North-South route: north cannot go green if west is orange
    prev.east = LightState::Red;
    prev.west = LightState::Orange; // EW route is active

    attemptGreen = prev;
    attemptGreen.north = LightState::Green;
    REQUIRE(c.isValidTransition(prev, attemptGreen, 0.1) == false);

    // Test North-South route: north CAN go green if both east and west are red
    prev.east = LightState::Red;
    prev.west = LightState::Red;

    attemptGreen = prev;
    attemptGreen.north = LightState::Green;
    REQUIRE(c.isValidTransition(prev, attemptGreen, 0.1) == true);

    // Test East-West route: east cannot go green if north OR south is still active
    prev.north = LightState::Green; // NS route is active
    prev.south = LightState::Red;
    prev.east = LightState::Red;
    prev.west = LightState::Red;

    attemptGreen = prev;
    attemptGreen.east = LightState::Green;
    REQUIRE(c.isValidTransition(prev, attemptGreen, 0.1) == false);

    // Test North and South can BOTH be green (same route)
    prev.north = LightState::Red;
    prev.south = LightState::Red;
    prev.east = LightState::Red;
    prev.west = LightState::Red;

    IntersectionState bothNSGreen = prev;
    bothNSGreen.north = LightState::Green;
    bothNSGreen.south = LightState::Green;
    REQUIRE(c.isSafe(bothNSGreen) == true);
}

TEST_CASE("Turning lights enforce cross-traffic constraints", "turning_lights")
{
    SafetyChecker c;

    // With lane-aware conflicts, S->E can go green if active W->E movement is lane-disjoint.
    IntersectionState prev;
    prev.north = LightState::Red;
    prev.south = LightState::Red;
    prev.east = LightState::Red;
    prev.west = LightState::Green; // West is active
    prev.turnSouthEast = LightState::Red;

    IntersectionState attemptTurnGreen = prev;
    attemptTurnGreen.turnSouthEast = LightState::Green;
    REQUIRE(c.isValidTransition(prev, attemptTurnGreen, 0.1) == true);

    // Test turnSouthEast CAN go green if West is red
    prev = IntersectionState{};
    prev.west = LightState::Red;
    attemptTurnGreen = prev;
    attemptTurnGreen.turnSouthEast = LightState::Green;
    REQUIRE(c.isValidTransition(prev, attemptTurnGreen, 0.1) == true);

    // Lane-disjoint right turns remain valid while opposing main is active.
    prev = IntersectionState{};
    prev.north = LightState::Red;
    prev.east = LightState::Orange; // East is active
    prev.turnNorthWest = LightState::Red;

    attemptTurnGreen = prev;
    attemptTurnGreen.turnNorthWest = LightState::Green;
    REQUIRE(c.isValidTransition(prev, attemptTurnGreen, 0.1) == true);

    // Lane-disjoint right turns remain valid while opposing main is active.
    prev = IntersectionState{};
    prev.north = LightState::Green; // North is active
    prev.east = LightState::Red;
    prev.west = LightState::Red;
    prev.turnWestSouth = LightState::Red;

    attemptTurnGreen = prev;
    attemptTurnGreen.turnWestSouth = LightState::Green;
    REQUIRE(c.isValidTransition(prev, attemptTurnGreen, 0.1) == true);

    // Lane-disjoint right turns remain valid while opposing main is active.
    prev = IntersectionState{};
    prev.north = LightState::Red;
    prev.south = LightState::Orange; // South is active
    prev.east = LightState::Red;
    prev.turnEastNorth = LightState::Red;

    attemptTurnGreen = prev;
    attemptTurnGreen.turnEastNorth = LightState::Green;
    REQUIRE(c.isValidTransition(prev, attemptTurnGreen, 0.1) == true);

    // Test turnEastNorth CAN go green if South is red
    prev.south = LightState::Red;
    attemptTurnGreen = prev;
    attemptTurnGreen.turnEastNorth = LightState::Green;
    REQUIRE(c.isValidTransition(prev, attemptTurnGreen, 0.1) == true);
}

TEST_CASE("BasicLightController initializes NS green", "controller")
{
    BasicLightController ctrl(5.0, 5.0);
    IntersectionState state = ctrl.getCurrentState();

    REQUIRE(state.north == LightState::Green);
    REQUIRE(state.south == LightState::Green);
    REQUIRE(state.east == LightState::Red);
    REQUIRE(state.west == LightState::Red);
}

TEST_CASE("BasicLightController cycles through phases", "controller")
{
    BasicLightController ctrl(1.0, 1.0); // 1 second green phases
    SafetyChecker checker;

    // Initial state: NS green
    auto state = ctrl.getCurrentState();
    REQUIRE(state.north == LightState::Green);
    REQUIRE(state.south == LightState::Green);

    // After 1+ second: should transition to NS orange
    ctrl.tick(1.1);
    state = ctrl.getCurrentState();
    REQUIRE(state.north == LightState::Orange);
    REQUIRE(state.south == LightState::Orange);
    REQUIRE(state.east == LightState::Red);
    REQUIRE(state.west == LightState::Red);

    // After 2 more seconds: should be EW green (orange took 2 seconds)
    ctrl.tick(2.1);
    state = ctrl.getCurrentState();
    REQUIRE(state.north == LightState::Red);
    REQUIRE(state.south == LightState::Red);
    REQUIRE(state.east == LightState::Green);
    REQUIRE(state.west == LightState::Green);

    // After 1 more second: should be EW orange
    ctrl.tick(1.1);
    state = ctrl.getCurrentState();
    REQUIRE(state.east == LightState::Orange);
    REQUIRE(state.west == LightState::Orange);

    // All transitions should be safe
    REQUIRE(checker.isSafe(state) == true);
}

TEST_CASE("BasicLightController reset works", "controller")
{
    BasicLightController ctrl(3.0, 3.0);

    // Advance time
    ctrl.tick(5.1);
    auto state = ctrl.getCurrentState();

    // Reset and verify initial state
    ctrl.reset();
    state = ctrl.getCurrentState();

    // Should be back to NS green
    REQUIRE(state.north == LightState::Green);
    REQUIRE(state.south == LightState::Green);
    REQUIRE(state.east == LightState::Red);
    REQUIRE(state.west == LightState::Red);
}

TEST_CASE("BasicLightController ends green early when active corridor has no demand", "[controller][demand]")
{
    BasicLightController ctrl(10.0, 10.0);

    ctrl.setDemandByDirection({false, false, true, true});
    ctrl.tick(1.1);

    auto state = ctrl.getCurrentState();
    REQUIRE(state.north == LightState::Orange);
    REQUIRE(state.south == LightState::Orange);

    ctrl.tick(2.1);
    state = ctrl.getCurrentState();
    REQUIRE(state.east == LightState::Green);
    REQUIRE(state.west == LightState::Green);
}

TEST_CASE("BasicLightController enforces max-red fairness under competing demand", "[controller][demand]")
{
    BasicLightController ctrl(100.0, 100.0);

    ctrl.setDemandByDirection({true, true, true, true});
    ctrl.tick(8.1);

    auto state = ctrl.getCurrentState();
    REQUIRE(state.north == LightState::Orange);
    REQUIRE(state.south == LightState::Orange);

    ctrl.tick(2.1);
    state = ctrl.getCurrentState();
    REQUIRE(state.east == LightState::Green);
    REQUIRE(state.west == LightState::Green);
}

TEST_CASE("BasicLightController keeps corridor green while only that corridor has demand", "[controller][demand]")
{
    BasicLightController ctrl(3.0, 3.0);
    ctrl.setDemandByDirection({true, true, false, false});

    ctrl.tick(9.0);
    auto state = ctrl.getCurrentState();
    REQUIRE(state.north == LightState::Green);
    REQUIRE(state.south == LightState::Green);
    REQUIRE(state.east == LightState::Red);
    REQUIRE(state.west == LightState::Red);
}

TEST_CASE("All four turning lights work correctly", "turning_lights_all")
{
    SafetyChecker c;

    // Test 1: Zuid->Oost (South->East) - allowed when West is Red
    IntersectionState state1;
    state1.north = LightState::Red;
    state1.south = LightState::Red;
    state1.east = LightState::Red;
    state1.west = LightState::Red;
    state1.turnSouthEast = LightState::Green;
    state1.turnNorthWest = LightState::Red;
    state1.turnWestSouth = LightState::Red;
    state1.turnEastNorth = LightState::Red;
    REQUIRE(c.isSafe(state1) == true);

    // Test 2: Oost->Noord (East->North) - allowed when South is Red
    IntersectionState state2;
    state2.north = LightState::Red;
    state2.south = LightState::Red;
    state2.east = LightState::Red;
    state2.west = LightState::Red;
    state2.turnSouthEast = LightState::Red;
    state2.turnNorthWest = LightState::Red;
    state2.turnWestSouth = LightState::Red;
    state2.turnEastNorth = LightState::Green;
    REQUIRE(c.isSafe(state2) == true);

    // Test 3: Noord->West (North->West) - allowed when East is Red
    IntersectionState state3;
    state3.north = LightState::Red;
    state3.south = LightState::Red;
    state3.east = LightState::Red;
    state3.west = LightState::Red;
    state3.turnSouthEast = LightState::Red;
    state3.turnNorthWest = LightState::Green;
    state3.turnWestSouth = LightState::Red;
    state3.turnEastNorth = LightState::Red;
    REQUIRE(c.isSafe(state3) == true);

    // Test 4: West->Zuid (West->South) - allowed when North is Red
    IntersectionState state4;
    state4.north = LightState::Red;
    state4.south = LightState::Red;
    state4.east = LightState::Red;
    state4.west = LightState::Red;
    state4.turnSouthEast = LightState::Red;
    state4.turnNorthWest = LightState::Red;
    state4.turnWestSouth = LightState::Green;
    state4.turnEastNorth = LightState::Red;
    REQUIRE(c.isSafe(state4) == true);
}

TEST_CASE("TrafficGenerator creates vehicles", "traffic")
{
    TrafficGenerator gen(0.5); // 0.5 vehicles per second per lane

    // Generate traffic for 2 seconds
    gen.generateTraffic(2.0, 0.0);

    // Should have some vehicles in queues now
    REQUIRE(gen.getTotalWaiting() > 0);
    REQUIRE(gen.getTotalGenerated() > 0);
}

TEST_CASE("TrafficGenerator queue management", "traffic")
{
    TrafficGenerator gen(2.0); // 2 vehicles per second per lane

    // Generate traffic for 0.6 seconds (enough for 1+ vehicle per lane)
    gen.generateTraffic(0.6, 0.0);
    size_t initial_count = gen.getTotalWaiting();
    REQUIRE(initial_count > 0);

    // Check that we have vehicles in different lanes
    REQUIRE(gen.getQueueLength(Direction::North) > 0);
    REQUIRE(gen.getQueueLength(Direction::South) > 0);
    REQUIRE(gen.getQueueLength(Direction::East) > 0);
    REQUIRE(gen.getQueueLength(Direction::West) > 0);
}

TEST_CASE("TrafficGenerator vehicle crossing simulation", "traffic")
{
    TrafficGenerator gen(2.0); // 2 vehicles per second per lane

    // Generate traffic for 0.6 seconds
    gen.generateTraffic(0.6, 0.0);
    size_t waiting = gen.getTotalWaiting();
    REQUIRE(waiting > 0);

    // Start crossing with first vehicle from North lane
    Vehicle *v = gen.peekNextVehicle(Direction::North);
    REQUIRE(v != nullptr);
    uint32_t vid = v->id;

    bool started = gen.startCrossing(Direction::North, vid, 0.6);
    REQUIRE(started == true);

    // Complete crossing after 2 seconds (typical crossing time)
    bool completed = gen.completeCrossing(vid, 2.6);
    REQUIRE(completed == true);

    // Should have one less vehicle waiting
    REQUIRE(gen.getTotalWaiting() == waiting - 1);

    // Should have one vehicle that has crossed
    REQUIRE(gen.getTotalCrossed() == 1);
}

TEST_CASE("TrafficGenerator reset", "traffic")
{
    TrafficGenerator gen(2.0); // 2 vehicles per second per lane

    // Generate traffic
    gen.generateTraffic(0.6, 0.0);
    REQUIRE(gen.getTotalWaiting() > 0);

    // Reset
    gen.reset();

    // Should be empty
    REQUIRE(gen.getTotalWaiting() == 0);
    REQUIRE(gen.getTotalGenerated() == 0);
    REQUIRE(gen.getTotalCrossed() == 0);
}

TEST_CASE("Vehicle speed updates gradually", "[vehicle]")
{
    Vehicle v(1, Direction::North, 0.0);
    v.updateSpeed(10.0, 0.5);
    REQUIRE(v.current_speed == Catch::Approx(1.5));
    v.updateSpeed(10.0, 0.5);
    REQUIRE(v.current_speed == Catch::Approx(3.0));
}

TEST_CASE("Vehicle crossing duration scales with density", "[vehicle]")
{
    Vehicle v(1, Direction::North, 0.0);
    REQUIRE(v.getCrossingDuration(0) == Catch::Approx(2.2));
    REQUIRE(v.getCrossingDuration(5) == Catch::Approx(3.0));
    REQUIRE(v.getCrossingDuration(10) == Catch::Approx(3.8));
}

TEST_CASE("SimulatorEngine initializes safely", "[engine]")
{
    SimulatorEngine engine(0.5, 10.0, 10.0);
    auto state = engine.getCurrentLightState();
    REQUIRE(engine.getMetrics().safety_violations == 0);
}

TEST_CASE("SimulatorEngine generates and processes vehicles", "[engine]")
{
    SimulatorEngine engine(1.0, 10.0, 10.0);
    engine.tick(0.1);
    auto metrics = engine.getMetrics();
    REQUIRE(metrics.vehicles_generated >= 0);
}

TEST_CASE("SimulatorEngine tracks metrics correctly", "[engine]")
{
    SimulatorEngine engine(0.5, 10.0, 10.0);
    engine.simulate(5.0, 0.1);
    auto metrics = engine.getMetrics();
    REQUIRE(metrics.total_time == Catch::Approx(5.0).margin(0.2));
    REQUIRE(metrics.vehicles_generated >= 0);
}

TEST_CASE("NullControlController flashes amber", "[controller][null]")
{
    NullControlController null_ctrl;

    auto state = null_ctrl.getCurrentState();
    REQUIRE(state.north == LightState::Orange);
    REQUIRE(state.east == LightState::Orange);

    null_ctrl.tick(1.0);
    state = null_ctrl.getCurrentState();
    REQUIRE(state.north == LightState::Red);
    REQUIRE(state.east == LightState::Red);
}

TEST_CASE("SimulatorEngine can switch control modes", "[engine][controller]")
{
    SimulatorEngine engine(0.5, 10.0, 10.0);
    REQUIRE(engine.getControlMode() == SimulatorEngine::ControlMode::Basic);

    engine.setControlMode(SimulatorEngine::ControlMode::NullControl);
    REQUIRE(engine.getControlMode() == SimulatorEngine::ControlMode::NullControl);

    auto state = engine.getCurrentLightState();
    REQUIRE(state.north == LightState::Orange);
    REQUIRE(state.west == LightState::Orange);
}

TEST_CASE("SimulatorEngine falls back to null-control when unsafe", "[engine][fallback]")
{
    class UnsafeController : public ITrafficLightController
    {
    public:
        void tick(double) override {}

        IntersectionState getCurrentState() const override
        {
            IntersectionState s{};
            s.north = LightState::Green;
            s.east = LightState::Green;
            s.south = LightState::Red;
            s.west = LightState::Red;
            return s;
        }

        void reset() override {}
    };

    SimulatorEngine engine(0.5, 10.0, 10.0);
    engine.setController(std::make_unique<UnsafeController>(), SimulatorEngine::ControlMode::Basic);

    engine.start();
    engine.tick(0.1);
    REQUIRE(engine.getControlMode() == SimulatorEngine::ControlMode::NullControl);
    REQUIRE(engine.getMetrics().safety_violations >= 1);
}

TEST_CASE("SimulatorEngine UI commands control run state", "[engine][ui]")
{
    SimulatorEngine engine(0.5, 10.0, 10.0);

    REQUIRE(engine.isRunning() == false);
    engine.handleCommand(SimulatorEngine::UICommand::Start);
    REQUIRE(engine.isRunning() == true);

    engine.tick(0.2);
    auto started_metrics = engine.getMetrics();
    REQUIRE(started_metrics.total_time > 0.0);

    engine.handleCommand(SimulatorEngine::UICommand::Stop);
    REQUIRE(engine.isRunning() == false);

    double frozen_time = engine.getMetrics().total_time;
    engine.tick(0.5);
    REQUIRE(engine.getMetrics().total_time == Catch::Approx(frozen_time));

    engine.handleCommand(SimulatorEngine::UICommand::Step, 0.1);
    REQUIRE(engine.getMetrics().total_time == Catch::Approx(frozen_time + 0.1));

    engine.handleCommand(SimulatorEngine::UICommand::Reset);
    REQUIRE(engine.getMetrics().total_time == Catch::Approx(0.0));
    REQUIRE(engine.isRunning() == false);
}

TEST_CASE("SimulatorEngine snapshot JSON includes UI fields", "[engine][ui]")
{
    SimulatorEngine engine(0.8, 10.0, 10.0);
    engine.start();
    engine.tick(0.1);

    std::string json = engine.getSnapshotJson();

    REQUIRE(json.find("\"sim_time\"") != std::string::npos);
    REQUIRE(json.find("\"running\":true") != std::string::npos);
    REQUIRE(json.find("\"metrics\"") != std::string::npos);
    REQUIRE(json.find("\"queues\"") != std::string::npos);
    REQUIRE(json.find("\"lights\"") != std::string::npos);
    REQUIRE(json.find("\"lanes\"") != std::string::npos);
    REQUIRE(json.find("\"north\"") != std::string::npos);
}

TEST_CASE("SimulatorEngine snapshot lights go all-red when no demand", "[engine][demand]")
{
    SimulatorEngine engine(0.0, 10.0, 10.0);
    engine.start();
    engine.tick(0.2);

    auto snapshot = engine.getSnapshot();
    REQUIRE(snapshot.metrics.total_queue_length == 0);
    REQUIRE(snapshot.lights.north == LightState::Red);
    REQUIRE(snapshot.lights.south == LightState::Red);
    REQUIRE(snapshot.lights.east == LightState::Red);
    REQUIRE(snapshot.lights.west == LightState::Red);
    REQUIRE(snapshot.lights.turnNorthWest == LightState::Red);
    REQUIRE(snapshot.lights.turnSouthEast == LightState::Red);
    REQUIRE(snapshot.lights.turnEastNorth == LightState::Red);
    REQUIRE(snapshot.lights.turnWestSouth == LightState::Red);
}

TEST_CASE("Dedicated right-turn lane actuates turn signal on demand", "[engine][demand][right-turn]")
{
    class AllRedController : public ITrafficLightController
    {
    public:
        void tick(double) override {}
        IntersectionState getCurrentState() const override { return IntersectionState{}; }
        void reset() override {}
    };

    IntersectionConfig config = makeDefaultIntersectionConfig();
    config.approaches[0].lanes = {
        {900, "N-right-only", {MovementType::Right}, true, true, true}};

    SimulatorEngine engine(config, 3.0, 10.0, 10.0);
    engine.setController(std::make_unique<AllRedController>(), SimulatorEngine::ControlMode::Basic);
    engine.setSpawnLaneFilter(TrafficGenerator::SpawnLaneFilter{ApproachId::North, 0});

    engine.start();
    bool saw_actuation = false;
    for (int i = 0; i < 30; ++i)
    {
        engine.tick(0.1);
        auto snapshot = engine.getSnapshot();
        if (snapshot.metrics.queue_lengths[0] > 0)
        {
            REQUIRE(snapshot.lights.north == LightState::Red);
            if (snapshot.lights.turnNorthWest == LightState::Green)
            {
                saw_actuation = true;
                break;
            }
        }
    }

    REQUIRE(saw_actuation);
}

TEST_CASE("Dedicated right-turn keeps minimum green after demand drops", "[engine][demand][right-turn]")
{
    class AllRedController : public ITrafficLightController
    {
    public:
        void tick(double) override {}
        IntersectionState getCurrentState() const override { return IntersectionState{}; }
        void reset() override {}
    };

    IntersectionConfig config = makeDefaultIntersectionConfig();
    config.approaches[0].lanes = {
        {910, "N-right-only", {MovementType::Right}, true, true, true}};
    config.approaches[2].lanes = {
        {920, "S-straight", {MovementType::Straight}, true, true, true}};

    SimulatorEngine engine(config, 3.0, 10.0, 10.0);
    engine.setController(std::make_unique<AllRedController>(), SimulatorEngine::ControlMode::Basic);

    engine.setSpawnLaneFilter(TrafficGenerator::SpawnLaneFilter{ApproachId::North, 0});
    engine.start();

    bool primed_right_turn = false;
    for (int i = 0; i < 40; ++i)
    {
        engine.tick(0.1);
        auto snapshot = engine.getSnapshot();
        if (snapshot.metrics.queue_lengths[0] > 0 && snapshot.lights.turnNorthWest == LightState::Green)
        {
            primed_right_turn = true;
            break;
        }
    }
    REQUIRE(primed_right_turn);

    engine.setSpawnLaneFilter(TrafficGenerator::SpawnLaneFilter{ApproachId::South, 0});
    bool north_drained_with_other_demand = false;
    double hold_start_time = 0.0;

    for (int i = 0; i < 200; ++i)
    {
        engine.tick(0.1);
        auto snapshot = engine.getSnapshot();
        const bool north_empty = snapshot.metrics.queue_lengths[0] == 0;
        const bool some_other_demand = snapshot.metrics.total_queue_length > 0;
        if (north_empty && some_other_demand)
        {
            north_drained_with_other_demand = true;
            hold_start_time = snapshot.sim_time;
            REQUIRE(snapshot.lights.turnNorthWest == LightState::Green);
            break;
        }
    }

    REQUIRE(north_drained_with_other_demand);

    while (engine.getSnapshot().sim_time < hold_start_time + 1.0)
    {
        engine.tick(0.1);
    }
    REQUIRE(engine.getSnapshot().lights.turnNorthWest == LightState::Green);

    while (engine.getSnapshot().sim_time < hold_start_time + 3.3)
    {
        engine.tick(0.1);
    }
    REQUIRE(engine.getSnapshot().lights.turnNorthWest != LightState::Green);

    while (engine.getSnapshot().sim_time < hold_start_time + 5.6)
    {
        engine.tick(0.1);
    }
    REQUIRE(engine.getSnapshot().lights.turnNorthWest == LightState::Red);
}

TEST_CASE("Mixed straight-left lane keeps dedicated left signal red under opposing conflict", "[engine][regression][left-turn]")
{
    class FixedConflictController : public ITrafficLightController
    {
    public:
        void tick(double) override {}
        IntersectionState getCurrentState() const override
        {
            IntersectionState s{};
            s.north = LightState::Green;
            s.south = LightState::Green;
            s.east = LightState::Red;
            s.west = LightState::Red;
            return s;
        }
        void reset() override {}
    };

    IntersectionConfig config = makeDefaultIntersectionConfig();
    config.approaches[0].lanes = {
        {930, "N-mixed-0", {MovementType::Straight, MovementType::Left}, true, true, true}};
    config.lane_connections.clear();
    config.lane_connections.push_back({ApproachId::North, 0, MovementType::Straight, ApproachId::South, 0});
    config.lane_connections.push_back({ApproachId::North, 0, MovementType::Left, ApproachId::East, 0});

    SimulatorEngine engine(config, 4.0, 10.0, 10.0);
    engine.setController(std::make_unique<FixedConflictController>(), SimulatorEngine::ControlMode::Basic);
    engine.setSpawnLaneFilter(TrafficGenerator::SpawnLaneFilter{ApproachId::North, 0});
    engine.start();

    auto hasLeftVehicleInNorth = [](const std::string &snapshot_json)
    {
        nlohmann::json doc = nlohmann::json::parse(snapshot_json);
        for (const auto &vehicle : doc["lanes"]["north"])
        {
            if (vehicle.value("movement", std::string{}) == "left")
            {
                return true;
            }
        }
        return false;
    };

    auto hasLeftVehicleCrossingInNorth = [](const std::string &snapshot_json)
    {
        nlohmann::json doc = nlohmann::json::parse(snapshot_json);
        for (const auto &vehicle : doc["lanes"]["north"])
        {
            if (vehicle.value("movement", std::string{}) == "left" && vehicle.value("crossing", false))
            {
                return true;
            }
        }
        return false;
    };

    bool saw_left_vehicle = false;
    bool saw_left_signal_red = false;

    for (int i = 0; i < 300; ++i)
    {
        engine.tick(0.1);
        const std::string snapshot_json = engine.getSnapshotJson();
        if (hasLeftVehicleInNorth(snapshot_json))
        {
            saw_left_vehicle = true;
        }
        nlohmann::json doc = nlohmann::json::parse(snapshot_json);
        const std::string left_signal = doc["lights"].value("turnNorthEast", std::string{"red"});
        if (hasLeftVehicleInNorth(snapshot_json) && left_signal == "red")
        {
            saw_left_signal_red = true;
            break;
        }
    }

    REQUIRE(saw_left_vehicle);
    REQUIRE(saw_left_signal_red);
}

TEST_CASE("Left-only lane without traffic light never enters crossing for free or existing destination lanes", "[engine][regression][left-turn]")
{
    class FixedConflictController : public ITrafficLightController
    {
    public:
        void tick(double) override {}
        IntersectionState getCurrentState() const override
        {
            IntersectionState s{};
            s.north = LightState::Green;
            s.south = LightState::Green;
            s.east = LightState::Red;
            s.west = LightState::Red;
            return s;
        }
        void reset() override {}
    };

    SECTION("free destination lane")
    {
        IntersectionConfig config = makeDefaultIntersectionConfig();
        config.approaches[0].lanes = {
            {940, "N-left-only-no-light", {MovementType::Left}, true, true, false}};
        config.approaches[1].lanes = {
            {941, "E-free", {MovementType::Straight}, true, true, true}};
        config.lane_connections.clear();
        config.lane_connections.push_back({ApproachId::North, 0, MovementType::Left, ApproachId::East, 0});

        SimulatorEngine engine(config, 4.0, 10.0, 10.0);
        engine.setController(std::make_unique<FixedConflictController>(), SimulatorEngine::ControlMode::Basic);
        engine.setSpawnLaneFilter(TrafficGenerator::SpawnLaneFilter{ApproachId::North, 0});
        engine.start();

        bool saw_left_crossing = false;
        for (int i = 0; i < 220; ++i)
        {
            engine.tick(0.1);
            nlohmann::json doc = nlohmann::json::parse(engine.getSnapshotJson());
            for (const auto &vehicle : doc["lanes"]["north"])
            {
                if (vehicle.value("movement", std::string{}) == "left" && vehicle.value("crossing", false))
                {
                    saw_left_crossing = true;
                }
            }
        }

        REQUIRE(engine.getMetrics().queue_lengths[0] > 0);
        REQUIRE(engine.getMetrics().vehicles_crossed == 0);
        REQUIRE_FALSE(saw_left_crossing);
    }

    SECTION("existing connected destination lane")
    {
        IntersectionConfig config = makeDefaultIntersectionConfig();
        config.approaches[0].lanes = {
            {950, "N-left-only-no-light", {MovementType::Left}, true, true, false}};
        config.approaches[1].lanes = {
            {951, "E-main-0", {MovementType::Straight}, true, true, true},
            {952, "E-main-1", {MovementType::Straight}, true, true, true}};
        config.lane_connections.clear();
        config.lane_connections.push_back({ApproachId::North, 0, MovementType::Left, ApproachId::East, 0});

        SimulatorEngine engine(config, 4.0, 10.0, 10.0);
        engine.setController(std::make_unique<FixedConflictController>(), SimulatorEngine::ControlMode::Basic);
        engine.setSpawnLaneFilter(TrafficGenerator::SpawnLaneFilter{ApproachId::North, 0});
        engine.start();

        bool saw_left_crossing = false;
        for (int i = 0; i < 220; ++i)
        {
            engine.tick(0.1);
            nlohmann::json doc = nlohmann::json::parse(engine.getSnapshotJson());
            for (const auto &vehicle : doc["lanes"]["north"])
            {
                if (vehicle.value("movement", std::string{}) == "left" && vehicle.value("crossing", false))
                {
                    saw_left_crossing = true;
                }
            }
        }

        REQUIRE(engine.getMetrics().queue_lengths[0] > 0);
        REQUIRE(engine.getMetrics().vehicles_crossed == 0);
        REQUIRE_FALSE(saw_left_crossing);
    }
}

TEST_CASE("Left-only lane with traffic light crosses when protected", "[engine][regression][left-turn]")
{
    class ProtectedLeftController : public ITrafficLightController
    {
    public:
        void tick(double) override {}
        IntersectionState getCurrentState() const override
        {
            IntersectionState s{};
            s.north = LightState::Green;
            s.south = LightState::Red;
            s.east = LightState::Red;
            s.west = LightState::Red;
            return s;
        }
        void reset() override {}
    };

    IntersectionConfig config = makeDefaultIntersectionConfig();
    config.approaches[0].lanes = {
        {960, "N-left-only-protected", {MovementType::Left}, true, true, true}};
    config.approaches[1].lanes = {
        {961, "E-main-0", {MovementType::Straight}, true, true, true},
        {962, "E-main-1", {MovementType::Straight}, true, true, true}};
    config.lane_connections.clear();
    config.lane_connections.push_back({ApproachId::North, 0, MovementType::Left, ApproachId::East, 0});

    SimulatorEngine engine(config, 3.0, 10.0, 10.0);
    engine.setController(std::make_unique<ProtectedLeftController>(), SimulatorEngine::ControlMode::Basic);
    engine.setSpawnLaneFilter(TrafficGenerator::SpawnLaneFilter{ApproachId::North, 0});
    engine.start();

    bool saw_left_crossing = false;
    for (int i = 0; i < 300; ++i)
    {
        engine.tick(0.1);
        nlohmann::json doc = nlohmann::json::parse(engine.getSnapshotJson());
        for (const auto &vehicle : doc["lanes"]["north"])
        {
            if (vehicle.value("movement", std::string{}) == "left" && vehicle.value("crossing", false))
            {
                saw_left_crossing = true;
                break;
            }
        }
        if (saw_left_crossing)
        {
            break;
        }
    }

    REQUIRE(saw_left_crossing);
}

TEST_CASE("Left-only lane with traffic light uses red dedicated left signal when opposing approach is active", "[engine][regression][left-turn]")
{
    class UnprotectedLeftController : public ITrafficLightController
    {
    public:
        void tick(double) override {}
        IntersectionState getCurrentState() const override
        {
            IntersectionState s{};
            s.north = LightState::Green;
            s.south = LightState::Green;
            s.east = LightState::Red;
            s.west = LightState::Red;
            return s;
        }
        void reset() override {}
    };

    IntersectionConfig config = makeDefaultIntersectionConfig();
    config.approaches[0].lanes = {
        {970, "N-left-only-signalized", {MovementType::Left}, true, true, true}};
    config.approaches[1].lanes = {
        {971, "E-main-0", {MovementType::Straight}, true, true, true}};
    config.lane_connections.clear();
    config.lane_connections.push_back({ApproachId::North, 0, MovementType::Left, ApproachId::East, 0});
    config.lane_connections.push_back({ApproachId::South, 0, MovementType::Straight, ApproachId::North, 0});

    SimulatorEngine engine(config, 3.0, 10.0, 10.0);
    engine.setController(std::make_unique<UnprotectedLeftController>(), SimulatorEngine::ControlMode::Basic);
    engine.start();

    bool saw_left_vehicle = false;
    bool saw_left_signal_red = false;
    for (int i = 0; i < 300; ++i)
    {
        engine.tick(0.1);
        nlohmann::json doc = nlohmann::json::parse(engine.getSnapshotJson());
        const std::string left_signal = doc["lights"].value("turnNorthEast", std::string{"red"});
        const bool south_has_demand = doc["metrics"]["queues"].value("south", 0) > 0;
        for (const auto &vehicle : doc["lanes"]["north"])
        {
            if (vehicle.value("movement", std::string{}) == "left")
            {
                saw_left_vehicle = true;
                if (south_has_demand && left_signal == "red")
                {
                    saw_left_signal_red = true;
                }
            }
        }
    }

    REQUIRE(saw_left_vehicle);
    REQUIRE(saw_left_signal_red);
}

TEST_CASE("Dedicated left lane gets explicit left green while main can stay green", "[engine][regression][left-signal]")
{
    class NSMainGreenController : public ITrafficLightController
    {
    public:
        void tick(double) override {}
        IntersectionState getCurrentState() const override
        {
            IntersectionState s{};
            s.north = LightState::Green;
            s.south = LightState::Red;
            s.east = LightState::Red;
            s.west = LightState::Red;
            return s;
        }
        void reset() override {}
    };

    IntersectionConfig config = makeDefaultIntersectionConfig();
    config.approaches[0].lanes = {
        {980, "N-straight", {MovementType::Straight}, true, true, true},
        {981, "N-left-only", {MovementType::Left}, true, true, true}};
    config.approaches[2].lanes = {
        {982, "S-straight", {MovementType::Straight}, true, true, true}};
    config.lane_connections.clear();
    config.lane_connections.push_back({ApproachId::North, 0, MovementType::Straight, ApproachId::South, 0});
    config.lane_connections.push_back({ApproachId::North, 1, MovementType::Left, ApproachId::East, 0});

    SimulatorEngine engine(config, 4.0, 10.0, 10.0);
    engine.setController(std::make_unique<NSMainGreenController>(), SimulatorEngine::ControlMode::Basic);
    engine.setSpawnLaneFilter(TrafficGenerator::SpawnLaneFilter{ApproachId::North, 1});
    engine.start();

    bool saw_left_crossing = false;
    bool saw_left_green = false;
    for (int i = 0; i < 280; ++i)
    {
        engine.tick(0.1);
        auto snapshot = engine.getSnapshot();
        if (snapshot.lights.turnNorthEast == LightState::Green)
        {
            saw_left_green = true;
        }

        nlohmann::json doc = nlohmann::json::parse(engine.getSnapshotJson());
        for (const auto &vehicle : doc["lanes"]["north"])
        {
            if (vehicle.value("movement", std::string{}) == "left" && vehicle.value("crossing", false))
            {
                saw_left_crossing = true;
                break;
            }
        }
        if (saw_left_crossing)
        {
            break;
        }
    }

    REQUIRE(saw_left_green);
    REQUIRE(saw_left_crossing);
}

TEST_CASE("Effective lights set opposite approach red when no demand there", "[engine][regression][demand]")
{
    SimulatorEngine engine(2.5, 10.0, 10.0);
    engine.setSpawnLaneFilter(TrafficGenerator::SpawnLaneFilter{ApproachId::North, 0});
    engine.start();

    bool saw_north_demand = false;
    bool saw_south_forced_red = false;
    for (int i = 0; i < 220; ++i)
    {
        engine.tick(0.1);
        auto snapshot = engine.getSnapshot();
        if (snapshot.metrics.queue_lengths[0] > 0)
        {
            saw_north_demand = true;
            if (snapshot.metrics.queue_lengths[2] == 0 && snapshot.lights.south == LightState::Red)
            {
                saw_south_forced_red = true;
                break;
            }
        }
    }

    REQUIRE(saw_north_demand);
    REQUIRE(saw_south_forced_red);
}

TEST_CASE("Dedicated left crossing keeps opposing straight red for safety", "[engine][regression][left-signal]")
{
    class NSMainGreenController : public ITrafficLightController
    {
    public:
        void tick(double) override {}
        IntersectionState getCurrentState() const override
        {
            IntersectionState s{};
            s.north = LightState::Green;
            s.south = LightState::Green;
            s.east = LightState::Red;
            s.west = LightState::Red;
            return s;
        }
        void reset() override {}
    };

    IntersectionConfig config = makeDefaultIntersectionConfig();
    config.approaches[0].lanes = {
        {990, "N-left-only", {MovementType::Left}, true, true, true}};
    config.approaches[2].lanes = {
        {991, "S-straight", {MovementType::Straight}, true, true, true}};
    config.lane_connections.clear();
    config.lane_connections.push_back({ApproachId::North, 0, MovementType::Left, ApproachId::East, 0});
    config.lane_connections.push_back({ApproachId::South, 0, MovementType::Straight, ApproachId::North, 0});

    SimulatorEngine engine(config, 4.0, 10.0, 10.0);
    engine.setController(std::make_unique<NSMainGreenController>(), SimulatorEngine::ControlMode::Basic);
    engine.setSpawnLaneFilter(TrafficGenerator::SpawnLaneFilter{ApproachId::North, 0});
    engine.start();

    bool saw_left_crossing = false;
    bool saw_opposing_red_during_crossing = false;
    for (int i = 0; i < 320; ++i)
    {
        engine.tick(0.1);
        nlohmann::json doc = nlohmann::json::parse(engine.getSnapshotJson());
        for (const auto &vehicle : doc["lanes"]["north"])
        {
            if (vehicle.value("movement", std::string{}) == "left" && vehicle.value("crossing", false))
            {
                saw_left_crossing = true;
                if (doc["lights"].value("south", std::string{"red"}) == "red")
                {
                    saw_opposing_red_during_crossing = true;
                }
            }
        }
        if (saw_left_crossing && saw_opposing_red_during_crossing)
        {
            break;
        }
    }

    REQUIRE(saw_left_crossing);
    REQUIRE(saw_opposing_red_during_crossing);
}

TEST_CASE("North left demand under green forces south main red", "[engine][regression][left-signal]")
{
    class NSMainGreenController : public ITrafficLightController
    {
    public:
        void tick(double) override {}
        IntersectionState getCurrentState() const override
        {
            IntersectionState s{};
            s.north = LightState::Green;
            s.south = LightState::Green;
            s.east = LightState::Red;
            s.west = LightState::Red;
            return s;
        }
        void reset() override {}
    };

    IntersectionConfig config = makeDefaultIntersectionConfig();
    config.approaches[0].lanes = {
        {997, "N-left-only", {MovementType::Left}, true, true, true}};
    config.approaches[2].lanes = {
        {998, "S-straight", {MovementType::Straight}, true, true, true}};
    config.lane_connections.clear();
    config.lane_connections.push_back({ApproachId::North, 0, MovementType::Left, ApproachId::East, 0});
    config.lane_connections.push_back({ApproachId::South, 0, MovementType::Straight, ApproachId::North, 0});

    SimulatorEngine engine(config, 4.0, 10.0, 10.0);
    engine.setController(std::make_unique<NSMainGreenController>(), SimulatorEngine::ControlMode::Basic);
    engine.setSpawnLaneFilter(TrafficGenerator::SpawnLaneFilter{ApproachId::North, 0});
    engine.start();

    bool saw_left_vehicle = false;
    bool saw_south_red = false;
    for (int i = 0; i < 220; ++i)
    {
        engine.tick(0.1);
        nlohmann::json doc = nlohmann::json::parse(engine.getSnapshotJson());
        for (const auto &vehicle : doc["lanes"]["north"])
        {
            if (vehicle.value("movement", std::string{}) == "left")
            {
                saw_left_vehicle = true;
                if (doc["lights"].value("south", std::string{"red"}) == "red")
                {
                    saw_south_red = true;
                }
            }
        }
        if (saw_left_vehicle && saw_south_red)
        {
            break;
        }
    }

    REQUIRE(saw_left_vehicle);
    REQUIRE(saw_south_red);
}

TEST_CASE("Exclusive dedicated right turn can actuate without waiting full opposing red cycle", "[engine][regression][right-turn]")
{
    class OpposingGreenController : public ITrafficLightController
    {
    public:
        void tick(double) override {}
        IntersectionState getCurrentState() const override
        {
            IntersectionState s{};
            s.east = LightState::Green;
            return s;
        }
        void reset() override {}
    };

    IntersectionConfig config = makeDefaultIntersectionConfig();
    config.approaches[0].lanes = {
        {995, "N-right-only", {MovementType::Right}, true, true, true}};
    config.approaches[3].lanes = {
        {996, "W-exclusive-target", {MovementType::Straight}, true, true, true}};
    config.lane_connections.clear();
    config.lane_connections.push_back({ApproachId::North, 0, MovementType::Right, ApproachId::West, 0});

    SimulatorEngine engine(config, 3.0, 10.0, 10.0);
    engine.setController(std::make_unique<OpposingGreenController>(), SimulatorEngine::ControlMode::Basic);
    engine.setSpawnLaneFilter(TrafficGenerator::SpawnLaneFilter{ApproachId::North, 0});
    engine.start();

    bool saw_right_green = false;
    for (int i = 0; i < 80; ++i)
    {
        engine.tick(0.1);
        auto snapshot = engine.getSnapshot();
        if (snapshot.metrics.queue_lengths[0] > 0 && snapshot.lights.turnNorthWest == LightState::Green)
        {
            saw_right_green = true;
            break;
        }
    }

    REQUIRE(saw_right_green);
}

TEST_CASE("SafetyChecker validates default configurable intersection", "[safety][config]")
{
    IntersectionConfig config = makeDefaultIntersectionConfig();
    SafetyChecker checker(config);
    REQUIRE(checker.isConfigValid());
}

TEST_CASE("IntersectionConfig JSON roundtrip keeps key fields", "[config][json]")
{
    IntersectionConfig config = makeDefaultIntersectionConfig();
    config.approaches[0].lanes[0].connected_to_intersection = true;
    config.approaches[0].lanes[0].has_traffic_light = true;
    config.approaches[0].lanes[1].connected_to_intersection = false;
    config.approaches[0].lanes[1].has_traffic_light = false;
    config.signal_groups = {
        {1, "NS-straight", {laneIdFor(ApproachId::North, 0), laneIdFor(ApproachId::South, 0)}, {MovementType::Straight}, 9.0, 2.0}};

    std::string json_text = intersectionConfigToJson(config);
    ConfigParseResult parsed = intersectionConfigFromJson(json_text);

    REQUIRE(parsed.ok);
    REQUIRE(parsed.config.approaches[0].id == ApproachId::North);
    REQUIRE(parsed.config.approaches[0].lanes.size() == 3);
    REQUIRE(parsed.config.approaches[0].lanes[0].connected_to_intersection);
    REQUIRE(parsed.config.approaches[0].lanes[0].has_traffic_light);
    REQUIRE_FALSE(parsed.config.approaches[0].lanes[1].connected_to_intersection);
    REQUIRE_FALSE(parsed.config.approaches[0].lanes[1].has_traffic_light);
    REQUIRE_FALSE(parsed.config.lane_connections.empty());
    REQUIRE(parsed.config.lane_connections[0].from_approach == ApproachId::North);
    REQUIRE(parsed.config.signal_groups.size() == 1);
    REQUIRE(parsed.config.signal_groups[0].controlled_lanes.size() == 2);
}

TEST_CASE("IntersectionConfig JSON parser rejects malformed structure", "[config][json]")
{
    std::string invalid_json = R"JSON({"approaches":[{"id":"north","lanes":[]}],"signal_groups":[]})JSON";
    ConfigParseResult parsed = intersectionConfigFromJson(invalid_json);

    REQUIRE_FALSE(parsed.ok);
    REQUIRE_FALSE(parsed.errors.empty());
}

TEST_CASE("SafetyChecker rejects invalid lane configuration", "[safety][config]")
{
    IntersectionConfig invalid = makeDefaultIntersectionConfig();
    invalid.approaches[0].lanes[0].allowed_movements.clear();

    SafetyChecker checker(invalid);
    REQUIRE_FALSE(checker.isConfigValid());
}

TEST_CASE("SafetyChecker movement conflict rules cover main corridors", "[safety][config]")
{
    SafetyChecker checker;

    REQUIRE(checker.hasMovementConflict(ApproachId::North, MovementType::Straight,
                                        ApproachId::East, MovementType::Straight));

    REQUIRE_FALSE(checker.hasMovementConflict(ApproachId::North, MovementType::Straight,
                                              ApproachId::South, MovementType::Straight));

    REQUIRE(checker.hasMovementConflict(ApproachId::North, MovementType::Left,
                                        ApproachId::South, MovementType::Left));
}

TEST_CASE("SimulatorEngine stores custom intersection config", "[engine][config]")
{
    IntersectionConfig config = makeDefaultIntersectionConfig();
    config.approaches[0].lanes.push_back({12, "N-3", {MovementType::Left}, true});

    SimulatorEngine engine(config, 0.5, 10.0, 10.0);
    REQUIRE(engine.getIntersectionConfig().approaches[0].lanes.size() == 4);
}

TEST_CASE("SafetyChecker detects conflicting active signal groups", "[safety][config][signal-groups]")
{
    IntersectionConfig config = makeDefaultIntersectionConfig();
    config.signal_groups = {
        {1, "NS-straight", {laneIdFor(ApproachId::North, 0), laneIdFor(ApproachId::South, 0)}, {MovementType::Straight}, 10.0, 2.0},
        {2, "EW-straight", {laneIdFor(ApproachId::East, 0), laneIdFor(ApproachId::West, 0)}, {MovementType::Straight}, 10.0, 2.0}};

    SafetyChecker checker(config);
    REQUIRE(checker.isConfigValid());
    REQUIRE_FALSE(checker.areSignalGroupsConflictFree({1, 2}));
}

TEST_CASE("SafetyChecker accepts non-conflicting active signal groups", "[safety][config][signal-groups]")
{
    IntersectionConfig config = makeDefaultIntersectionConfig();
    config.signal_groups = {
        {1, "NS-straight", {laneIdFor(ApproachId::North, 0), laneIdFor(ApproachId::South, 0)}, {MovementType::Straight}, 10.0, 2.0}};

    SafetyChecker checker(config);
    REQUIRE(checker.isConfigValid());
    REQUIRE(checker.areSignalGroupsConflictFree({1}));
}

TEST_CASE("SafetyChecker rejects unknown signal groups in active set", "[safety][config][signal-groups]")
{
    IntersectionConfig config = makeDefaultIntersectionConfig();
    config.signal_groups = {
        {11, "NS-straight", {laneIdFor(ApproachId::North, 0), laneIdFor(ApproachId::South, 0)}, {MovementType::Straight}, 10.0, 2.0}};

    SafetyChecker checker(config);
    REQUIRE(checker.isConfigValid());
    REQUIRE_FALSE(checker.areSignalGroupsConflictFree({99}));
}

TEST_CASE("SimulatorEngine enforces config signal-group conflicts at runtime", "[engine][config][signal-groups]")
{
    class OpposingProtectedLeftController : public ITrafficLightController
    {
    public:
        void tick(double) override {}

        IntersectionState getCurrentState() const override
        {
            IntersectionState s{};
            s.turnNorthEast = LightState::Green;
            s.turnSouthWest = LightState::Green;
            return s;
        }

        void reset() override {}
    };

    IntersectionConfig config = makeDefaultIntersectionConfig();
    config.approaches[0].lanes[0].allowed_movements = {MovementType::Straight, MovementType::Left};
    config.approaches[2].lanes[0].allowed_movements = {MovementType::Straight, MovementType::Left};
    config.signal_groups = {
        {101, "N-left", {laneIdFor(ApproachId::North, 0)}, {MovementType::Left}, 8.0, 2.0},
        {102, "S-left", {laneIdFor(ApproachId::South, 0)}, {MovementType::Left}, 8.0, 2.0}};

    SimulatorEngine engine(config, 0.5, 10.0, 10.0);
    engine.setController(std::make_unique<OpposingProtectedLeftController>(), SimulatorEngine::ControlMode::Basic);

    engine.start();
    engine.tick(0.1);

    REQUIRE(engine.getControlMode() == SimulatorEngine::ControlMode::NullControl);
    REQUIRE(engine.getMetrics().safety_violations >= 1);
}

TEST_CASE("SimulatorEngine does not infer left signal groups from main green", "[engine][config][signal-groups]")
{
    class MainNSGreenController : public ITrafficLightController
    {
    public:
        void tick(double) override {}

        IntersectionState getCurrentState() const override
        {
            IntersectionState s{};
            s.north = LightState::Green;
            s.south = LightState::Green;
            return s;
        }

        void reset() override {}
    };

    IntersectionConfig config = makeDefaultIntersectionConfig();
    config.approaches[0].lanes[0].allowed_movements = {MovementType::Straight, MovementType::Left};
    config.approaches[2].lanes[0].allowed_movements = {MovementType::Straight, MovementType::Left};
    config.signal_groups = {
        {201, "N-left", {laneIdFor(ApproachId::North, 0)}, {MovementType::Left}, 8.0, 2.0},
        {202, "S-left", {laneIdFor(ApproachId::South, 0)}, {MovementType::Left}, 8.0, 2.0}};

    SimulatorEngine engine(config, 0.5, 10.0, 10.0);
    engine.setController(std::make_unique<MainNSGreenController>(), SimulatorEngine::ControlMode::Basic);

    engine.start();
    engine.tick(0.1);

    REQUIRE(engine.getControlMode() == SimulatorEngine::ControlMode::Basic);
    REQUIRE(engine.getMetrics().safety_violations == 0);
}

TEST_CASE("SimulatorEngine keeps straight-only throughput under signal-group control", "[engine][config][throughput]")
{
    IntersectionConfig config = makeDefaultIntersectionConfig();

    config.approaches[0].lanes = {{laneIdFor(ApproachId::North, 0), "N-0", {MovementType::Straight}, true, true, true}};
    config.approaches[1].lanes = {{laneIdFor(ApproachId::East, 0), "E-0", {MovementType::Straight}, true, true, true}};
    config.approaches[2].lanes = {{laneIdFor(ApproachId::South, 0), "S-0", {MovementType::Straight}, true, true, true}};
    config.approaches[3].lanes = {{laneIdFor(ApproachId::West, 0), "W-0", {MovementType::Straight}, true, true, true}};
    for (auto &approach : config.approaches)
    {
        approach.to_lane_count = 1;
    }

    config.lane_connections = {
        {ApproachId::North, 0, MovementType::Straight, ApproachId::South, 0},
        {ApproachId::South, 0, MovementType::Straight, ApproachId::North, 0},
        {ApproachId::East, 0, MovementType::Straight, ApproachId::West, 0},
        {ApproachId::West, 0, MovementType::Straight, ApproachId::East, 0}};

    config.signal_groups = {
        {401,
         "NS-straight",
         {laneIdFor(ApproachId::North, 0), laneIdFor(ApproachId::South, 0)},
         {MovementType::Straight},
         1.5,
         0.5},
        {402,
         "EW-straight",
         {laneIdFor(ApproachId::East, 0), laneIdFor(ApproachId::West, 0)},
         {MovementType::Straight},
         1.5,
         0.5}};

    SimulatorEngine engine(config, 3.0, 10.0, 10.0);
    engine.start();

    bool observed_progress = false;
    size_t previous_crossed = 0;

    for (int i = 0; i < 300; ++i)
    {
        engine.tick(0.1);
        const auto metrics = engine.getMetrics();
        if (metrics.vehicles_crossed > previous_crossed)
        {
            observed_progress = true;
        }
        previous_crossed = metrics.vehicles_crossed;
        REQUIRE(engine.getControlMode() == SimulatorEngine::ControlMode::Basic);
    }

    const auto final_metrics = engine.getMetrics();
    REQUIRE(observed_progress);
    REQUIRE(final_metrics.vehicles_crossed > 10);
    REQUIRE(final_metrics.safety_violations == 0);
}

TEST_CASE("SimulatorEngine keeps green active for at least three seconds", "[engine][timing][green-min]")
{
    class OneShotNorthGreenController : public ITrafficLightController
    {
    public:
        void tick(double) override
        {
            if (!seen_first_tick)
            {
                seen_first_tick = true;
            }
            else
            {
                issued_green = true;
            }
        }

        IntersectionState getCurrentState() const override
        {
            IntersectionState s{};
            s.north = issued_green ? LightState::Red : LightState::Green;
            return s;
        }

        void reset() override
        {
            seen_first_tick = false;
            issued_green = false;
        }

    private:
        bool seen_first_tick = false;
        bool issued_green = false;
    };

    IntersectionConfig config = makeDefaultIntersectionConfig();
    config.approaches[0].lanes = {
        {950, "N-straight", {MovementType::Straight}, true, true, true}};
    config.approaches[1].lanes = {
        {951, "E-disconnected", {MovementType::Straight}, true, false, false}};
    config.approaches[2].lanes = {
        {952, "S-disconnected", {MovementType::Straight}, true, false, false}};
    config.approaches[3].lanes = {
        {953, "W-disconnected", {MovementType::Straight}, true, false, false}};
    config.lane_connections = {
        {ApproachId::North, 0, MovementType::Straight, ApproachId::South, 0}};

    SimulatorEngine engine(config, 10.0, 10.0, 10.0);
    engine.setController(std::make_unique<OneShotNorthGreenController>(), SimulatorEngine::ControlMode::Basic);
    engine.setSpawnLaneFilter(TrafficGenerator::SpawnLaneFilter{ApproachId::North, 0});
    engine.start();

    double first_green_time = -1.0;
    double first_non_green_after_green_time = -1.0;

    for (int i = 0; i < 80; ++i)
    {
        engine.tick(0.1);
        const auto snapshot = engine.getSnapshot();

        if (snapshot.lights.north == LightState::Green && first_green_time < 0.0)
        {
            first_green_time = snapshot.sim_time;
        }

        if (first_green_time >= 0.0 &&
            snapshot.lights.north != LightState::Green &&
            first_non_green_after_green_time < 0.0)
        {
            first_non_green_after_green_time = snapshot.sim_time;
        }
    }

    REQUIRE(first_green_time >= 0.0);
    if (first_non_green_after_green_time >= 0.0)
    {
        REQUIRE(first_non_green_after_green_time - first_green_time >= 3.0);
    }
    else
    {
        REQUIRE(engine.getSnapshot().sim_time - first_green_time >= 3.0);
    }
}

TEST_CASE("SimulatorEngine scheduler conflict matrix reports known crossing conflicts", "[engine][scheduler][conflicts]")
{
    SimulatorEngine engine(makeDefaultIntersectionConfig(), 0.0, 10.0, 10.0);
    nlohmann::json doc = nlohmann::json::parse(engine.getSnapshotJson());

    REQUIRE(doc.contains("scheduler"));
    REQUIRE(doc["scheduler"].contains("routes"));
    REQUIRE(doc["scheduler"]["routes"].is_array());

    auto find_route = [&](const std::string &name) -> const nlohmann::json *
    {
        for (const auto &route : doc["scheduler"]["routes"])
        {
            if (route.value("route", std::string{}) == name)
            {
                return &route;
            }
        }
        return nullptr;
    };

    const nlohmann::json *n_to_s = find_route("N->S");
    const nlohmann::json *e_to_w = find_route("E->W");
    const nlohmann::json *n_to_e = find_route("N->E");
    const nlohmann::json *s_to_w = find_route("S->W");
    const nlohmann::json *s_to_n = find_route("S->N");
    REQUIRE(n_to_s != nullptr);
    REQUIRE(e_to_w != nullptr);
    REQUIRE(n_to_e == nullptr);
    REQUIRE(s_to_w == nullptr);
    REQUIRE(s_to_n != nullptr);
    REQUIRE((*n_to_s).contains("conflicts"));
    REQUIRE((*e_to_w).contains("conflicts"));
    REQUIRE((*s_to_n).contains("conflicts"));
    REQUIRE((*n_to_s)["conflicts"].is_array());
    REQUIRE((*e_to_w)["conflicts"].is_array());
    REQUIRE((*s_to_n)["conflicts"].is_array());

    auto contains_conflict = [](const nlohmann::json &route, const std::string &target)
    {
        if (!route.contains("conflicts") || !route["conflicts"].is_array())
        {
            return false;
        }

        for (const auto &entry : route["conflicts"])
        {
            if (entry.is_string() && entry.get<std::string>() == target)
            {
                return true;
            }
        }
        return false;
    };

    REQUIRE_FALSE((*n_to_s)["conflicts"].empty());
    REQUIRE(contains_conflict(*n_to_s, "E->W"));
    REQUIRE(contains_conflict(*e_to_w, "N->S"));
    REQUIRE_FALSE(contains_conflict(*n_to_s, "E->N"));
    REQUIRE_FALSE(contains_conflict(*n_to_s, "S->N"));
    REQUIRE_FALSE(contains_conflict(*n_to_s, "W->S"));
}

TEST_CASE("SimulatorEngine scheduler lists only configured approach-movement routes", "[engine][scheduler][routes]")
{
    IntersectionConfig config = makeDefaultIntersectionConfig();

    config.approaches[0].lanes = {
        {900, "N-left", {MovementType::Left}, true, true, true}};
    config.approaches[2].lanes = {
        {920, "S-straight", {MovementType::Straight}, true, true, true}};
    config.approaches[1].lanes = {
        {910, "E-straight", {MovementType::Straight}, true, true, true}};
    config.approaches[3].lanes = {
        {930, "W-straight", {MovementType::Straight}, true, true, true}};
    for (auto &approach : config.approaches)
    {
        approach.to_lane_count = 1;
    }

    config.lane_connections = {
        {ApproachId::North, 0, MovementType::Left, ApproachId::East, 0},
        {ApproachId::South, 0, MovementType::Straight, ApproachId::North, 0},
        {ApproachId::East, 0, MovementType::Straight, ApproachId::West, 0},
        {ApproachId::West, 0, MovementType::Straight, ApproachId::East, 0}};

    SimulatorEngine engine(config, 0.0, 10.0, 10.0);
    nlohmann::json doc = nlohmann::json::parse(engine.getSnapshotJson());

    auto find_route = [&](const std::string &name) -> const nlohmann::json *
    {
        for (const auto &route : doc["scheduler"]["routes"])
        {
            if (route.value("route", std::string{}) == name)
            {
                return &route;
            }
        }
        return nullptr;
    };

    const nlohmann::json *n_to_e_route = find_route("N->E");
    const nlohmann::json *s_to_n_route = find_route("S->N");
    const nlohmann::json *s_to_w_route = find_route("S->W");
    const nlohmann::json *w_to_n_route = find_route("W->N");
    REQUIRE(n_to_e_route != nullptr);
    REQUIRE(s_to_n_route != nullptr);
    REQUIRE(s_to_w_route == nullptr);
    REQUIRE(w_to_n_route == nullptr);
}

TEST_CASE("SimulatorEngine effective lights enforce orange step and minimum orange duration", "[engine][transition][effective]")
{
    SimulatorEngine engine(makeDefaultIntersectionConfig(), 8.0, 10.0, 10.0);
    engine.start();

    auto light_values = [](const nlohmann::json &lights)
    {
        return std::array<std::string, 12>{
            lights.value("north", std::string{"red"}),
            lights.value("east", std::string{"red"}),
            lights.value("south", std::string{"red"}),
            lights.value("west", std::string{"red"}),
            lights.value("turnSouthEast", std::string{"red"}),
            lights.value("turnNorthWest", std::string{"red"}),
            lights.value("turnWestSouth", std::string{"red"}),
            lights.value("turnEastNorth", std::string{"red"}),
            lights.value("turnNorthEast", std::string{"red"}),
            lights.value("turnSouthWest", std::string{"red"}),
            lights.value("turnEastSouth", std::string{"red"}),
            lights.value("turnWestNorth", std::string{"red"})};
    };

    nlohmann::json first = nlohmann::json::parse(engine.getSnapshotJson());
    auto previous = light_values(first["lights"]);

    std::array<double, 12> orange_elapsed{};
    orange_elapsed.fill(0.0);
    constexpr double dt = 0.1;

    for (int i = 0; i < 400; ++i)
    {
        engine.tick(dt);
        nlohmann::json doc = nlohmann::json::parse(engine.getSnapshotJson());
        auto current = light_values(doc["lights"]);

        for (std::size_t li = 0; li < current.size(); ++li)
        {
            const std::string &prev = previous[li];
            const std::string &now = current[li];

            const bool direct_green_to_red = (prev == "green" && now == "red");
            const bool direct_red_to_orange = (prev == "red" && now == "orange");
            REQUIRE_FALSE(direct_green_to_red);
            REQUIRE_FALSE(direct_red_to_orange);

            if (prev == "orange" && now == "orange")
            {
                orange_elapsed[li] += dt;
            }
            if (prev != "orange" && now == "orange")
            {
                orange_elapsed[li] = 0.0;
            }
            if (prev == "orange" && now == "red")
            {
                REQUIRE(orange_elapsed[li] + dt >= SafetyChecker::ORANGE_DURATION);
                orange_elapsed[li] = 0.0;
            }
            if (now != "orange" && prev != "orange")
            {
                orange_elapsed[li] = 0.0;
            }
        }

        previous = current;
    }
}

TEST_CASE("SimulatorEngine prevents starvation for South to North straight under competing north-left demand", "[engine][fairness][starvation]")
{
    class NSMainGreenController : public ITrafficLightController
    {
    public:
        void tick(double) override {}
        IntersectionState getCurrentState() const override
        {
            IntersectionState s{};
            s.north = LightState::Green;
            s.south = LightState::Green;
            s.east = LightState::Red;
            s.west = LightState::Red;
            return s;
        }
        void reset() override {}
    };

    IntersectionConfig config = makeDefaultIntersectionConfig();
    config.approaches[0].lanes = {
        {910, "N-left-only", {MovementType::Left}, true, true, true}};
    config.approaches[1].lanes = {
        {911, "E-disconnected", {MovementType::Straight}, true, false, false}};
    config.approaches[2].lanes = {
        {920, "S-straight-only", {MovementType::Straight}, true, true, true}};
    config.approaches[3].lanes = {
        {921, "W-disconnected", {MovementType::Straight}, true, false, false}};
    config.lane_connections = {
        {ApproachId::North, 0, MovementType::Left, ApproachId::East, 0},
        {ApproachId::South, 0, MovementType::Straight, ApproachId::North, 0}};

    SimulatorEngine engine(config, 8.0, 10.0, 10.0);
    engine.setController(std::make_unique<NSMainGreenController>(), SimulatorEngine::ControlMode::Basic);
    engine.start();

    bool saw_north_left_demand = false;
    bool saw_south_straight_service_window = false;

    for (int i = 0; i < 700; ++i)
    {
        engine.tick(0.1);
        nlohmann::json doc = nlohmann::json::parse(engine.getSnapshotJson());

        for (const auto &vehicle : doc["lanes"]["north"])
        {
            if (vehicle.value("movement", std::string{}) == "left")
            {
                saw_north_left_demand = true;
            }
        }

        bool south_straight_waiting = false;
        for (const auto &vehicle : doc["lanes"]["south"])
        {
            if (vehicle.value("movement", std::string{}) == "straight")
            {
                south_straight_waiting = true;
            }
        }

        if (saw_north_left_demand &&
            south_straight_waiting &&
            doc["lights"].value("south", std::string{"red"}) == "green")
        {
            saw_south_straight_service_window = true;
        }

        REQUIRE(engine.getControlMode() == SimulatorEngine::ControlMode::Basic);
    }

    REQUIRE(saw_north_left_demand);
    REQUIRE(saw_south_straight_service_window);
    REQUIRE(engine.getMetrics().safety_violations == 0);
}

TEST_CASE("SimulatorEngine prevents starvation for North to East left under competing south-right demand", "[engine][fairness][starvation]")
{
    class AllRedController : public ITrafficLightController
    {
    public:
        void tick(double) override {}
        IntersectionState getCurrentState() const override
        {
            return IntersectionState{};
        }
        void reset() override {}
    };

    IntersectionConfig config = makeDefaultIntersectionConfig();
    config.approaches[0].lanes = {
        {930, "N-left-only", {MovementType::Left}, true, true, true}};
    config.approaches[1].lanes = {
        {931, "E-disconnected", {MovementType::Straight}, true, false, false}};
    config.approaches[2].lanes = {
        {940, "S-right-only", {MovementType::Right}, true, true, true}};
    config.approaches[3].lanes = {
        {941, "W-disconnected", {MovementType::Straight}, true, false, false}};
    config.lane_connections = {
        {ApproachId::North, 0, MovementType::Left, ApproachId::East, 0},
        {ApproachId::South, 0, MovementType::Right, ApproachId::East, 0}};

    SimulatorEngine engine(config, 8.0, 10.0, 10.0);
    engine.setController(std::make_unique<AllRedController>(), SimulatorEngine::ControlMode::Basic);
    engine.start();

    bool saw_south_right_demand = false;
    bool saw_north_left_demand = false;
    bool saw_conflicting_right_paused = false;
    bool saw_north_left_green = false;

    for (int i = 0; i < 900; ++i)
    {
        engine.tick(0.1);
        nlohmann::json doc = nlohmann::json::parse(engine.getSnapshotJson());

        bool south_right_waiting = false;
        for (const auto &vehicle : doc["lanes"]["south"])
        {
            if (vehicle.value("movement", std::string{}) == "right" && !vehicle.value("crossing", false))
            {
                saw_south_right_demand = true;
                south_right_waiting = true;
            }
        }

        bool north_left_waiting = false;
        for (const auto &vehicle : doc["lanes"]["north"])
        {
            if (vehicle.value("movement", std::string{}) == "left")
            {
                saw_north_left_demand = true;
                north_left_waiting = true;
            }
        }

        if (south_right_waiting &&
            north_left_waiting &&
            doc["lights"].value("turnSouthEast", std::string{"red"}) == "red")
        {
            saw_conflicting_right_paused = true;
        }

        if (north_left_waiting &&
            doc["lights"].value("turnNorthEast", std::string{"red"}) == "green")
        {
            saw_north_left_green = true;
        }

        REQUIRE(engine.getControlMode() == SimulatorEngine::ControlMode::Basic);
    }

    REQUIRE(saw_south_right_demand);
    REQUIRE(saw_north_left_demand);
    REQUIRE(saw_conflicting_right_paused);
    REQUIRE(saw_north_left_green);
    REQUIRE(engine.getMetrics().safety_violations == 0);
}

TEST_CASE("SimulatorEngine rejects red to orange transition in non-null control", "[engine][transition]")
{
    class RedToOrangeController : public ITrafficLightController
    {
    public:
        void tick(double) override
        {
            switched = true;
        }

        IntersectionState getCurrentState() const override
        {
            IntersectionState s{};
            s.north = switched ? LightState::Orange : LightState::Red;
            return s;
        }

        void reset() override
        {
            switched = false;
        }

    private:
        bool switched = false;
    };

    SimulatorEngine engine(0.5, 10.0, 10.0);
    engine.setController(std::make_unique<RedToOrangeController>(), SimulatorEngine::ControlMode::Basic);

    engine.start();
    engine.tick(0.1);

    REQUIRE(engine.getControlMode() == SimulatorEngine::ControlMode::NullControl);
    REQUIRE(engine.getMetrics().safety_violations >= 1);
}

TEST_CASE("SafetyChecker rejects signal group with conflicting movements", "[safety][config][signal-groups]")
{
    IntersectionConfig config = makeDefaultIntersectionConfig();
    config.signal_groups = {
        {301,
         "conflicting-straight",
         {laneIdFor(ApproachId::North, 0), laneIdFor(ApproachId::East, 0)},
         {MovementType::Straight},
         10.0,
         2.0}};

    SafetyChecker checker(config);
    REQUIRE_FALSE(checker.isConfigValid());
}

TEST_CASE("ConfigurableSignalGroupController cycles green and orange per group", "[controller][config]")
{
    IntersectionConfig config = makeDefaultIntersectionConfig();
    config.signal_groups = {
        {1, "North straight", {laneIdFor(ApproachId::North, 0)}, {MovementType::Straight}, 1.0, 0.5},
        {2, "East right", {laneIdFor(ApproachId::East, 2)}, {MovementType::Right}, 1.0, 0.5}};

    ConfigurableSignalGroupController ctrl(config);

    auto s = ctrl.getCurrentState();
    REQUIRE(s.north == LightState::Green);
    REQUIRE(s.turnEastNorth == LightState::Red);

    ctrl.tick(1.0);
    s = ctrl.getCurrentState();
    REQUIRE(s.north == LightState::Orange);

    ctrl.tick(0.5);
    s = ctrl.getCurrentState();
    REQUIRE(s.north == LightState::Red);
    REQUIRE(s.turnEastNorth == LightState::Green);
}

TEST_CASE("SimulatorEngine uses configurable controller for signal-group config", "[engine][config][controller]")
{
    IntersectionConfig config = makeDefaultIntersectionConfig();
    config.signal_groups = {
        {9, "North straight", {laneIdFor(ApproachId::North, 0)}, {MovementType::Straight}, 1.0, 0.5},
        {10, "South straight", {laneIdFor(ApproachId::South, 0)}, {MovementType::Straight}, 1.0, 0.5}};

    SimulatorEngine engine(config, 0.1, 10.0, 10.0);
    engine.start();

    auto s = engine.getCurrentLightState();
    REQUIRE(s.north == LightState::Green);
    REQUIRE(s.south == LightState::Red);

    engine.tick(1.1);
    s = engine.getCurrentLightState();
    REQUIRE(s.north == LightState::Orange);
}

TEST_CASE("TrafficGenerator assigns movement intent from lane config", "[traffic][config][movement]")
{
    IntersectionConfig config = makeDefaultIntersectionConfig();
    config.approaches[0].lanes = {
        {100, "N-mixed", {MovementType::Straight, MovementType::Right}, true},
        {101, "N-left", {MovementType::Left}, true}};

    TrafficGenerator gen(config, 2.0);
    gen.generateTraffic(1.0, 0.0);

    const auto &north_queue = gen.getQueueByDirection(Direction::North);
    REQUIRE_FALSE(north_queue.empty());

    for (const auto &vehicle : north_queue)
    {
        REQUIRE((vehicle.movement == MovementType::Straight ||
                 vehicle.movement == MovementType::Right ||
                 vehicle.movement == MovementType::Left));
        REQUIRE((vehicle.turning == (vehicle.movement != MovementType::Straight)));
    }
}

TEST_CASE("TrafficGenerator propagates lane config flags to vehicles", "[traffic][config][lane]")
{
    IntersectionConfig config = makeDefaultIntersectionConfig();
    config.approaches[1].lanes = {
        {200, "E-fixed", {MovementType::Straight}, false}};

    TrafficGenerator gen(config, 2.0);
    gen.generateTraffic(1.0, 0.0);

    const auto &east_queue = gen.getQueueByDirection(Direction::East);
    REQUIRE_FALSE(east_queue.empty());
    for (const auto &vehicle : east_queue)
    {
        REQUIRE(vehicle.lane_id == 200);
        REQUIRE_FALSE(vehicle.lane_change_allowed);
        REQUIRE(vehicle.movement == MovementType::Straight);
    }
}

TEST_CASE("TrafficGenerator skips disconnected lanes for configured spawns", "[traffic][config][connectivity]")
{
    IntersectionConfig config = makeDefaultIntersectionConfig();
    config.approaches[0].lanes = {
        {700, "N-disconnected", {MovementType::Straight}, true, false, true},
        {701, "N-connected", {MovementType::Straight}, true, true, true}};

    TrafficGenerator gen(config, 2.0);
    gen.generateTraffic(1.0, 0.0);

    const auto &north_queue = gen.getQueueByDirection(Direction::North);
    REQUIRE_FALSE(north_queue.empty());
    for (const auto &vehicle : north_queue)
    {
        REQUIRE(vehicle.lane_id == 701);
    }
}

TEST_CASE("TrafficGenerator changes to preferred right lane early", "[traffic][lane-change]")
{
    IntersectionConfig config = makeDefaultIntersectionConfig();
    config.approaches[0].lanes = {
        {300, "N-left", {MovementType::Left}, true},
        {301, "N-straight", {MovementType::Straight}, true},
        {302, "N-right", {MovementType::Right}, true}};

    TrafficGenerator gen(config, 1.0);
    auto &north = gen.getQueueByDirection(Direction::North);

    Vehicle v1(1, Direction::North, 0.0);
    v1.lane_id = 300;
    v1.queue_index = 0;
    v1.movement = MovementType::Right;
    v1.turning = true;
    v1.lane_change_allowed = true;
    v1.position_in_lane = 10.0;
    north.push_back(v1);

    std::array<bool, 4> can_move = {false, false, false, false};
    gen.updateVehicleSpeeds(0.1, can_move);

    REQUIRE(north.front().lane_id == 302);
    REQUIRE(north.front().movement == MovementType::Right);
    REQUIRE(north.front().turning == true);
}

TEST_CASE("TrafficGenerator falls back to straight when lane change unavailable", "[traffic][lane-change]")
{
    IntersectionConfig config = makeDefaultIntersectionConfig();
    config.approaches[0].lanes = {
        {400, "N-left", {MovementType::Left}, false},
        {401, "N-straight", {MovementType::Straight}, false}};

    TrafficGenerator gen(config, 1.0);
    auto &north = gen.getQueueByDirection(Direction::North);

    Vehicle v1(2, Direction::North, 0.0);
    v1.lane_id = 400;
    v1.queue_index = 0;
    v1.movement = MovementType::Right;
    v1.turning = true;
    v1.lane_change_allowed = false;
    v1.position_in_lane = 15.0;
    north.push_back(v1);

    std::array<bool, 4> can_move = {false, false, false, false};
    gen.updateVehicleSpeeds(0.1, can_move);

    REQUIRE(north.front().movement == MovementType::Straight);
    REQUIRE(north.front().turning == false);
}

TEST_CASE("TrafficGenerator resolves vehicle destination from lane connections", "[traffic][config][routing]")
{
    IntersectionConfig config = makeDefaultIntersectionConfig();
    config.approaches[0].lanes = {
        {500, "N-only-right", {MovementType::Right}, true}};
    config.approaches[1].lanes = {
        {600, "E-0", {MovementType::Straight}, true},
        {601, "E-1", {MovementType::Straight}, true}};
    config.lane_connections.clear();
    config.lane_connections.push_back({ApproachId::North, 0, MovementType::Right, ApproachId::East, 1});

    TrafficGenerator gen(config, 2.0);
    gen.generateTraffic(1.0, 0.0);

    const auto &north = gen.getQueueByDirection(Direction::North);
    REQUIRE_FALSE(north.empty());
    for (const auto &vehicle : north)
    {
        REQUIRE(vehicle.movement == MovementType::Right);
        REQUIRE(vehicle.destination_approach == ApproachId::East);
        REQUIRE(vehicle.destination_lane_index == 1);
        REQUIRE(vehicle.destination_lane_id == laneIdFor(ApproachId::East, 1));
    }
}

TEST_CASE("Mixed north lane 0 keeps straight+left intents from config", "[traffic][config][movement][regression]")
{
    IntersectionConfig config = makeDefaultIntersectionConfig();
    config.approaches[0].lanes = {
        {800, "N-mixed-0", {MovementType::Straight, MovementType::Left}, true, true, true}};
    config.lane_connections.clear();
    config.lane_connections.push_back({ApproachId::North, 0, MovementType::Straight, ApproachId::South, 0});
    config.lane_connections.push_back({ApproachId::North, 0, MovementType::Left, ApproachId::East, 0});

    TrafficGenerator gen(config, 1.0);
    bool seen_straight = false;
    bool seen_left = false;

    for (int i = 0; i < 40; ++i)
    {
        const double current_time = static_cast<double>(i) * 0.25;
        gen.generateTraffic(0.25, current_time);

        while (auto *next = gen.peekNextVehicle(Direction::North))
        {
            REQUIRE(next->lane_id == 800);
            REQUIRE((next->movement == MovementType::Straight || next->movement == MovementType::Left));

            if (next->movement == MovementType::Straight)
                seen_straight = true;
            if (next->movement == MovementType::Left)
                seen_left = true;

            const uint32_t id = next->id;
            REQUIRE(gen.startCrossing(Direction::North, id, current_time));
            REQUIRE(gen.completeCrossing(id, current_time + 1.0));
        }
    }

    REQUIRE(seen_straight);
    REQUIRE(seen_left);
}