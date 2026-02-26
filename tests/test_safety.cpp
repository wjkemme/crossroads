#define CATCH_CONFIG_MAIN
#include <catch2/catch_all.hpp>
#include <memory>
#include "SafetyChecker.hpp"
#include "BasicLightController.hpp"
#include "TrafficGenerator.hpp"
#include "SimulatorEngine.hpp"
#include "TrafficLightControllers.hpp"
#include "IntersectionConfigJson.hpp"

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

    // Test turnSouthEast cannot go green if West is active
    IntersectionState prev;
    prev.north = LightState::Red;
    prev.south = LightState::Red;
    prev.east = LightState::Red;
    prev.west = LightState::Green; // West is active
    prev.turnSouthEast = LightState::Red;

    IntersectionState attemptTurnGreen = prev;
    attemptTurnGreen.turnSouthEast = LightState::Green;
    REQUIRE(c.isValidTransition(prev, attemptTurnGreen, 0.1) == false);

    // Test turnSouthEast CAN go green if West is red
    prev.west = LightState::Red;
    attemptTurnGreen = prev;
    attemptTurnGreen.turnSouthEast = LightState::Green;
    REQUIRE(c.isValidTransition(prev, attemptTurnGreen, 0.1) == true);

    // Test turnNorthWest cannot go green if East is orange
    prev.north = LightState::Red;
    prev.east = LightState::Orange; // East is active
    prev.turnNorthWest = LightState::Red;

    attemptTurnGreen = prev;
    attemptTurnGreen.turnNorthWest = LightState::Green;
    REQUIRE(c.isValidTransition(prev, attemptTurnGreen, 0.1) == false);

    // Test turnWestSouth cannot go green if North is green
    prev.north = LightState::Green; // North is active
    prev.east = LightState::Red;
    prev.west = LightState::Red;
    prev.turnWestSouth = LightState::Red;

    attemptTurnGreen = prev;
    attemptTurnGreen.turnWestSouth = LightState::Green;
    REQUIRE(c.isValidTransition(prev, attemptTurnGreen, 0.1) == false);

    // Test turnEastNorth cannot go green if South is active
    prev.north = LightState::Red;
    prev.south = LightState::Orange; // South is active
    prev.east = LightState::Red;
    prev.turnEastNorth = LightState::Red;

    attemptTurnGreen = prev;
    attemptTurnGreen.turnEastNorth = LightState::Green;
    REQUIRE(c.isValidTransition(prev, attemptTurnGreen, 0.1) == false);

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
    REQUIRE(v.getCrossingDuration(0) == Catch::Approx(2.5));
    REQUIRE(v.getCrossingDuration(5) == Catch::Approx(3.5));
    REQUIRE(v.getCrossingDuration(10) == Catch::Approx(4.5));
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
    class OpposingNSGreenController : public ITrafficLightController
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
    config.signal_groups = {
        {101, "N-left", {laneIdFor(ApproachId::North, 0)}, {MovementType::Left}, 8.0, 2.0},
        {102, "S-left", {laneIdFor(ApproachId::South, 0)}, {MovementType::Left}, 8.0, 2.0}};

    SimulatorEngine engine(config, 0.5, 10.0, 10.0);
    engine.setController(std::make_unique<OpposingNSGreenController>(), SimulatorEngine::ControlMode::Basic);

    engine.start();
    engine.tick(0.1);

    REQUIRE(engine.getControlMode() == SimulatorEngine::ControlMode::NullControl);
    REQUIRE(engine.getMetrics().safety_violations >= 1);
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