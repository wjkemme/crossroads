#include <iostream>
#include <iomanip>
#include "SafetyChecker.hpp"
#include "BasicLightController.hpp"
#include "TrafficGenerator.hpp"

using namespace crossroads;

// Helper to print light state
std::string lightStateStr(LightState s)
{
    switch (s)
    {
    case LightState::Red:
        return "🔴 Red   ";
    case LightState::Orange:
        return "🟠 Orange";
    case LightState::Green:
        return "🟢 Green ";
    }
    return "Unknown";
}

int main()
{
    std::cout << "Crossroads simulator - BasicLightController demo" << std::endl;

    // Create controller with 3-second green phases for testing
    BasicLightController ctrl(3.0, 3.0);
    double elapsed = 0.0;

    std::cout << "\n=== Automatic Light Cycles ===" << std::endl;
    std::cout << std::fixed << std::setprecision(1);

    // Simulate 20 seconds of operation
    for (int step = 0; step < 15; ++step)
    {
        auto state = ctrl.getCurrentState();

        std::cout << "\nTime: " << elapsed << "s" << std::endl;
        std::cout << "  N:" << lightStateStr(state.north) << " | "
                  << "S:" << lightStateStr(state.south) << " | "
                  << "E:" << lightStateStr(state.east) << " | "
                  << "W:" << lightStateStr(state.west) << std::endl;

        SafetyChecker checker;
        if (!checker.isSafe(state))
        {
            std::cout << "  ⚠️  UNSAFE STATE DETECTED!" << std::endl;
        }

        ctrl.tick(0.5);
        elapsed += 0.5;
    }

    // Demo: Turning lights (Afslagindicatoren)
    std::cout << "\n\n=== Turning Light Scenarios ===" << std::endl;
    SafetyChecker checker;

    // Scenario 1: Zuid->Oost (South->East) turn
    std::cout << "\n1. Zuid->Oost turn (only if West is NOT active):" << std::endl;
    IntersectionState allowedTurn1;
    allowedTurn1.north = LightState::Red;
    allowedTurn1.south = LightState::Red;
    allowedTurn1.east = LightState::Red;
    allowedTurn1.west = LightState::Red;
    allowedTurn1.turnSouthEast = LightState::Green;
    std::cout << "   turnSouthEast GREEN, West RED: " << (checker.isSafe(allowedTurn1) ? "✓ ALLOWED" : "✗ BLOCKED") << std::endl;

    IntersectionState blockedTurn1 = allowedTurn1;
    blockedTurn1.west = LightState::Green;
    std::cout << "   turnSouthEast GREEN, West GREEN: " << (checker.isSafe(blockedTurn1) ? "✗ ALLOWED (ERROR)" : "✓ BLOCKED") << std::endl;

    // Scenario 2: Oost->Noord (East->North) turn
    std::cout << "\n2. Oost->Noord turn (only if South is NOT active):" << std::endl;
    IntersectionState allowedTurn2;
    allowedTurn2.north = LightState::Red;
    allowedTurn2.south = LightState::Red;
    allowedTurn2.east = LightState::Red;
    allowedTurn2.west = LightState::Red;
    allowedTurn2.turnEastNorth = LightState::Green;
    std::cout << "   turnEastNorth GREEN, South RED: " << (checker.isSafe(allowedTurn2) ? "✓ ALLOWED" : "✗ BLOCKED") << std::endl;

    IntersectionState blockedTurn2 = allowedTurn2;
    blockedTurn2.south = LightState::Orange;
    std::cout << "   turnEastNorth GREEN, South ORANGE: " << (checker.isSafe(blockedTurn2) ? "✗ ALLOWED (ERROR)" : "✓ BLOCKED") << std::endl;

    // Scenario 3: Noord->West (North->West) turn
    std::cout << "\n3. Noord->West turn (only if East is NOT active):" << std::endl;
    IntersectionState allowedTurn3;
    allowedTurn3.north = LightState::Red;
    allowedTurn3.south = LightState::Red;
    allowedTurn3.east = LightState::Red;
    allowedTurn3.west = LightState::Red;
    allowedTurn3.turnNorthWest = LightState::Green;
    std::cout << "   turnNorthWest GREEN, East RED: " << (checker.isSafe(allowedTurn3) ? "✓ ALLOWED" : "✗ BLOCKED") << std::endl;

    IntersectionState blockedTurn3 = allowedTurn3;
    blockedTurn3.east = LightState::Green;
    std::cout << "   turnNorthWest GREEN, East GREEN: " << (checker.isSafe(blockedTurn3) ? "✗ ALLOWED (ERROR)" : "✓ BLOCKED") << std::endl;

    // Scenario 4: West->Zuid (West->South) turn
    std::cout << "\n4. West->Zuid turn (only if North is NOT active):" << std::endl;
    IntersectionState allowedTurn4;
    allowedTurn4.north = LightState::Red;
    allowedTurn4.south = LightState::Red;
    allowedTurn4.east = LightState::Red;
    allowedTurn4.west = LightState::Red;
    allowedTurn4.turnWestSouth = LightState::Green;
    std::cout << "   turnWestSouth GREEN, North RED: " << (checker.isSafe(allowedTurn4) ? "✓ ALLOWED" : "✗ BLOCKED") << std::endl;

    IntersectionState blockedTurn4 = allowedTurn4;
    blockedTurn4.north = LightState::Green;
    std::cout << "   turnWestSouth GREEN, North GREEN: " << (checker.isSafe(blockedTurn4) ? "✗ ALLOWED (ERROR)" : "✓ BLOCKED") << std::endl;

    // Demo: Traffic Generator
    std::cout << "\n\n=== Traffic Generator Demo ===" << std::endl;
    TrafficGenerator traffic(0.3); // 0.3 vehicles per second per lane

    std::cout << "\nGenerating traffic for 5 seconds..." << std::endl;
    traffic.generateTraffic(5.0, 0.0);

    std::cout << "\nQueue status after generation:" << std::endl;
    std::cout << "  North queue: " << traffic.getQueueLength(Direction::North) << " vehicles" << std::endl;
    std::cout << "  South queue: " << traffic.getQueueLength(Direction::South) << " vehicles" << std::endl;
    std::cout << "  East queue:  " << traffic.getQueueLength(Direction::East) << " vehicles" << std::endl;
    std::cout << "  West queue:  " << traffic.getQueueLength(Direction::West) << " vehicles" << std::endl;
    std::cout << "  Total waiting: " << traffic.getTotalWaiting() << " vehicles" << std::endl;
    std::cout << "  Total generated: " << traffic.getTotalGenerated() << " vehicles" << std::endl;

    // Simulate some vehicles crossing
    std::cout << "\nSimulating vehicle crossings..." << std::endl;
    for (Direction dir : {Direction::North, Direction::South, Direction::East, Direction::West})
    {
        while (true)
        {
            Vehicle *v = traffic.peekNextVehicle(dir);
            if (!v)
                break;

            uint32_t vid = v->id;
            if (traffic.startCrossing(dir, vid, 5.5))
            {
                traffic.completeCrossing(vid, 7.5); // 2 seconds crossing time
            }
            else
            {
                break;
            }
        }
    }

    std::cout << "  Vehicles crossed: " << traffic.getTotalCrossed() << std::endl;
    std::cout << "  Remaining waiting: " << traffic.getTotalWaiting() << std::endl;

    if (traffic.getTotalCrossed() > 0)
    {
        std::cout << "  Average wait time: " << std::fixed << std::setprecision(2)
                  << traffic.getAverageWaitTime() << " seconds" << std::endl;
    }

    return 0;
}
