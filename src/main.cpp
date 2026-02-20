#include <iostream>
#include "SimulatorEngine.hpp"

int main()
{
    std::cout << "=== Crossroads Traffic Simulator ===" << std::endl;
    std::cout << std::endl;

    crossroads::SimulatorEngine engine(0.8, 10.0, 10.0);

    std::cout << "Running 30-second simulation..." << std::endl;
    std::cout << std::endl;

    const double time_step = 0.1;
    const double report_interval = 5.0;

    for (double report_time = 0; report_time <= 30.0; report_time += report_interval)
    {
        // Simulate until next report time
        for (int i = 0; i < static_cast<int>(report_interval / time_step); ++i)
        {
            engine.tick(time_step);
        }

        auto metrics = engine.getMetrics();
        std::cout << "Time: " << metrics.total_time << "s" << std::endl;
        std::cout << "  Vehicles generated: " << metrics.vehicles_generated << std::endl;
        std::cout << "  Vehicles crossed: " << metrics.vehicles_crossed << std::endl;
        std::cout << "  Avg wait time: " << metrics.average_wait_time << "s" << std::endl;
        std::cout << "  Queue length: " << static_cast<int>(metrics.queue_length) << std::endl;
        std::cout << "  Safety violations: " << metrics.safety_violations << std::endl;
        std::cout << std::endl;
    }

    std::cout << "Simulation complete!" << std::endl;
    return 0;
}
