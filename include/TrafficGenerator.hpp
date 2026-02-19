#pragma once

#include <vector>
#include <queue>
#include <cstdint>
#include "Vehicle.hpp"
#include "Intersection.hpp"

namespace crossroads
{

    class TrafficGenerator
    {
    public:
        TrafficGenerator(double arrival_rate = 0.5); // vehicles per second, per lane

        // Generate new vehicles for given time step and add them to appropriate lane queues
        void generateTraffic(double dt_seconds, double current_time);

        // Move a vehicle from lane queue to crossing state
        bool startCrossing(Direction lane, uint32_t vehicle_id, double current_time);

        // Mark a vehicle as having completed crossing
        bool completeCrossing(uint32_t vehicle_id, double current_time);

        // Get number of waiting vehicles in a lane
        size_t getQueueLength(Direction lane) const;

        // Get total number of vehicles waiting across all lanes
        size_t getTotalWaiting() const;

        // Get the first waiting vehicle in a lane (or nullptr if none)
        Vehicle *peekNextVehicle(Direction lane);

        // Statistics: total vehicles generated
        uint32_t getTotalGenerated() const { return next_vehicle_id - 1; }

        // Statistics: total vehicles that have crossed
        uint32_t getTotalCrossed() const { return crossed_vehicles.size(); }

        // Statistics: average wait time for crossed vehicles
        double getAverageWaitTime() const;

        // Reset all state
        void reset();

    private:
        double arrival_rate;      // vehicles per second per lane
        double time_accumulated;  // accumulated time for next spawn calculation
        uint32_t next_vehicle_id; // counter for unique IDs

        // Vehicle queues for each direction
        std::queue<Vehicle> north_queue;
        std::queue<Vehicle> east_queue;
        std::queue<Vehicle> south_queue;
        std::queue<Vehicle> west_queue;

        // Vehicles that have completed crossing
        std::vector<Vehicle> crossed_vehicles;

        // Helper to get queue reference by direction
        std::queue<Vehicle> &getQueueByDirection(Direction dir);

        // Helper to calculate next spawn time using Poisson-like distribution
        double getNextSpawnInterval();
    };

} // namespace crossroads
