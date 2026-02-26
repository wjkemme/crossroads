#pragma once

#include <vector>
#include <deque>
#include <cstdint>
#include <array>
#include "Vehicle.hpp"
#include "Intersection.hpp"
#include "IntersectionConfig.hpp"

// Add these constants after the class declaration begins:
static constexpr double VEHICLE_SPACING = 5.0; // meters
static constexpr size_t LANE_CAPACITY = 10;    // vehicles
namespace crossroads
{

    struct LaneVehicleState
    {
        uint32_t id = 0;
        double position_in_lane = 0.0;
        double speed = 0.0;
        bool crossing = false;
        bool turning = false;
        double crossing_time = -1.0;
        double crossing_duration = 0.0;
        uint8_t queue_index = 0; // 0 or 1 for straight, 2 for turn
        uint16_t lane_id = 0;
        MovementType movement = MovementType::Straight;
        ApproachId destination_approach = ApproachId::North;
        uint16_t destination_lane_index = 0;
        LaneId destination_lane_id = 0;
        bool lane_change_allowed = true;
    };

    class TrafficGenerator
    {
    public:
        TrafficGenerator(double arrival_rate = 0.5); // vehicles per second, per lane
        TrafficGenerator(const IntersectionConfig &config, double arrival_rate = 0.5);

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

        void updateVehicleSpeeds(double dt_seconds, const std::array<bool, 4> &lane_can_move);
        double getAverageQueueDensity(Direction dir) const;
        std::vector<LaneVehicleState> getLaneVehicleStates(Direction dir) const;

        // Get queue reference by direction (for direct iteration)
        std::deque<Vehicle> &getQueueByDirection(Direction dir);
        const std::deque<Vehicle> &getQueueByDirection(Direction dir) const;

    private:
        const ApproachConfig *getApproachConfig(Direction dir) const;
        bool laneAllowsMovement(const LaneConfig &lane, MovementType movement) const;
        const LaneConnectionConfig *findLaneConnection(ApproachId from_approach, uint16_t from_lane_index, MovementType movement) const;
        bool resolveVehicleRoute(Vehicle &vehicle, ApproachId from_approach, uint16_t from_lane_index, MovementType movement) const;
        size_t chooseSpawnMovementIndex(const std::vector<MovementType> &movements, uint32_t vehicle_id) const;
        size_t choosePreferredLaneIndex(const ApproachConfig &approach, MovementType movement, size_t current_index) const;
        void maybeApplyLaneChanges(Direction dir, std::deque<Vehicle> &queue);
        bool hasSafeGapForLaneChange(const std::deque<Vehicle> &queue, size_t vehicle_index, LaneId target_lane_id) const;

        IntersectionConfig intersection_config;
        bool use_configured_spawns = false;
        std::array<size_t, 4> spawn_lane_cursor{};

        double arrival_rate;      // vehicles per second per lane
        double time_accumulated;  // accumulated time for next spawn calculation
        uint32_t next_vehicle_id; // counter for unique IDs

        // Vehicle queues for each direction
        std::deque<Vehicle> north_queue;
        std::deque<Vehicle> east_queue;
        std::deque<Vehicle> south_queue;
        std::deque<Vehicle> west_queue;

        // Vehicles that have completed crossing
        std::vector<Vehicle> crossed_vehicles;

        // Helper to calculate next spawn time using Poisson-like distribution
        double getNextSpawnInterval();
    };

} // namespace crossroads
