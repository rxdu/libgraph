# Real-World Examples and Use Cases

This document provides comprehensive real-world examples demonstrating how to apply libgraph in various domains and industries.

## Table of Contents

- [Game Development](#game-development)
- [Robotics and Motion Planning](#robotics-and-motion-planning)
- [GPS Navigation Systems](#gps-navigation-systems)
- [Network Analysis](#network-analysis)
- [Supply Chain Optimization](#supply-chain-optimization)
- [Social Network Analysis](#social-network-analysis)
- [Transportation Planning](#transportation-planning)
- [Resource Allocation](#resource-allocation)
- [Workflow Management](#workflow-management)
- [Financial Networks](#financial-networks)

## Game Development

### 1. NPC Pathfinding in 3D Environments

```cpp
#include "graph/graph.hpp"
#include "graph/search/astar.hpp"

struct GamePosition {
    float x, y, z;
    TerrainType terrain;
    
    int64_t GetId() const {
        // Discretize position for graph representation
        int64_t ix = static_cast<int64_t>(x * 10);  // 0.1 unit resolution
        int64_t iy = static_cast<int64_t>(y * 10);
        int64_t iz = static_cast<int64_t>(z * 10);
        return (iz << 40) | (iy << 20) | ix;
    }
    
    bool operator==(const GamePosition& other) const {
        return std::abs(x - other.x) < 0.05f && 
               std::abs(y - other.y) < 0.05f && 
               std::abs(z - other.z) < 0.05f;
    }
};

struct MovementCost {
    float time_seconds;
    float energy_cost;
    float stealth_penalty;
    
    bool operator<(const MovementCost& other) const {
        // Prioritize time, then energy, then stealth
        if (time_seconds != other.time_seconds) 
            return time_seconds < other.time_seconds;
        if (energy_cost != other.energy_cost) 
            return energy_cost < other.energy_cost;
        return stealth_penalty < other.stealth_penalty;
    }
    
    MovementCost operator+(const MovementCost& other) const {
        return {time_seconds + other.time_seconds,
                energy_cost + other.energy_cost,
                std::max(stealth_penalty, other.stealth_penalty)};  // Worst stealth
    }
};

namespace xmotion {
    template<>
    struct CostTraits<MovementCost> {
        static MovementCost infinity() {
            return {std::numeric_limits<float>::max(),
                   std::numeric_limits<float>::max(),
                   std::numeric_limits<float>::max()};
        }
    };
}

class NPCPathfinder {
private:
    Graph<GamePosition, MovementCost> world_graph_;
    SearchContext<GamePosition> context_;
    
public:
    NPCPathfinder() {
        context_.PreAllocate(50000);  // Pre-allocate for large game world
    }
    
    std::vector<GamePosition> FindPath(const GamePosition& start, 
                                     const GamePosition& goal,
                                     NPCType npc_type) {
        
        // Define heuristic based on NPC capabilities
        auto heuristic = [npc_type](const GamePosition& from, const GamePosition& to) -> MovementCost {
            float distance = std::sqrt(
                std::pow(from.x - to.x, 2) + 
                std::pow(from.y - to.y, 2) + 
                std::pow(from.z - to.z, 2)
            );
            
            float base_speed = GetNPCSpeed(npc_type);
            float terrain_modifier = GetTerrainModifier(to.terrain, npc_type);
            
            return {distance / (base_speed * terrain_modifier), 
                   distance * GetEnergyMultiplier(npc_type),
                   0.0f};  // No stealth penalty in heuristic
        };
        
        context_.Reset();
        return AStar::Search(world_graph_, context_, start, goal, heuristic);
    }
    
    void BuildWorldGraph(const GameWorld& world) {
        // Build navigation mesh-based graph
        for (const auto& navmesh_triangle : world.GetNavMesh()) {
            GamePosition center = navmesh_triangle.GetCenter();
            world_graph_.AddVertex(center);
            
            // Connect to adjacent triangles
            for (const auto& adjacent : navmesh_triangle.GetAdjacent()) {
                GamePosition adj_center = adjacent.GetCenter();
                
                MovementCost cost = CalculateMovementCost(center, adj_center);
                world_graph_.AddEdge(center, adj_center, cost);
            }
        }
    }
    
private:
    MovementCost CalculateMovementCost(const GamePosition& from, const GamePosition& to) {
        float distance = EuclideanDistance(from, to);
        float height_diff = std::abs(to.z - from.z);
        
        // Base movement time
        float time = distance / 5.0f;  // Base speed 5 units/second
        
        // Terrain penalties
        float terrain_penalty = 1.0f;
        if (to.terrain == TerrainType::WATER) terrain_penalty = 2.0f;
        else if (to.terrain == TerrainType::ROUGH) terrain_penalty = 1.5f;
        
        // Climbing penalty
        if (height_diff > 0.5f) {
            terrain_penalty *= (1.0f + height_diff);
        }
        
        return {time * terrain_penalty, 
               distance * terrain_penalty, 
               GetStealthPenalty(to.terrain)};
    }
};
```

### 2. Dynamic Quest System

```cpp
struct QuestNode {
    std::string quest_id;
    QuestType type;
    std::vector<std::string> prerequisites;
    int level_requirement;
    float estimated_time_hours;
    
    int64_t GetId() const {
        return std::hash<std::string>{}(quest_id);
    }
};

class QuestManager {
private:
    Graph<QuestNode> quest_graph_;
    
public:
    std::vector<QuestNode> FindOptimalQuestPath(const PlayerState& player,
                                               const QuestNode& target_quest) {
        
        // Create temporary start node representing player's current state
        QuestNode player_state{
            "PLAYER_CURRENT", 
            QuestType::NONE, 
            {}, 
            player.level, 
            0.0f
        };
        
        // Build dynamic graph including only achievable quests
        Graph<QuestNode> dynamic_graph = BuildAchievableGraph(player);
        dynamic_graph.AddVertex(player_state);
        
        // Connect player state to all immediately available quests
        for (const auto& quest : GetImmediatelyAvailable(player)) {
            dynamic_graph.AddEdge(player_state, quest, quest.estimated_time_hours);
        }
        
        SearchContext<QuestNode> context;
        return Dijkstra::Search(dynamic_graph, context, player_state, target_quest);
    }
    
private:
    Graph<QuestNode> BuildAchievableGraph(const PlayerState& player) {
        Graph<QuestNode> graph;
        
        // Add all potentially achievable quests
        for (const auto& quest : all_quests_) {
            if (quest.level_requirement <= player.level + 5) {  // Within reasonable level range
                graph.AddVertex(quest);
            }
        }
        
        // Connect quests based on prerequisites and logical progression
        for (const auto& quest : graph.GetVertices()) {
            for (const auto& other_quest : graph.GetVertices()) {
                if (CanProgressTo(quest.GetState(), other_quest.GetState())) {
                    float cost = CalculateQuestTransitionCost(quest.GetState(), 
                                                            other_quest.GetState());
                    graph.AddEdge(quest.GetState(), other_quest.GetState(), cost);
                }
            }
        }
        
        return graph;
    }
};
```

## Robotics and Motion Planning

### 1. Industrial Robot Path Planning

```cpp
struct RobotJointState {
    std::array<double, 6> joint_angles;  // 6-DOF robot arm
    double timestamp;
    
    int64_t GetId() const {
        // Discretize joint angles for graph representation
        int64_t id = 0;
        for (size_t i = 0; i < 6; ++i) {
            int64_t discretized = static_cast<int64_t>(joint_angles[i] * 1000);  // 0.001 rad resolution
            id = id * 1000000 + (discretized + 500000);  // Offset for negative angles
        }
        return id;
    }
    
    bool IsCollisionFree() const {
        // Check collision with workspace obstacles
        CartesianPose end_effector = ForwardKinematics(*this);
        return collision_checker_.IsValid(end_effector);
    }
};

struct MotionCost {
    double time_seconds;
    double energy_joules;
    double smoothness_penalty;  // Penalize jerky movements
    
    bool operator<(const MotionCost& other) const {
        // Prioritize safety (smoothness), then time, then energy
        if (smoothness_penalty != other.smoothness_penalty)
            return smoothness_penalty < other.smoothness_penalty;
        if (time_seconds != other.time_seconds)
            return time_seconds < other.time_seconds;
        return energy_joules < other.energy_joules;
    }
    
    MotionCost operator+(const MotionCost& other) const {
        return {time_seconds + other.time_seconds,
               energy_joules + other.energy_joules,
               smoothness_penalty + other.smoothness_penalty};
    }
};

class RobotMotionPlanner {
private:
    Graph<RobotJointState, MotionCost> configuration_space_;
    CollisionChecker collision_checker_;
    
public:
    std::vector<RobotJointState> PlanMotion(const RobotJointState& start,
                                          const RobotJointState& goal) {
        
        // Build configuration space graph using RRT-Connect approach
        BuildConfigurationSpace(start, goal);
        
        // Use A* with joint-space heuristic
        auto heuristic = [](const RobotJointState& from, const RobotJointState& to) -> MotionCost {
            double total_angular_distance = 0;
            for (size_t i = 0; i < 6; ++i) {
                total_angular_distance += std::abs(from.joint_angles[i] - to.joint_angles[i]);
            }
            
            // Estimate time based on maximum joint velocity
            double max_joint_velocity = 2.0;  // rad/s
            double estimated_time = total_angular_distance / max_joint_velocity;
            
            return {estimated_time, estimated_time * 100.0, 0.0};  // No smoothness in heuristic
        };
        
        SearchContext<RobotJointState> context;
        auto path = AStar::Search(configuration_space_, context, start, goal, heuristic);
        
        return SmoothPath(path);  // Post-process for smoother motion
    }
    
private:
    void BuildConfigurationSpace(const RobotJointState& start, const RobotJointState& goal) {
        // Implement RRT-Connect for high-dimensional configuration space
        std::vector<RobotJointState> samples;
        samples.push_back(start);
        samples.push_back(goal);
        
        // Generate collision-free samples
        for (int i = 0; i < 1000; ++i) {
            RobotJointState sample = GenerateRandomState();
            if (sample.IsCollisionFree()) {
                samples.push_back(sample);
            }
        }
        
        // Add samples to graph
        for (const auto& sample : samples) {
            configuration_space_.AddVertex(sample);
        }
        
        // Connect nearby collision-free configurations
        for (size_t i = 0; i < samples.size(); ++i) {
            for (size_t j = i + 1; j < samples.size(); ++j) {
                if (IsLocallyConnectable(samples[i], samples[j])) {
                    MotionCost cost = CalculateMotionCost(samples[i], samples[j]);
                    configuration_space_.AddEdge(samples[i], samples[j], cost);
                    configuration_space_.AddEdge(samples[j], samples[i], cost);
                }
            }
        }
    }
    
    MotionCost CalculateMotionCost(const RobotJointState& from, const RobotJointState& to) {
        // Calculate actual motion cost considering dynamics
        double max_angular_velocity = 0;
        double total_angular_distance = 0;
        
        for (size_t i = 0; i < 6; ++i) {
            double angular_diff = std::abs(to.joint_angles[i] - from.joint_angles[i]);
            max_angular_velocity = std::max(max_angular_velocity, angular_diff);
            total_angular_distance += angular_diff;
        }
        
        // Time limited by slowest joint
        double time = max_angular_velocity / GetMaxJointVelocity();
        
        // Energy proportional to total movement
        double energy = total_angular_distance * GetAverageJointInertia();
        
        // Smoothness penalty for large accelerations
        double smoothness = CalculateSmoothnessReplaced(from, to);
        
        return {time, energy, smoothness};
    }
};
```

### 2. Multi-Robot Coordination

```cpp
struct MultiRobotState {
    std::vector<RobotPose> robot_positions;
    double timestamp;
    
    int64_t GetId() const {
        // Combine all robot positions into single ID
        std::hash<std::string> hasher;
        std::string state_string;
        
        for (const auto& pose : robot_positions) {
            state_string += std::to_string(pose.x) + "," + 
                           std::to_string(pose.y) + "," + 
                           std::to_string(pose.theta) + ";";
        }
        
        return static_cast<int64_t>(hasher(state_string));
    }
    
    bool IsValidConfiguration() const {
        // Check inter-robot collisions and workspace constraints
        for (size_t i = 0; i < robot_positions.size(); ++i) {
            for (size_t j = i + 1; j < robot_positions.size(); ++j) {
                if (RobotsCollide(robot_positions[i], robot_positions[j])) {
                    return false;
                }
            }
        }
        return true;
    }
};

class MultiRobotPlanner {
private:
    Graph<MultiRobotState, double> composite_state_space_;
    size_t num_robots_;
    
public:
    std::vector<MultiRobotState> PlanCoordinatedMotion(
        const MultiRobotState& start,
        const MultiRobotState& goal) {
        
        // Build composite state space (exponentially complex!)
        BuildCompositeStateSpace(start, goal);
        
        // Use prioritized planning for scalability
        return PrioritizedPlanning(start, goal);
    }
    
private:
    std::vector<MultiRobotState> PrioritizedPlanning(
        const MultiRobotState& start,
        const MultiRobotState& goal) {
        
        std::vector<std::vector<RobotPose>> individual_paths(num_robots_);
        
        // Plan for each robot in priority order
        for (size_t robot_id = 0; robot_id < num_robots_; ++robot_id) {
            Graph<RobotPose> single_robot_graph = BuildSingleRobotGraph(robot_id);
            
            // Add temporal constraints from previously planned robots
            ApplyTemporalConstraints(single_robot_graph, individual_paths, robot_id);
            
            SearchContext<RobotPose> context;
            individual_paths[robot_id] = Dijkstra::Search(
                single_robot_graph, 
                context,
                start.robot_positions[robot_id],
                goal.robot_positions[robot_id]
            );
        }
        
        // Combine individual paths into coordinated trajectory
        return CombinePaths(individual_paths);
    }
    
    void ApplyTemporalConstraints(Graph<RobotPose>& graph,
                                const std::vector<std::vector<RobotPose>>& existing_paths,
                                size_t current_robot) {
        
        // Remove states that would cause collisions with already-planned robots
        std::vector<RobotPose> states_to_remove;
        
        for (const auto& vertex : graph.GetVertices()) {
            RobotPose current_pose = vertex.GetState();
            
            // Check collision with all previously planned robots at all times
            for (size_t other_robot = 0; other_robot < current_robot; ++other_robot) {
                for (size_t time_step = 0; time_step < existing_paths[other_robot].size(); ++time_step) {
                    if (RobotsCollide(current_pose, existing_paths[other_robot][time_step])) {
                        states_to_remove.push_back(current_pose);
                        break;
                    }
                }
            }
        }
        
        // Remove collision states
        for (const auto& state : states_to_remove) {
            graph.RemoveVertex(state);
        }
    }
};
```

## GPS Navigation Systems

### 1. Multi-Modal Transportation Planning

```cpp
struct TransportationNode {
    double latitude, longitude;
    TransportMode mode;  // WALKING, DRIVING, PUBLIC_TRANSIT, CYCLING
    std::string node_id;
    
    int64_t GetId() const {
        return std::hash<std::string>{}(node_id);
    }
};

struct TravelCost {
    double time_minutes;
    double monetary_cost;
    double environmental_impact;  // CO2 equivalent
    double comfort_level;         // 1.0 = most comfortable
    
    bool operator<(const TravelCost& other) const {
        // User-customizable priority weights
        double this_score = time_minutes * time_weight + 
                           monetary_cost * cost_weight +
                           environmental_impact * env_weight +
                           (10.0 - comfort_level) * comfort_weight;
                           
        double other_score = other.time_minutes * time_weight + 
                            other.monetary_cost * cost_weight +
                            other.environmental_impact * env_weight +
                            (10.0 - other.comfort_level) * comfort_weight;
                            
        return this_score < other_score;
    }
    
    TravelCost operator+(const TravelCost& other) const {
        return {time_minutes + other.time_minutes,
               monetary_cost + other.monetary_cost,
               environmental_impact + other.environmental_impact,
               std::min(comfort_level, other.comfort_level)};
    }
    
    static double time_weight, cost_weight, env_weight, comfort_weight;
};

class MultiModalNavigator {
private:
    Graph<TransportationNode, TravelCost> transportation_network_;
    TrafficService traffic_service_;
    TransitService transit_service_;
    
public:
    std::vector<TransportationNode> FindOptimalRoute(
        const TransportationNode& origin,
        const TransportationNode& destination,
        const UserPreferences& preferences) {
        
        // Update weights based on user preferences
        TravelCost::time_weight = preferences.time_priority;
        TravelCost::cost_weight = preferences.cost_priority;
        TravelCost::env_weight = preferences.environmental_priority;
        TravelCost::comfort_weight = preferences.comfort_priority;
        
        // Update graph with real-time information
        UpdateWithRealTimeData();
        
        // Use A* with multi-criteria heuristic
        auto heuristic = [&preferences](const TransportationNode& from, 
                                       const TransportationNode& to) -> TravelCost {
            double distance_km = HaversineDistance(from.latitude, from.longitude,
                                                 to.latitude, to.longitude);
            
            // Estimate based on fastest transportation mode available
            double driving_time = distance_km / 50.0 * 60;  // 50 km/h average, convert to minutes
            double transit_time = distance_km / 30.0 * 60;  // 30 km/h average for transit
            double walking_time = distance_km / 5.0 * 60;   // 5 km/h walking speed
            
            double min_time = std::min({driving_time, transit_time, walking_time});
            
            return {min_time, 0.0, 0.0, 5.0};  // Optimistic estimate
        };
        
        SearchContext<TransportationNode> context;
        return AStar::Search(transportation_network_, context, origin, destination, heuristic);
    }
    
private:
    void UpdateWithRealTimeData() {
        // Update edge costs with current traffic conditions
        for (const auto& vertex : transportation_network_.GetVertices()) {
            TransportationNode current = vertex.GetState();
            
            for (const auto& edge : vertex.GetEdges()) {
                TransportationNode next = edge.GetDst()->GetState();
                
                // Get real-time cost based on transportation mode
                TravelCost updated_cost = edge.GetCost();
                
                if (current.mode == TransportMode::DRIVING) {
                    // Update with traffic data
                    double traffic_factor = traffic_service_.GetTrafficFactor(current, next);
                    updated_cost.time_minutes *= traffic_factor;
                }
                else if (current.mode == TransportMode::PUBLIC_TRANSIT) {
                    // Update with transit delays
                    auto delays = transit_service_.GetDelays(current, next);
                    updated_cost.time_minutes += delays.average_delay_minutes;
                }
                
                // Update edge in graph (simplified - actual implementation would need edge modification)
                // This demonstrates the concept
            }
        }
    }
};
```

### 2. Dynamic Route Optimization with Traffic

```cpp
class DynamicNavigationSystem {
private:
    Graph<GeoPoint, TrafficAwareCost> road_network_;
    TrafficPredictor traffic_predictor_;
    
public:
    std::vector<GeoPoint> FindOptimalRouteWithTraffic(
        const GeoPoint& start,
        const GeoPoint& destination,
        const TimePoint& departure_time) {
        
        // Build time-expanded graph for traffic prediction
        Graph<TimedGeoPoint, TrafficAwareCost> time_expanded_graph = 
            BuildTimeExpandedGraph(departure_time);
        
        TimedGeoPoint timed_start{start, departure_time};
        TimedGeoPoint timed_destination{destination, departure_time + std::chrono::hours(3)};  // Max 3 hour journey
        
        auto heuristic = [](const TimedGeoPoint& from, const TimedGeoPoint& to) -> TrafficAwareCost {
            double distance = HaversineDistance(from.location.latitude, from.location.longitude,
                                              to.location.latitude, to.location.longitude);
            
            // Optimistic travel time at free-flow speed
            double free_flow_time = distance / 60.0;  // 60 km/h average
            
            return {free_flow_time, 0.0, 0.0};
        };
        
        SearchContext<TimedGeoPoint> context;
        auto timed_path = AStar::Search(time_expanded_graph, context, timed_start, timed_destination, heuristic);
        
        // Extract geographical path
        std::vector<GeoPoint> result;
        for (const auto& timed_point : timed_path) {
            result.push_back(timed_point.location);
        }
        
        return result;
    }
    
    // Re-route dynamically based on updated traffic conditions
    std::vector<GeoPoint> DynamicReRoute(const std::vector<GeoPoint>& current_path,
                                       const GeoPoint& current_position,
                                       size_t current_waypoint_index) {
        
        // Check if significant traffic changes have occurred
        bool should_reroute = false;
        for (size_t i = current_waypoint_index; i < current_path.size() - 1; ++i) {
            auto current_cost = GetCurrentTravelCost(current_path[i], current_path[i + 1]);
            auto predicted_cost = GetPredictedCost(current_path[i], current_path[i + 1]);
            
            if (current_cost.travel_time > predicted_cost.travel_time * 1.5) {
                should_reroute = true;
                break;
            }
        }
        
        if (should_reroute) {
            return FindOptimalRouteWithTraffic(current_position, 
                                             current_path.back(), 
                                             std::chrono::system_clock::now());
        }
        
        return current_path;  // Keep existing route
    }
    
private:
    Graph<TimedGeoPoint, TrafficAwareCost> BuildTimeExpandedGraph(const TimePoint& start_time) {
        Graph<TimedGeoPoint, TrafficAwareCost> graph;
        
        // Create time layers (e.g., every 5 minutes for next 3 hours)
        const auto time_resolution = std::chrono::minutes(5);
        const auto max_duration = std::chrono::hours(3);
        
        for (auto time = start_time; time < start_time + max_duration; time += time_resolution) {
            // Add all geographical points at this time
            for (const auto& vertex : road_network_.GetVertices()) {
                GeoPoint geo_point = vertex.GetState();
                TimedGeoPoint timed_point{geo_point, time};
                graph.AddVertex(timed_point);
            }
        }
        
        // Connect time layers with predicted travel costs
        ConnectTimeLayers(graph, start_time, time_resolution);
        
        return graph;
    }
};
```

## Network Analysis

### 1. Computer Network Routing

```cpp
struct NetworkNode {
    std::string ip_address;
    NodeType type;  // ROUTER, SWITCH, HOST
    double cpu_load;
    double available_bandwidth_mbps;
    
    int64_t GetId() const {
        return std::hash<std::string>{}(ip_address);
    }
};

struct NetworkCost {
    double latency_ms;
    double bandwidth_utilization;  // 0.0 to 1.0
    double reliability_score;      // 0.0 to 1.0, higher is better
    int hop_count;
    
    bool operator<(const NetworkCost& other) const {
        // Multi-objective optimization for network routing
        double this_score = latency_ms * 0.4 + 
                           bandwidth_utilization * 100 * 0.3 +
                           (1.0 - reliability_score) * 100 * 0.2 +
                           hop_count * 10 * 0.1;
                           
        double other_score = other.latency_ms * 0.4 + 
                            other.bandwidth_utilization * 100 * 0.3 +
                            (1.0 - other.reliability_score) * 100 * 0.2 +
                            other.hop_count * 10 * 0.1;
                            
        return this_score < other_score;
    }
    
    NetworkCost operator+(const NetworkCost& other) const {
        return {latency_ms + other.latency_ms,
               std::max(bandwidth_utilization, other.bandwidth_utilization),  // Bottleneck
               std::min(reliability_score, other.reliability_score),          // Weakest link
               hop_count + other.hop_count};
    }
};

class NetworkRoutingEngine {
private:
    Graph<NetworkNode, NetworkCost> network_topology_;
    NetworkMonitor monitor_;
    
public:
    std::vector<NetworkNode> FindOptimalRoute(const NetworkNode& source,
                                            const NetworkNode& destination,
                                            const QoSRequirements& qos) {
        
        // Update network state with current measurements
        UpdateNetworkState();
        
        // Apply QoS constraints by removing unsuitable edges
        Graph<NetworkNode, NetworkCost> constrained_graph = 
            ApplyQoSConstraints(network_topology_, qos);
        
        SearchContext<NetworkNode> context;
        return Dijkstra::Search(constrained_graph, context, source, destination);
    }
    
    // Find multiple disjoint paths for fault tolerance
    std::vector<std::vector<NetworkNode>> FindDisjointPaths(
        const NetworkNode& source,
        const NetworkNode& destination,
        size_t num_paths) {
        
        std::vector<std::vector<NetworkNode>> paths;
        Graph<NetworkNode, NetworkCost> working_graph = network_topology_;
        
        for (size_t i = 0; i < num_paths; ++i) {
            SearchContext<NetworkNode> context;
            auto path = Dijkstra::Search(working_graph, context, source, destination);
            
            if (path.empty()) break;  // No more paths available
            
            paths.push_back(path);
            
            // Remove edges used in this path to find disjoint paths
            for (size_t j = 0; j < path.size() - 1; ++j) {
                working_graph.RemoveEdge(path[j], path[j + 1]);
                working_graph.RemoveEdge(path[j + 1], path[j]);  // Bidirectional
            }
        }
        
        return paths;
    }
    
private:
    void UpdateNetworkState() {
        // Update edge costs based on current network conditions
        for (const auto& vertex : network_topology_.GetVertices()) {
            NetworkNode current = vertex.GetState();
            
            for (const auto& edge : vertex.GetEdges()) {
                NetworkNode next = edge.GetDst()->GetState();
                
                // Measure current link properties
                auto link_stats = monitor_.GetLinkStatistics(current.ip_address, next.ip_address);
                
                NetworkCost updated_cost = {
                    link_stats.current_latency_ms,
                    link_stats.bandwidth_utilization,
                    link_stats.reliability_score,
                    1  // Single hop
                };
                
                // Update edge cost (simplified representation)
                // Real implementation would modify the graph structure
            }
        }
    }
    
    Graph<NetworkNode, NetworkCost> ApplyQoSConstraints(
        const Graph<NetworkNode, NetworkCost>& original_graph,
        const QoSRequirements& qos) {
        
        Graph<NetworkNode, NetworkCost> constrained_graph;
        
        // Copy vertices
        for (const auto& vertex : original_graph.GetVertices()) {
            constrained_graph.AddVertex(vertex.GetState());
        }
        
        // Copy edges that meet QoS requirements
        for (const auto& vertex : original_graph.GetVertices()) {
            for (const auto& edge : vertex.GetEdges()) {
                NetworkCost cost = edge.GetCost();
                
                if (cost.latency_ms <= qos.max_latency_ms &&
                    cost.bandwidth_utilization <= qos.max_utilization &&
                    cost.reliability_score >= qos.min_reliability) {
                    
                    constrained_graph.AddEdge(vertex.GetState(), 
                                            edge.GetDst()->GetState(), 
                                            cost);
                }
            }
        }
        
        return constrained_graph;
    }
};
```

### 2. Social Network Analysis

```cpp
struct SocialUser {
    std::string user_id;
    std::string username;
    std::vector<std::string> interests;
    double influence_score;
    
    int64_t GetId() const {
        return std::hash<std::string>{}(user_id);
    }
};

struct SocialConnection {
    double relationship_strength;  // 0.0 to 1.0
    ConnectionType type;           // FRIEND, FOLLOWER, COLLEAGUE, etc.
    double interaction_frequency;  // interactions per day
    
    bool operator<(const SocialConnection& other) const {
        // Stronger connections have lower "cost" for information propagation
        return relationship_strength > other.relationship_strength;
    }
    
    SocialConnection operator+(const SocialConnection& other) const {
        // Path strength is limited by weakest connection
        return {std::min(relationship_strength, other.relationship_strength),
               type,  // Keep first connection type
               std::min(interaction_frequency, other.interaction_frequency)};
    }
};

class SocialNetworkAnalyzer {
private:
    Graph<SocialUser, SocialConnection> social_graph_;
    
public:
    // Find influencers who can best reach a target audience
    std::vector<SocialUser> FindInfluencers(const std::vector<std::string>& target_interests,
                                          size_t max_influencers) {
        
        std::vector<SocialUser> influencers;
        std::set<std::string> covered_users;
        
        // Greedy algorithm to find influencers with maximum reach
        for (size_t i = 0; i < max_influencers; ++i) {
            SocialUser best_influencer = FindBestUncoveredInfluencer(target_interests, covered_users);
            
            if (best_influencer.user_id.empty()) break;  // No more suitable influencers
            
            influencers.push_back(best_influencer);
            
            // Add users reachable by this influencer to covered set
            auto reachable = FindReachableUsers(best_influencer, 3);  // 3 degrees of separation
            for (const auto& user : reachable) {
                covered_users.insert(user.user_id);
            }
        }
        
        return influencers;
    }
    
    // Find shortest path between users (degrees of separation)
    std::vector<SocialUser> FindConnectionPath(const SocialUser& from_user,
                                             const SocialUser& to_user) {
        
        SearchContext<SocialUser> context;
        return BFS::Search(social_graph_, context, from_user, to_user);
    }
    
    // Identify communities using graph clustering
    std::vector<std::vector<SocialUser>> FindCommunities() {
        // Implement Louvain method for community detection
        return LouvainClustering();
    }
    
    // Predict information spread using epidemic models
    double PredictInformationSpread(const SocialUser& seed_user,
                                  const InformationType& info_type,
                                  double time_horizon_days) {
        
        // Use SIR (Susceptible-Infected-Recovered) model
        std::unordered_map<std::string, UserState> user_states;
        
        // Initialize all users as susceptible except seed
        for (const auto& vertex : social_graph_.GetVertices()) {
            SocialUser user = vertex.GetState();
            user_states[user.user_id] = (user == seed_user) ? UserState::INFECTED : UserState::SUSCEPTIBLE;
        }
        
        double infection_rate = GetInfectionRate(info_type);
        double recovery_rate = GetRecoveryRate(info_type);
        double time_step = 0.1;  // days
        
        for (double t = 0; t < time_horizon_days; t += time_step) {
            UpdateEpidemicModel(user_states, infection_rate, recovery_rate, time_step);
        }
        
        // Count total users who were infected
        size_t infected_count = 0;
        for (const auto& [user_id, state] : user_states) {
            if (state == UserState::RECOVERED) {
                infected_count++;
            }
        }
        
        return static_cast<double>(infected_count) / user_states.size();
    }
    
private:
    std::vector<SocialUser> FindReachableUsers(const SocialUser& source, int max_depth) {
        std::vector<SocialUser> reachable;
        std::unordered_set<std::string> visited;
        std::queue<std::pair<SocialUser, int>> queue;
        
        queue.push({source, 0});
        visited.insert(source.user_id);
        
        while (!queue.empty()) {
            auto [current_user, depth] = queue.front();
            queue.pop();
            
            reachable.push_back(current_user);
            
            if (depth < max_depth) {
                auto* vertex = social_graph_.GetVertexPtr(current_user);
                for (const auto& edge : vertex->GetEdges()) {
                    SocialUser neighbor = edge.GetDst()->GetState();
                    
                    if (visited.find(neighbor.user_id) == visited.end()) {
                        visited.insert(neighbor.user_id);
                        queue.push({neighbor, depth + 1});
                    }
                }
            }
        }
        
        return reachable;
    }
};
```

This comprehensive collection of real-world examples demonstrates the versatility and power of libgraph across multiple domains, showing how the same core graph algorithms can be adapted to solve complex real-world problems through appropriate state modeling and cost functions.