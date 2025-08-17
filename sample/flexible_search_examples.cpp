/*
 * flexible_search_examples.cpp
 *
 * Examples showing how to use the flexible SearchContext for different algorithms
 */

#include "graph/graph.hpp"
#include "graph/search/search_context.hpp"
#include <iostream>
#include <vector>

struct GridCell {
    int x, y;
    GridCell(int x_, int y_) : x(x_), y(y_) {}
    int64_t GetId() const { return x * 1000 + y; }
};

using Graph = xmotion::Graph<GridCell, double>;
using SearchContext = xmotion::SearchContext<GridCell, double, xmotion::DefaultIndexer<GridCell>>;

void DijkstraExample(Graph& graph, SearchContext& context) {
    std::cout << "\n=== Dijkstra Algorithm (Traditional) ===" << std::endl;
    
    // Traditional usage - works exactly as before
    auto& info = context.GetSearchInfo(1001); // Cell at (1,1)
    info.g_cost = 0.0;
    info.parent_id = -1;
    info.is_checked = true;
    
    std::cout << "Dijkstra - Vertex (1,1) g_cost: " << info.g_cost << std::endl;
}

void AStarExample(Graph& graph, SearchContext& context) {
    std::cout << "\n=== A* Algorithm (Traditional) ===" << std::endl;
    
    // Traditional A* fields still work
    auto& info = context.GetSearchInfo(2002); // Cell at (2,2)
    info.g_cost = 1.414;  // sqrt(2) for diagonal move
    info.h_cost = 2.828;  // heuristic distance
    info.f_cost = info.g_cost + info.h_cost;
    info.parent_id = 1001;
    
    std::cout << "A* - Vertex (2,2) f_cost: " << info.f_cost << std::endl;
}

void DStarLiteExample(Graph& graph, SearchContext& context) {
    std::cout << "\n=== D* Lite Algorithm (Flexible) ===" << std::endl;
    
    int64_t vertex_id = 3003; // Cell at (3,3)
    
    // D* Lite specific attributes
    context.SetVertexAttribute(vertex_id, "rhs", 5.0);
    context.SetVertexAttribute(vertex_id, "g", std::numeric_limits<double>::max());
    context.SetVertexAttribute(vertex_id, "key1", 7.0);
    context.SetVertexAttribute(vertex_id, "key2", 5.0);
    context.SetVertexAttribute(vertex_id, "in_queue", true);
    
    // Store predecessors and successors for dynamic updates
    std::vector<int64_t> predecessors = {2002, 2003, 3002};
    std::vector<int64_t> successors = {3004, 4003, 4004};
    context.SetVertexAttribute(vertex_id, "predecessors", predecessors);
    context.SetVertexAttribute(vertex_id, "successors", successors);
    
    std::cout << "D* Lite - Vertex (3,3) rhs: " << 
        context.GetVertexAttribute<double>(vertex_id, "rhs") << std::endl;
    std::cout << "D* Lite - Vertex (3,3) key1: " << 
        context.GetVertexAttribute<double>(vertex_id, "key1") << std::endl;
    
    auto succ = context.GetVertexAttribute<std::vector<int64_t>>(vertex_id, "successors");
    std::cout << "D* Lite - Successors count: " << succ.size() << std::endl;
}

void JumpPointSearchExample(Graph& graph, SearchContext& context) {
    std::cout << "\n=== Jump Point Search (Flexible) ===" << std::endl;
    
    int64_t vertex_id = 4004; // Cell at (4,4)
    
    // JPS specific attributes
    context.SetVertexAttribute(vertex_id, "is_jump_point", true);
    context.SetVertexAttribute(vertex_id, "jump_direction", std::string("northeast"));
    context.SetVertexAttribute(vertex_id, "parent_direction", std::string("north"));
    context.SetVertexAttribute(vertex_id, "forced_neighbors", 2);
    context.SetVertexAttribute(vertex_id, "pruned", false);
    
    // Store the actual jump distances
    std::vector<int> jump_distances = {3, 5, 2}; // different directions
    context.SetVertexAttribute(vertex_id, "jump_distances", jump_distances);
    
    std::cout << "JPS - Vertex (4,4) is jump point: " << 
        context.GetVertexAttribute<bool>(vertex_id, "is_jump_point") << std::endl;
    std::cout << "JPS - Jump direction: " << 
        context.GetVertexAttribute<std::string>(vertex_id, "jump_direction") << std::endl;
}

void FlowNetworkExample(Graph& graph, SearchContext& context) {
    std::cout << "\n=== Max Flow Algorithm (Flexible) ===" << std::endl;
    
    int64_t vertex_id = 5005; // Cell at (5,5)
    
    // Flow network specific attributes
    context.SetVertexAttribute(vertex_id, "level", 3);           // BFS level
    context.SetVertexAttribute(vertex_id, "excess_flow", 2.5);   // Excess flow
    context.SetVertexAttribute(vertex_id, "current_edge", 1);    // Current edge index
    context.SetVertexAttribute(vertex_id, "active", true);       // Active vertex
    context.SetVertexAttribute(vertex_id, "height", 4);          // Push-relabel height
    
    std::cout << "Flow - Vertex (5,5) level: " << 
        context.GetVertexAttribute<int>(vertex_id, "level") << std::endl;
    std::cout << "Flow - Excess flow: " << 
        context.GetVertexAttribute<double>(vertex_id, "excess_flow") << std::endl;
}

void CustomAlgorithmExample(Graph& graph, SearchContext& context) {
    std::cout << "\n=== Custom Algorithm (Mixed Usage) ===" << std::endl;
    
    int64_t vertex_id = 6006; // Cell at (6,6)
    
    // Mix traditional and flexible approaches
    auto& info = context.GetSearchInfo(vertex_id);
    
    // Use traditional fields
    info.g_cost = 10.0;
    info.parent_id = 5005;
    
    // Add custom attributes for your specific algorithm
    info.SetAttribute("custom_priority", 15.5);
    info.SetAttribute("algorithm_phase", std::string("exploration"));
    info.SetAttribute("visit_count", 3);
    info.SetAttribute("last_update_time", 12345);
    
    // Complex custom data
    struct CustomData {
        double confidence;
        std::string source;
        bool validated;
    };
    CustomData data{0.95, "sensor_A", true};
    info.SetAttribute("sensor_data", data);
    
    std::cout << "Custom - Traditional g_cost: " << info.g_cost << std::endl;
    std::cout << "Custom - Custom priority: " << 
        info.GetAttribute<double>("custom_priority") << std::endl;
    std::cout << "Custom - Algorithm phase: " << 
        info.GetAttribute<std::string>("algorithm_phase") << std::endl;
    
    auto sensor = info.GetAttribute<CustomData>("sensor_data");
    std::cout << "Custom - Sensor confidence: " << sensor.confidence << std::endl;
}

int main() {
    Graph graph;
    SearchContext context;
    
    // Add some grid cells
    for (int x = 1; x <= 6; ++x) {
        for (int y = 1; y <= 6; ++y) {
            graph.AddVertex(GridCell(x, y));
        }
    }
    
    // Show examples of different algorithms using the same SearchContext
    DijkstraExample(graph, context);
    AStarExample(graph, context);
    DStarLiteExample(graph, context);
    JumpPointSearchExample(graph, context);
    FlowNetworkExample(graph, context);
    CustomAlgorithmExample(graph, context);
    
    std::cout << "\n=== Performance and Thread Safety ===" << std::endl;
    std::cout << "Total vertices with search data: " << context.Size() << std::endl;
    
    // Show that contexts are independent (thread safety)
    SearchContext context2;
    context2.SetVertexAttribute(1001, "different_value", 999.0);
    
    std::cout << "Context 1 has different_value: " << 
        context.HasVertexAttribute(1001, "different_value") << std::endl;
    std::cout << "Context 2 has different_value: " << 
        context2.HasVertexAttribute(1001, "different_value") << std::endl;
    
    // Efficient reset for reuse
    context.Reset();
    std::cout << "After reset, context size: " << context.Size() << std::endl;
    
    std::cout << "\nFlexible SearchContext enables any algorithm!" << std::endl;
    
    return 0;
}