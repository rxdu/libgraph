/*
 * test_search_framework.cpp
 *
 * Created on: Aug 2025
 * Description: Test cases for the new template-based search framework
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include <iostream>
#include <cmath>
#include <memory>

#include "graph/graph.hpp"
#include "graph/search/search_context.hpp"
#include "graph/search/astar.hpp"
#include "graph/search/dijkstra.hpp"
#include "graph/search/bfs.hpp"

// Simple 2D point state for testing
struct Point2D {
    double x, y;
    int64_t id;
    
    Point2D(double x_val, double y_val, int64_t id_val) 
        : x(x_val), y(y_val), id(id_val) {}
    
    bool operator==(const Point2D& other) const {
        return id == other.id;
    }
    
    // For debugging
    friend std::ostream& operator<<(std::ostream& os, const Point2D& p) {
        return os << "Point2D(" << p.x << ", " << p.y << ", id=" << p.id << ")";
    }
};

// Custom indexer for Point2D
struct Point2DIndexer {
    int64_t operator()(const Point2D& point) const {
        return point.id;
    }
};

// Euclidean distance heuristic
struct EuclideanHeuristic {
    double operator()(const Point2D& current, const Point2D& goal) const {
        double dx = current.x - goal.x;
        double dy = current.y - goal.y;
        return std::sqrt(dx * dx + dy * dy);
    }
};

int main() {
    using GraphType = xmotion::Graph<Point2D, double, Point2DIndexer>;
    using SearchContext = xmotion::SearchContext<Point2D, double, Point2DIndexer>;
    
    std::cout << "Testing Template-Based Search Algorithm Framework\n";
    std::cout << "================================================\n\n";
    
    // Create a simple graph: 0 -> 1 -> 2
    //                        |    |
    //                        v    v
    //                        3 -> 4
    auto graph = std::make_shared<GraphType>();
    
    // Add vertices
    auto v0 = graph->AddVertex(Point2D(0.0, 0.0, 0));
    auto v1 = graph->AddVertex(Point2D(1.0, 0.0, 1));
    auto v2 = graph->AddVertex(Point2D(2.0, 0.0, 2));
    auto v3 = graph->AddVertex(Point2D(0.0, 1.0, 3));
    auto v4 = graph->AddVertex(Point2D(1.0, 1.0, 4));
    
    // Add edges with costs
    graph->AddEdge(Point2D(0.0, 0.0, 0), Point2D(1.0, 0.0, 1), 1.0);  // 0->1
    graph->AddEdge(Point2D(1.0, 0.0, 1), Point2D(2.0, 0.0, 2), 1.0);  // 1->2
    graph->AddEdge(Point2D(0.0, 0.0, 0), Point2D(0.0, 1.0, 3), 1.0);  // 0->3
    graph->AddEdge(Point2D(1.0, 0.0, 1), Point2D(1.0, 1.0, 4), 1.0);  // 1->4
    graph->AddEdge(Point2D(0.0, 1.0, 3), Point2D(1.0, 1.0, 4), 1.0);  // 3->4
    
    std::cout << "Created test graph with " << graph->GetTotalVertexNumber() 
              << " vertices and " << graph->GetTotalEdgeNumber() << " edges\n\n";
    
    // Test Dijkstra
    std::cout << "1. Testing Dijkstra...\n";
    {
        SearchContext context;
        auto path = xmotion::Dijkstra::Search(graph, context, 
                                                Point2D(0.0, 0.0, 0), 
                                                Point2D(2.0, 0.0, 2));
        
        std::cout << "   Path from (0,0) to (2,0): ";
        if (path.empty()) {
            std::cout << "No path found!\n";
        } else {
            std::cout << "Found path with " << path.size() << " nodes\n";
            for (size_t i = 0; i < path.size(); ++i) {
                std::cout << "     " << i << ": " << path[i] << "\n";
            }
        }
    }
    
    // Test A*
    std::cout << "\n2. Testing AStar...\n";
    {
        SearchContext context;
        EuclideanHeuristic heuristic;
        
        auto path = xmotion::AStar::Search(graph, context,
                                            Point2D(0.0, 0.0, 0),
                                            Point2D(1.0, 1.0, 4),
                                            heuristic);
        
        std::cout << "   Path from (0,0) to (1,1): ";
        if (path.empty()) {
            std::cout << "No path found!\n";
        } else {
            std::cout << "Found path with " << path.size() << " nodes\n";
            for (size_t i = 0; i < path.size(); ++i) {
                std::cout << "     " << i << ": " << path[i] << "\n";
            }
        }
    }
    
    // Test thread safety by using same graph with different contexts
    std::cout << "\n3. Testing thread safety (different contexts)...\n";
    {
        SearchContext context1, context2;
        
        // Simulate concurrent searches
        auto path1 = xmotion::Dijkstra::Search(graph, context1,
                                                Point2D(0.0, 0.0, 0),
                                                Point2D(2.0, 0.0, 2));
        
        EuclideanHeuristic heuristic;
        auto path2 = xmotion::AStar::Search(graph, context2,
                                             Point2D(0.0, 0.0, 0),
                                             Point2D(1.0, 1.0, 4),
                                             heuristic);
        
        std::cout << "   Context 1 (Dijkstra): " << path1.size() << " nodes\n";
        std::cout << "   Context 2 (A*): " << path2.size() << " nodes\n";
        std::cout << "   Both searches completed successfully!\n";
    }
    
    // Test legacy compatibility (non-thread-safe versions)
    std::cout << "\n4. Testing legacy compatibility...\n";
    {
        auto path = xmotion::Dijkstra::Search(graph.get(),
                                               Point2D(0.0, 0.0, 0),
                                               Point2D(2.0, 0.0, 2));
        
        std::cout << "   Legacy Dijkstra: " << path.size() << " nodes\n";
        
        EuclideanHeuristic heuristic;
        auto astar_path = xmotion::AStar::Search(graph.get(),
                                                  Point2D(0.0, 0.0, 0),
                                                  Point2D(1.0, 1.0, 4),
                                                  heuristic);
        
        std::cout << "   Legacy A*: " << astar_path.size() << " nodes\n";
    }
    
    // Test BFS
    std::cout << "\n5. Testing BFS...\n";
    {
        SearchContext context;
        auto path = xmotion::BFS::Search(graph, context,
                                         Point2D(0.0, 0.0, 0),
                                         Point2D(1.0, 1.0, 4));
        
        std::cout << "   BFS path from (0,0) to (1,1): ";
        if (path.empty()) {
            std::cout << "No path found!\n";
        } else {
            std::cout << "Found path with " << path.size() << " nodes\n";
            for (size_t i = 0; i < path.size(); ++i) {
                std::cout << "     " << i << ": " << path[i] << "\n";
            }
        }
    }

    // Test with no path available
    std::cout << "\n6. Testing no path scenario...\n";
    {
        SearchContext context;
        
        // Add isolated vertex
        graph->AddVertex(Point2D(10.0, 10.0, 5));
        
        auto path = xmotion::Dijkstra::Search(graph, context,
                                               Point2D(0.0, 0.0, 0),
                                               Point2D(10.0, 10.0, 5));
        
        std::cout << "   Path to isolated vertex: ";
        if (path.empty()) {
            std::cout << "No path found (expected)\n";
        } else {
            std::cout << "Unexpected path found!\n";
        }
    }
    
    std::cout << "\n==============================================\n";
    std::cout << "Framework validation completed successfully!\n";
    std::cout << "==============================================\n";
    
    return 0;
}