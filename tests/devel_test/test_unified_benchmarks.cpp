/*
 * test_unified_benchmarks.cpp
 *
 * Created on: Aug 2025
 * Description: Unified performance benchmark suite combining micro and large-scale tests
 *              Outputs all results to a single comprehensive report file
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include <iostream>
#include <vector>
#include <chrono>
#include <algorithm>
#include <random>
#include <thread>
#include <future>
#include <iomanip>
#include <fstream>
#include <memory>
#include <cmath>
#include <sstream>

#include "graph/graph.hpp"
#include "graph/search/dijkstra.hpp"
#include "graph/search/astar.hpp"
#include "graph/search/bfs.hpp"
#include "graph/search/dfs.hpp"

using namespace xmotion;

// Unified timer for all benchmarks
class UnifiedTimer {
public:
    using clock_t = std::chrono::high_resolution_clock;
    using duration_t = std::chrono::nanoseconds;
    
    void start() {
        start_time_ = clock_t::now();
    }
    
    double stop_seconds() {
        auto end_time = clock_t::now();
        auto duration = std::chrono::duration_cast<duration_t>(end_time - start_time_);
        return duration.count() / 1e9;
    }
    
    double stop_ms() {
        auto end_time = clock_t::now();
        auto duration = std::chrono::duration_cast<duration_t>(end_time - start_time_);
        return duration.count() / 1e6;
    }
    
    double stop_us() {
        auto end_time = clock_t::now();
        auto duration = std::chrono::duration_cast<duration_t>(end_time - start_time_);
        return duration.count() / 1e3;
    }

private:
    clock_t::time_point start_time_;
};

// Memory tracking utilities
class MemoryTracker {
public:
    static size_t GetCurrentMemoryUsage() {
        std::ifstream statm("/proc/self/statm");
        if (statm.is_open()) {
            size_t size, resident, shared, text, lib, data, dt;
            statm >> size >> resident >> shared >> text >> lib >> data >> dt;
            return resident * 4096; // Convert pages to bytes
        }
        return 0;
    }
    
    static std::string FormatBytes(size_t bytes) {
        const char* units[] = {"B", "KB", "MB", "GB"};
        int unit = 0;
        double size = static_cast<double>(bytes);
        
        while (size >= 1024 && unit < 3) {
            size /= 1024;
            unit++;
        }
        
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(1) << size << " " << units[unit];
        return oss.str();
    }
};

// Unified state types for different test scales
struct MicroState {
    int x, y, id;
    MicroState(int x = 0, int y = 0, int id = 0) : x(x), y(y), id(id) {}
    int GetId() const { return id; }
    bool operator==(const MicroState& other) const {
        return x == other.x && y == other.y && id == other.id;
    }
};

struct LargeState {
    int32_t x, y;
    LargeState(int32_t x = 0, int32_t y = 0) : x(x), y(y) {}
    int64_t GetId() const { return static_cast<int64_t>(y) * 100000 + x; }
    bool operator==(const LargeState& other) const {
        return x == other.x && y == other.y;
    }
};

using MicroGraph = Graph<MicroState, double, DefaultIndexer<MicroState>>;
using LargeGraph = Graph<LargeState, float, DefaultIndexer<LargeState>>;

// Unified benchmark runner with single output
class UnifiedBenchmarkSuite {
private:
    std::ostringstream report_;
    size_t initial_memory_;
    
public:
    UnifiedBenchmarkSuite() {
        initial_memory_ = MemoryTracker::GetCurrentMemoryUsage();
        GenerateHeader();
    }
    
    void RunAllBenchmarks() {
        // Micro-benchmarks (detailed operation analysis)
        RunMicroBenchmarks();
        
        // Large-scale benchmarks (realistic workloads)
        RunLargeScaleBenchmarks();
        
        // Summary and recommendations
        GenerateSummary();
    }
    
    std::string GetReport() const {
        return report_.str();
    }
    
private:
    void GenerateHeader() {
        report_ << "UNIFIED PERFORMANCE BENCHMARK REPORT\n";
        report_ << "====================================\n";
        report_ << "Generated: " << GetTimestamp() << "\n";
        report_ << "Initial Memory: " << MemoryTracker::FormatBytes(initial_memory_) << "\n";
        report_ << "\n";
        report_ << "This report combines micro-benchmarks (operation-level) and large-scale\n";
        report_ << "benchmarks (realistic workloads) to provide complete performance analysis.\n";
        report_ << "\n";
    }
    
    void RunMicroBenchmarks() {
        report_ << "SECTION 1: MICRO-BENCHMARKS\n";
        report_ << "===========================\n";
        report_ << "Testing specific operations for optimization targeting\n\n";
        
        RunEdgeLookupMicro();
        RunVertexRemovalMicro();
        RunSearchContextMicro();
        RunConcurrentMicro();
    }
    
    void RunLargeScaleBenchmarks() {
        report_ << "\nSECTION 2: LARGE-SCALE BENCHMARKS\n";
        report_ << "=================================\n";
        report_ << "Testing realistic workloads and scaling characteristics\n\n";
        
        RunConstructionBenchmarks();
        RunSearchScalingBenchmarks();
        RunMemoryScalingBenchmarks();
        RunConcurrentScalingBenchmarks();
    }
    
    void RunEdgeLookupMicro() {
        report_ << "Edge Lookup Performance (Micro):\n";
        report_ << "---------------------------------\n";
        
        std::vector<std::pair<int, double>> configs = {{100, 0.1}, {100, 0.5}, {50, 0.9}};
        
        for (auto config : configs) {
            int vertices = config.first;
            double density = config.second;
            
            auto graph = CreateRandomMicroGraph(vertices, density);
            int num_edges = CountEdges(graph);
            
            double lookup_time = BenchmarkEdgeLookups(graph, 1000);
            
            report_ << "  " << vertices << " vertices, " << num_edges << " edges (" 
                    << std::fixed << std::setprecision(1) << density * 100 << "% density): "
                    << std::setprecision(2) << lookup_time << " μs/lookup\n";
        }
        report_ << "\n";
    }
    
    void RunVertexRemovalMicro() {
        report_ << "Vertex Removal Performance (Micro):\n";
        report_ << "------------------------------------\n";
        
        // Star graph (worst case)
        report_ << "  Star Graph (worst case):\n";
        std::vector<int> star_sizes = {50, 100, 200};
        for (int size : star_sizes) {
            auto graph = CreateStarMicroGraph(size);
            double removal_time = BenchmarkVertexRemoval(graph, MicroState(0, 0, 0));
            report_ << "    " << size << " vertices: " << std::fixed << std::setprecision(2) 
                    << removal_time << " ms\n";
        }
        
        // Grid graph (typical case)
        report_ << "  Grid Graph (typical case):\n";
        std::vector<std::pair<int, int>> grid_sizes = {{10, 10}, {15, 15}, {20, 20}};
        for (auto size : grid_sizes) {
            int w = size.first;
            int h = size.second;
            auto graph = CreateGridMicroGraph(w, h);
            double removal_time = BenchmarkVertexRemoval(graph, MicroState(w/2, h/2, (h/2) * w + (w/2)));
            report_ << "    " << w << "x" << h << " grid: " << std::fixed << std::setprecision(2) 
                    << removal_time << " ms\n";
        }
        report_ << "\n";
    }
    
    void RunSearchContextMicro() {
        report_ << "Search Context Performance (Micro):\n";
        report_ << "------------------------------------\n";
        
        auto graph = CreateGridMicroGraph(20, 20);
        MicroState start(0, 0, 0);
        MicroState goal(19, 19, 19 * 20 + 19);
        
        // Context creation overhead
        double creation_time = BenchmarkContextCreation(graph, start, goal, 100);
        report_ << "  New context per search: " << std::fixed << std::setprecision(2) 
                << creation_time << " ms/search\n";
        
        // Context reuse
        double reuse_time = BenchmarkContextReuse(graph, start, goal, 100);
        report_ << "  Reused context: " << std::fixed << std::setprecision(2) 
                << reuse_time << " ms/search\n";
        
        double improvement = ((creation_time - reuse_time) / creation_time) * 100;
        report_ << "  Context reuse improvement: " << std::fixed << std::setprecision(1) 
                << improvement << "%\n";
        report_ << "\n";
    }
    
    void RunConcurrentMicro() {
        report_ << "Concurrent Search Performance (Micro):\n";
        report_ << "---------------------------------------\n";
        
        auto graph = CreateGridMicroGraph(25, 25);
        std::vector<int> thread_counts = {1, 2, 4, 8};
        
        for (int threads : thread_counts) {
            double throughput = BenchmarkConcurrentSearches(graph, threads, 25);
            report_ << "  " << threads << " threads: " << std::fixed << std::setprecision(0) 
                    << throughput << " searches/sec\n";
        }
        report_ << "\n";
    }
    
    void RunConstructionBenchmarks() {
        report_ << "Graph Construction Performance (Large-Scale):\n";
        report_ << "----------------------------------------------\n";
        
        std::vector<std::pair<std::string, std::function<std::shared_ptr<LargeGraph>()>>> tests = {
            {"10K Road Network (100x100)", [this]() { return CreateRoadNetwork(100, 100); }},
            {"100K Road Network (316x316)", [this]() { return CreateRoadNetwork(316, 316); }},
            {"50K Social Network", [this]() { return CreateSocialNetwork(50000); }},
        };
        
        for (auto& test : tests) {
            std::string name = test.first;
            std::function<std::shared_ptr<LargeGraph>()> generator = test.second;
            
            size_t memory_before = MemoryTracker::GetCurrentMemoryUsage();
            UnifiedTimer timer;
            timer.start();
            
            auto graph = generator();
            
            double time = timer.stop_seconds();
            size_t memory_after = MemoryTracker::GetCurrentMemoryUsage();
            size_t memory_used = memory_after - memory_before;
            
            size_t vertex_count = CountVertices(graph);
            size_t edge_count = CountLargeEdges(graph);
            
            report_ << "  " << name << ":\n";
            report_ << "    Construction time: " << std::fixed << std::setprecision(2) << time << " seconds\n";
            report_ << "    Vertices: " << vertex_count << ", Edges: " << edge_count << "\n";
            report_ << "    Memory used: " << MemoryTracker::FormatBytes(memory_used) << "\n";
            report_ << "    Rate: " << static_cast<int>(vertex_count / time) << " vertices/sec\n";
            report_ << "    Memory efficiency: " << (memory_used / vertex_count) << " bytes/vertex\n\n";
        }
    }
    
    void RunSearchScalingBenchmarks() {
        report_ << "Search Algorithm Scaling (Large-Scale):\n";
        report_ << "----------------------------------------\n";
        
        std::vector<std::pair<int, int>> sizes = {{100, 100}, {200, 200}, {316, 316}};
        
        report_ << "  Road Network Search Performance:\n";
        for (auto size : sizes) {
            int w = size.first;
            int h = size.second;
            
            auto graph = CreateRoadNetwork(w, h);
            auto results = BenchmarkSearchAlgorithms(graph, w, h);
            
            report_ << "    " << w << "x" << h << " (" << w*h << " vertices):\n";
            report_ << "      Dijkstra: " << std::fixed << std::setprecision(1) 
                    << results[0] << " ms avg\n";
            report_ << "      BFS: " << std::fixed << std::setprecision(1) 
                    << results[1] << " ms avg\n";
            report_ << "      DFS: " << std::fixed << std::setprecision(1) 
                    << results[2] << " ms avg\n";
        }
        report_ << "\n";
    }
    
    void RunMemoryScalingBenchmarks() {
        report_ << "Memory Scaling Analysis (Large-Scale):\n";
        report_ << "---------------------------------------\n";
        
        std::vector<std::pair<int, int>> sizes = {{50, 50}, {100, 100}, {200, 200}, {300, 300}};
        
        for (auto size : sizes) {
            int w = size.first;
            int h = size.second;
            
            size_t memory_before = MemoryTracker::GetCurrentMemoryUsage();
            auto graph = CreateRoadNetwork(w, h);
            size_t memory_after = MemoryTracker::GetCurrentMemoryUsage();
            
            size_t memory_used = memory_after - memory_before;
            size_t vertex_count = w * h;
            
            report_ << "  " << w << "x" << h << " (" << vertex_count << " vertices): "
                    << MemoryTracker::FormatBytes(memory_used) 
                    << " (" << (memory_used / vertex_count) << " bytes/vertex)\n";
        }
        report_ << "\n";
    }
    
    void RunConcurrentScalingBenchmarks() {
        report_ << "Concurrent Scaling Analysis (Large-Scale):\n";
        report_ << "-------------------------------------------\n";
        
        auto graph = CreateRoadNetwork(200, 200);
        std::vector<int> thread_counts = {1, 2, 4, 8};
        
        for (int threads : thread_counts) {
            double throughput = BenchmarkLargeConcurrentSearches(graph, threads);
            report_ << "  " << threads << " threads: " << std::fixed << std::setprecision(0) 
                    << throughput << " searches/sec (40K vertex graph)\n";
        }
        report_ << "\n";
    }
    
    void GenerateSummary() {
        size_t final_memory = MemoryTracker::GetCurrentMemoryUsage();
        
        report_ << "SECTION 3: SUMMARY AND RECOMMENDATIONS\n";
        report_ << "======================================\n\n";
        
        report_ << "Memory Usage Summary:\n";
        report_ << "---------------------\n";
        report_ << "  Initial memory: " << MemoryTracker::FormatBytes(initial_memory_) << "\n";
        report_ << "  Peak memory: " << MemoryTracker::FormatBytes(final_memory) << "\n";
        report_ << "  Total allocated: " << MemoryTracker::FormatBytes(final_memory - initial_memory_) << "\n\n";
        
        report_ << "Performance Optimization Targets:\n";
        report_ << "----------------------------------\n";
        report_ << "1. EDGE LOOKUP OPTIMIZATION\n";
        report_ << "   Current: O(n) linear search through edge lists\n";
        report_ << "   Target: O(1) hash-based lookup\n";
        report_ << "   Expected improvement: 10-100x faster edge operations\n\n";
        
        report_ << "2. VERTEX REMOVAL OPTIMIZATION\n";
        report_ << "   Current: O(m²) - scan all vertices for incoming edges\n";
        report_ << "   Target: O(m) - maintain bidirectional edge references\n";
        report_ << "   Expected improvement: 2-10x faster removal operations\n\n";
        
        report_ << "3. MEMORY POOLING\n";
        report_ << "   Current: Dynamic allocation per search context\n";
        report_ << "   Target: Pre-allocated memory pools\n";
        report_ << "   Expected improvement: 20-50% faster context operations\n\n";
        
        report_ << "4. CONTEXT REUSE\n";
        report_ << "   Current: Limited reuse awareness\n";
        report_ << "   Target: Systematic context reuse patterns\n";
        report_ << "   Expected improvement: 30-70% faster repeated searches\n\n";
        
        report_ << "Benchmarking Notes:\n";
        report_ << "-------------------\n";
        report_ << "- Use this report as baseline for optimization evaluation\n";
        report_ << "- Run benchmarks before and after each optimization\n";
        report_ << "- Focus on operations showing highest time/memory usage\n";
        report_ << "- Test both micro-improvements and large-scale impact\n\n";
        
        report_ << "System Recommendations:\n";
        report_ << "------------------------\n";
        report_ << "- 4GB RAM: Up to 100K vertices\n";
        report_ << "- 8GB RAM: Up to 500K vertices\n";
        report_ << "- 16GB RAM: Up to 1M+ vertices\n";
        report_ << "- Use Release build for production measurements\n";
        report_ << "- Monitor memory usage during large-scale tests\n\n";
        
        report_ << "====================================\n";
        report_ << "End of Unified Performance Report\n";
        report_ << "Generated: " << GetTimestamp() << "\n";
        report_ << "====================================\n";
    }
    
    // Helper methods for graph creation and benchmarking
    std::shared_ptr<MicroGraph> CreateRandomMicroGraph(int vertices, double density) {
        auto graph = std::make_shared<MicroGraph>();
        std::mt19937 rng(42);
        std::uniform_real_distribution<double> cost_dist(1.0, 10.0);
        std::uniform_real_distribution<double> edge_prob(0.0, 1.0);
        
        for (int i = 0; i < vertices; ++i) {
            graph->AddVertex(MicroState(i % 100, i / 100, i));
        }
        
        for (int i = 0; i < vertices; ++i) {
            for (int j = i + 1; j < vertices; ++j) {
                if (edge_prob(rng) < density) {
                    MicroState state_i(i % 100, i / 100, i);
                    MicroState state_j(j % 100, j / 100, j);
                    graph->AddEdge(state_i, state_j, cost_dist(rng));
                    graph->AddEdge(state_j, state_i, cost_dist(rng));
                }
            }
        }
        return graph;
    }
    
    std::shared_ptr<MicroGraph> CreateStarMicroGraph(int vertices) {
        auto graph = std::make_shared<MicroGraph>();
        
        MicroState center(0, 0, 0);
        graph->AddVertex(center);
        
        for (int i = 1; i < vertices; ++i) {
            MicroState spoke(i, 0, i);
            graph->AddVertex(spoke);
            graph->AddEdge(center, spoke, 1.0);
            graph->AddEdge(spoke, center, 1.0);
        }
        return graph;
    }
    
    std::shared_ptr<MicroGraph> CreateGridMicroGraph(int width, int height) {
        auto graph = std::make_shared<MicroGraph>();
        
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                int id = y * width + x;
                graph->AddVertex(MicroState(x, y, id));
            }
        }
        
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                MicroState current(x, y, y * width + x);
                
                std::vector<std::pair<int, int>> neighbors = {{x+1, y}, {x-1, y}, {x, y+1}, {x, y-1}};
                for (auto neighbor : neighbors) {
                    int nx = neighbor.first;
                    int ny = neighbor.second;
                    if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
                        MicroState neighbor_state(nx, ny, ny * width + nx);
                        graph->AddEdge(current, neighbor_state, 1.0);
                    }
                }
            }
        }
        return graph;
    }
    
    std::shared_ptr<LargeGraph> CreateRoadNetwork(int width, int height) {
        auto graph = std::make_shared<LargeGraph>();
        std::mt19937 rng(42);
        std::uniform_real_distribution<float> cost_dist(1.0f, 5.0f);
        std::uniform_real_distribution<float> connection_prob(0.0f, 1.0f);
        
        // Add vertices
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                graph->AddVertex(LargeState(x, y));
            }
        }
        
        // Add edges
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                LargeState current(x, y);
                
                std::vector<std::pair<int, int>> neighbors = {{x+1, y}, {x-1, y}, {x, y+1}, {x, y-1}};
                for (auto neighbor : neighbors) {
                    int nx = neighbor.first;
                    int ny = neighbor.second;
                    if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
                        if (connection_prob(rng) < 0.85) {
                            LargeState neighbor_state(nx, ny);
                            graph->AddEdge(current, neighbor_state, cost_dist(rng));
                        }
                    }
                }
            }
        }
        return graph;
    }
    
    std::shared_ptr<LargeGraph> CreateSocialNetwork(int vertices) {
        auto graph = std::make_shared<LargeGraph>();
        std::mt19937 rng(42);
        std::uniform_real_distribution<float> cost_dist(1.0f, 3.0f);
        std::exponential_distribution<float> degree_dist(0.1f);
        
        int grid_size = static_cast<int>(std::sqrt(vertices)) + 1;
        for (int i = 0; i < vertices; ++i) {
            int x = i % grid_size;
            int y = i / grid_size;
            graph->AddVertex(LargeState(x, y));
        }
        
        for (int i = 0; i < vertices; ++i) {
            int x = i % grid_size;
            int y = i / grid_size;
            LargeState current(x, y);
            
            int connections = std::min(static_cast<int>(degree_dist(rng)) + 1, 50);
            std::uniform_int_distribution<int> target_dist(0, vertices - 1);
            
            for (int c = 0; c < connections; ++c) {
                int target_idx = target_dist(rng);
                if (target_idx != i) {
                    int tx = target_idx % grid_size;
                    int ty = target_idx / grid_size;
                    LargeState target(tx, ty);
                    graph->AddEdge(current, target, cost_dist(rng));
                }
            }
        }
        return graph;
    }
    
    // Benchmark implementation methods
    int CountEdges(std::shared_ptr<MicroGraph> graph) {
        int count = 0;
        for (auto v_it = graph->vertex_begin(); v_it != graph->vertex_end(); ++v_it) {
            count += std::distance(v_it->edge_begin(), v_it->edge_end());
        }
        return count;
    }
    
    size_t CountVertices(std::shared_ptr<LargeGraph> graph) {
        return std::distance(graph->vertex_begin(), graph->vertex_end());
    }
    
    size_t CountLargeEdges(std::shared_ptr<LargeGraph> graph) {
        size_t count = 0;
        for (auto v_it = graph->vertex_begin(); v_it != graph->vertex_end(); ++v_it) {
            count += std::distance(v_it->edge_begin(), v_it->edge_end());
        }
        return count;
    }
    
    double BenchmarkEdgeLookups(std::shared_ptr<MicroGraph> graph, int num_lookups) {
        std::mt19937 rng(42);
        std::vector<std::pair<MicroState, MicroState>> vertex_pairs;
        
        for (auto v1 = graph->vertex_begin(); v1 != graph->vertex_end(); ++v1) {
            for (auto v2 = graph->vertex_begin(); v2 != graph->vertex_end(); ++v2) {
                if (v1 != v2) {
                    vertex_pairs.emplace_back(v1->state, v2->state);
                }
            }
        }
        
        if (vertex_pairs.empty()) return 0.0;
        
        std::uniform_int_distribution<size_t> pair_dist(0, vertex_pairs.size() - 1);
        
        UnifiedTimer timer;
        timer.start();
        
        for (int i = 0; i < num_lookups; ++i) {
            std::pair<MicroState, MicroState> pair = vertex_pairs[pair_dist(rng)];
            MicroState src = pair.first;
            MicroState dst = pair.second;
            auto src_vertex = graph->FindVertex(src);
            if (src_vertex != graph->vertex_end()) {
                auto edge_it = src_vertex->FindEdge(dst.GetId());
                // Just access the result to prevent optimization
                (void)edge_it;
            }
        }
        
        return timer.stop_us() / num_lookups;
    }
    
    double BenchmarkVertexRemoval(std::shared_ptr<MicroGraph> graph, const MicroState& vertex) {
        UnifiedTimer timer;
        timer.start();
        graph->RemoveVertex(vertex);
        return timer.stop_ms();
    }
    
    double BenchmarkContextCreation(std::shared_ptr<MicroGraph> graph, const MicroState& start, const MicroState& goal, int num_searches) {
        UnifiedTimer timer;
        timer.start();
        
        for (int i = 0; i < num_searches; ++i) {
            SearchContext<MicroState, double, DefaultIndexer<MicroState>> context;
            auto path = Dijkstra::Search(graph.get(), context, start, goal);
            (void)path; // Prevent optimization
        }
        
        return timer.stop_ms() / num_searches;
    }
    
    double BenchmarkContextReuse(std::shared_ptr<MicroGraph> graph, const MicroState& start, const MicroState& goal, int num_searches) {
        SearchContext<MicroState, double, DefaultIndexer<MicroState>> context;
        
        UnifiedTimer timer;
        timer.start();
        
        for (int i = 0; i < num_searches; ++i) {
            context.Reset();
            auto path = Dijkstra::Search(graph.get(), context, start, goal);
            (void)path; // Prevent optimization
        }
        
        return timer.stop_ms() / num_searches;
    }
    
    double BenchmarkConcurrentSearches(std::shared_ptr<MicroGraph> graph, int num_threads, int searches_per_thread) {
        std::mt19937 rng(42);
        std::uniform_int_distribution<int> coord_dist(0, 24);
        
        std::vector<std::pair<MicroState, MicroState>> search_pairs;
        for (int i = 0; i < searches_per_thread * num_threads; ++i) {
            int start_id = coord_dist(rng);
            int goal_id = coord_dist(rng);
            MicroState start(start_id % 5, start_id / 5, start_id);
            MicroState goal(goal_id % 5, goal_id / 5, goal_id);
            search_pairs.emplace_back(start, goal);
        }
        
        UnifiedTimer timer;
        timer.start();
        
        std::vector<std::future<int>> futures;
        for (int t = 0; t < num_threads; ++t) {
            futures.push_back(std::async(std::launch::async, [&, t]() {
                SearchContext<MicroState, double, DefaultIndexer<MicroState>> context;
                for (int i = 0; i < searches_per_thread; ++i) {
                    int search_idx = t * searches_per_thread + i;
                    std::pair<MicroState, MicroState> search_pair = search_pairs[search_idx];
                    MicroState start = search_pair.first;
                    MicroState goal = search_pair.second;
                    
                    context.Reset();
                    auto path = Dijkstra::Search(graph.get(), context, start, goal);
                    (void)path; // Prevent optimization
                }
                return searches_per_thread;
            }));
        }
        
        for (auto& future : futures) {
            future.get();
        }
        
        double total_time = timer.stop_seconds();
        return (searches_per_thread * num_threads) / total_time;
    }
    
    std::vector<double> BenchmarkSearchAlgorithms(std::shared_ptr<LargeGraph> graph, int max_x, int max_y) {
        std::mt19937 rng(42);
        std::uniform_int_distribution<int> x_dist(0, max_x - 1);
        std::uniform_int_distribution<int> y_dist(0, max_y - 1);
        
        std::vector<std::pair<LargeState, LargeState>> test_cases;
        for (int i = 0; i < 8; ++i) {
            LargeState start(x_dist(rng), y_dist(rng));
            LargeState goal(x_dist(rng), y_dist(rng));
            test_cases.emplace_back(start, goal);
        }
        
        std::vector<double> results;
        SearchContext<LargeState, float, DefaultIndexer<LargeState>> context;
        
        // Dijkstra
        UnifiedTimer timer;
        timer.start();
        for (auto test_case : test_cases) {
            context.Reset();
            auto path = Dijkstra::Search(graph.get(), context, test_case.first, test_case.second);
            (void)path;
        }
        results.push_back(timer.stop_ms() / test_cases.size());
        
        // BFS
        timer.start();
        for (auto test_case : test_cases) {
            context.Reset();
            auto path = BFS::Search(graph.get(), context, test_case.first, test_case.second);
            (void)path;
        }
        results.push_back(timer.stop_ms() / test_cases.size());
        
        // DFS
        timer.start();
        for (auto test_case : test_cases) {
            context.Reset();
            auto path = DFS::Search(graph.get(), context, test_case.first, test_case.second);
            (void)path;
        }
        results.push_back(timer.stop_ms() / test_cases.size());
        
        return results;
    }
    
    double BenchmarkLargeConcurrentSearches(std::shared_ptr<LargeGraph> graph, int num_threads) {
        std::mt19937 rng(42);
        std::uniform_int_distribution<int> coord_dist(0, 199);
        
        int searches_per_thread = 10;
        std::vector<std::pair<LargeState, LargeState>> search_pairs;
        for (int i = 0; i < searches_per_thread * num_threads; ++i) {
            LargeState start(coord_dist(rng), coord_dist(rng));
            LargeState goal(coord_dist(rng), coord_dist(rng));
            search_pairs.emplace_back(start, goal);
        }
        
        UnifiedTimer timer;
        timer.start();
        
        std::vector<std::future<int>> futures;
        for (int t = 0; t < num_threads; ++t) {
            futures.push_back(std::async(std::launch::async, [&, t]() {
                SearchContext<LargeState, float, DefaultIndexer<LargeState>> context;
                for (int i = 0; i < searches_per_thread; ++i) {
                    int search_idx = t * searches_per_thread + i;
                    std::pair<LargeState, LargeState> search_pair = search_pairs[search_idx];
                    LargeState start = search_pair.first;
                    LargeState goal = search_pair.second;
                    
                    context.Reset();
                    auto path = Dijkstra::Search(graph.get(), context, start, goal);
                    (void)path;
                }
                return searches_per_thread;
            }));
        }
        
        for (auto& future : futures) {
            future.get();
        }
        
        double total_time = timer.stop_seconds();
        return (searches_per_thread * num_threads) / total_time;
    }
    
    std::string GetTimestamp() const {
        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);
        std::ostringstream oss;
        oss << std::put_time(std::localtime(&time_t), "%Y-%m-%d %H:%M:%S");
        return oss.str();
    }
};

int main() {
    std::cout << "Running Unified Performance Benchmark Suite...\n";
    std::cout << "===============================================\n";
    std::cout << "This will generate a comprehensive single-file report.\n\n";
    
    UnifiedBenchmarkSuite suite;
    suite.RunAllBenchmarks();
    
    std::string report = suite.GetReport();
    std::cout << report;
    
    return 0;
}