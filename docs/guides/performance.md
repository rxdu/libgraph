# Performance Testing Guide

This document explains how to use the performance testing framework to evaluate optimization improvements and test at various scales.

## Overview

The performance testing suite measures baseline performance for key operations:

1. **Edge Lookup Performance** - Measures O(n) linear search times
2. **Vertex Removal Performance** - Measures O(m) removal complexity
3. **Search Context Performance** - Measures allocation/context reuse overhead
4. **Concurrent Search Performance** - Measures threading scalability

## Quick Start

### Run Baseline Measurements

```bash
cd build
../scripts/run_performance_tests.sh
```

This will:
- Build the performance benchmarks if needed
- Collect system information
- Run comprehensive benchmarks
- Save timestamped results to `performance_results/`

### Compare Results

```bash
# Automatic comparison with detailed analysis
../scripts/compare_performance.py baseline_old.txt baseline_new.txt

# Manual comparison
diff -u baseline_old.txt baseline_new.txt
```

## Benchmark Categories

### Edge Lookup Benchmarks

**What it measures**: Time to find edges from vertices
**Scenarios tested**:
- Sparse graphs (10% edge density)
- Medium density (50% edge density)
- Dense graphs (90% edge density)

### Vertex Removal Benchmarks

**What it measures**: Time to remove vertices with all incoming/outgoing edges
**Scenarios tested**:
- Star graphs (worst case - central vertex connected to all others)
- Dense grid graphs (typical case)
- Different graph sizes (50, 100, 200+ vertices)

### Search Context Benchmarks

**What it measures**: Memory allocation overhead and context reuse benefits
**Scenarios tested**:
- New context creation per search
- Context reuse across searches
- Memory allocation scaling with graph size

### Concurrent Search Benchmarks

**What it measures**: Throughput scaling with multiple threads
**Scenarios tested**:
- 1, 2, 4, 8 concurrent threads
- Realistic search workloads
- Thread safety validation

## Interpreting Results

### Key Metrics

| Metric | Unit | Goal |
|--------|------|------|
| Edge Lookups | us/lookup | Lower is better |
| Vertex Removal | ms/removal | Lower is better |
| Context Creation | ms/search | Lower is better |
| Concurrent Throughput | searches/sec | Higher is better |

### Expected Performance

| Operation | Complexity | Typical Performance |
|-----------|------------|---------------------|
| Graph Construction | O(V + E) | ~500K vertices/sec |
| Dijkstra Search | O((V + E) log V) | ~35ms for 100K vertices |
| BFS | O(V + E) | Linear with graph size |
| Context Reuse | N/A | 20-50% faster than new allocation |

---

## Large-Scale Testing

For production-sized graphs (10K-1M+ vertices), large-scale testing addresses:

- **Memory consumption** and scaling patterns
- **Construction time** for realistic graph sizes
- **Search performance** on graphs too large to fit in CPU cache
- **Concurrent access** patterns with memory pressure
- **System resource utilization** under heavy loads

### When to Use Large-Scale Tests

**Use large-scale tests when:**
- Implementing optimizations for memory usage
- Testing performance on production-sized graphs
- Validating algorithmic complexity claims (O(n), O(m), etc.)
- Measuring cache effects and memory hierarchy impact
- Testing concurrent performance under memory pressure
- Benchmarking system limits and breaking points

**Use micro-benchmarks when:**
- Testing specific operations (edge lookup, vertex removal)
- Measuring small improvements (5-50% gains)
- Quick development feedback cycles
- Regression testing during development

### System Requirements

| Graph Size | Memory Needed | Use Case |
|-----------|---------------|----------|
| 10K vertices | ~50 MB | Development, CI testing |
| 100K vertices | ~500 MB | Realistic applications |
| 500K vertices | ~2.5 GB | Large applications |
| 1M+ vertices | ~5+ GB | Enterprise, research |

```bash
# Check available memory
free -h

# Recommended minimums:
# 4GB RAM: Up to 100K vertices
# 8GB RAM: Up to 500K vertices
# 16GB RAM: Up to 1M+ vertices
```

### Graph Types for Large-Scale Testing

#### Road Networks (Sparse, Connected)
```cpp
// ~4 edges per vertex (realistic road connectivity)
auto graph = LargeGraphGenerator::CreateRoadNetwork(316, 316); // 100K vertices
```
**Characteristics:**
- Low average degree (4-8 edges/vertex)
- High connectivity (most vertices reachable)
- Models: GPS navigation, logistics

#### Social Networks (Power-Law Distribution)
```cpp
// Variable degree distribution (some highly connected nodes)
auto graph = LargeGraphGenerator::CreateSocialNetwork(100000);
```
**Characteristics:**
- Few highly connected vertices
- Many low-degree vertices
- Models: Social media, web graphs

#### Clustered Graphs (Dense Local, Sparse Global)
```cpp
// Dense connections within clusters, sparse between clusters
auto graph = LargeGraphGenerator::CreateClusteredGraph(1000, 100); // 100K vertices
```
**Characteristics:**
- Dense local neighborhoods
- Sparse inter-cluster connections
- Models: Hierarchical systems, modules

### Running Large-Scale Tests

```bash
# Run comprehensive large-scale benchmarks
cd build
../scripts/run_large_scale_tests.sh

# Or manually with timeout (recommended)
timeout 30m ./bin/test_large_scale_benchmarks

# Monitor memory usage during execution
watch -n 1 'free -h && ps aux | grep test_large_scale'
```

### Understanding Results

#### Construction Performance
```
100K Road Network (316x316):
  Construction time: 0.18 seconds
  Vertices: 99856
  Edges: 398727
  Memory used: 42.3 MB
  Memory per vertex: 444 bytes
  Construction rate: 549296 vertices/sec
```

**Key Metrics:**
- **Construction rate**: Vertices/second (higher is better)
- **Memory per vertex**: Bytes/vertex (lower is better)
- **Edge/vertex ratio**: Graph density indicator

#### Search Performance Scaling
```
Road Network Searches:
  100x100 network (10000 vertices):
    Dijkstra: 2.7 ms avg, 8/8 successful, avg path: 48 nodes
  200x200 network (40000 vertices):
    Dijkstra: 12.6 ms avg, 8/8 successful, avg path: 96 nodes
  316x316 network (99856 vertices):
    Dijkstra: 35.9 ms avg, 8/8 successful, avg path: 152 nodes
```

#### Memory Scaling Analysis
```
Memory Scaling Analysis:
  50x50 (2500 vertices): 0.9 MB (378 bytes/vertex)
  100x100 (10000 vertices): 4.7 MB (491 bytes/vertex)
  200x200 (40000 vertices): 18.2 MB (477 bytes/vertex)
```

---

## Performance Best Practices

### Consistent Environment

- Run tests on same machine with same load
- Use Release build mode for accurate measurements
- Close unnecessary applications
- Run multiple times and average results

### Context Reuse

For repeated searches, always reuse SearchContext:

```cpp
SearchContext<State, Transition, Indexer> context;
context.Reserve(graph.GetVertexCount());

for (const auto& query : queries) {
    context.Reset();  // Clears state, preserves capacity
    auto path = Dijkstra::Search(graph, context, query.start, query.goal);
}
```

### Pre-allocation

Avoid allocations during search:

```cpp
// Reserve graph capacity before bulk inserts
graph.ReserveVertices(expected_vertices);

// Reserve context before search
context.Reserve(graph.GetVertexCount());
```

### Algorithm Selection

| Graph Type | Recommended Algorithm |
|------------|----------------------|
| Weighted, with heuristic | A* |
| Weighted, no heuristic | Dijkstra |
| Unweighted | BFS |
| Traversal/reachability | DFS |

---

## Troubleshooting

### Common Issues

1. **Out of Memory (OOM)**
   - Symptoms: Process killed, system freezing
   - Solutions: Reduce graph size, increase swap, use smaller data types

2. **Excessive Swap Usage**
   ```bash
   # Check swap usage
   swapon --show
   free -h
   ```

3. **Long Execution Times**
   ```bash
   # Use timeouts and progress monitoring
   timeout 30m ./bin/test_large_scale_benchmarks
   ```

4. **Inconsistent Results**
   ```bash
   # Ensure consistent system state
   echo 3 > /proc/sys/vm/drop_caches  # Clear caches (requires root)
   ```

### Performance Analysis Tools

```bash
# Memory profiling
valgrind --tool=massif ./bin/test_large_scale_benchmarks

# CPU profiling
perf record ./bin/test_large_scale_benchmarks
perf report

# System monitoring
iostat -x 1
vmstat 1
```

---

## Files Overview

- `test_performance_benchmarks.cpp` - Main benchmark implementation
- `run_performance_tests.sh` - Test runner script
- `compare_performance.py` - Result comparison tool
- `performance_results/` - Timestamped results directory

This framework provides the foundation for quantitative performance evaluation and ensures optimizations deliver measurable improvements.
