# Large-Scale Performance Testing Guide

This document explains how to test performance on very large graphs (10K-1M+ vertices) and understand scalability characteristics.

## Overview

Large-scale performance testing addresses different concerns than micro-benchmarks:

- **Memory consumption** and scaling patterns
- **Construction time** for realistic graph sizes  
- **Search performance** on graphs too large to fit in CPU cache
- **Concurrent access** patterns with memory pressure
- **System resource utilization** under heavy loads

## When to Use Large-Scale Testing

### Use large-scale tests when:
- ✅ Implementing optimizations for memory usage
- ✅ Testing performance on production-sized graphs
- ✅ Validating algorithmic complexity claims (O(n), O(m), etc.)
- ✅ Measuring cache effects and memory hierarchy impact
- ✅ Testing concurrent performance under memory pressure
- ✅ Benchmarking system limits and breaking points

### Use micro-benchmarks when:
- ⚡ Testing specific operations (edge lookup, vertex removal)
- ⚡ Measuring small improvements (5-50% gains)
- ⚡ Quick development feedback cycles
- ⚡ Regression testing during development

## Graph Sizes and System Requirements

### Memory Requirements by Graph Size

| Graph Size | Memory Needed | Use Case |
|-----------|---------------|----------|
| 10K vertices | ~50 MB | Development, CI testing |
| 100K vertices | ~500 MB | Realistic applications |
| 500K vertices | ~2.5 GB | Large applications |
| 1M+ vertices | ~5+ GB | Enterprise, research |

### System Recommendations

```bash
# Check available memory
free -h

# Recommended minimums:
# 4GB RAM: Up to 100K vertices
# 8GB RAM: Up to 500K vertices  
# 16GB RAM: Up to 1M+ vertices
```

## Graph Types for Large-Scale Testing

### 1. Road Networks (Sparse, Connected)
```cpp
// ~4 edges per vertex (realistic road connectivity)
auto graph = LargeGraphGenerator::CreateRoadNetwork(316, 316); // 100K vertices
```
**Characteristics:**
- Low average degree (4-8 edges/vertex)
- High connectivity (most vertices reachable)
- Realistic pathfinding scenarios
- Models: GPS navigation, logistics

### 2. Social Networks (Power-Law Distribution)
```cpp
// Variable degree distribution (some highly connected nodes)
auto graph = LargeGraphGenerator::CreateSocialNetwork(100000);
```
**Characteristics:**
- Few highly connected vertices
- Many low-degree vertices
- Small-world properties (short average paths)
- Models: Social media, web graphs

### 3. Clustered Graphs (Dense Local, Sparse Global)
```cpp
// Dense connections within clusters, sparse between clusters
auto graph = LargeGraphGenerator::CreateClusteredGraph(1000, 100); // 100K vertices
```
**Characteristics:**
- Dense local neighborhoods
- Sparse inter-cluster connections
- Models: Hierarchical systems, modules

## Running Large-Scale Tests

### Quick Start

```bash
# Run comprehensive large-scale benchmarks
cd build
../scripts/run_large_scale_tests.sh
```

### Manual Execution

```bash
# Build large-scale benchmarks
make test_large_scale_benchmarks

# Run with timeout (recommended)
timeout 30m ./bin/test_large_scale_benchmarks

# Monitor memory usage during execution
watch -n 1 'free -h && ps aux | grep test_large_scale'
```

### Safe Testing Practices

1. **Check available memory first**:
```bash
# Ensure sufficient memory
AVAILABLE_MB=$(awk '/MemAvailable/ {print int($2/1024)}' /proc/meminfo)
echo "Available memory: ${AVAILABLE_MB} MB"
```

2. **Use timeouts to prevent system freeze**:
```bash
# 30-minute timeout for safety
timeout 1800 ./bin/test_large_scale_benchmarks
```

3. **Monitor system resources**:
```bash
# In another terminal
htop
# or
watch -n 1 'free -h && df -h'
```

## Understanding Large-Scale Benchmark Results

### Construction Performance
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

### Search Performance Scaling
```
Road Network Searches:
  100x100 network (10000 vertices):
    Dijkstra: 2.7 ms avg, 8/8 successful, avg path: 48 nodes
  200x200 network (40000 vertices):  
    Dijkstra: 12.6 ms avg, 8/8 successful, avg path: 96 nodes
  316x316 network (99856 vertices):
    Dijkstra: 35.9 ms avg, 8/8 successful, avg path: 152 nodes
```

**Analysis:**
- **Scaling factor**: How time increases with graph size
- **Success rate**: Reachability in graph type
- **Path length**: Average solution size

### Memory Scaling Analysis
```
Memory Scaling Analysis:
  50x50 (2500 vertices): 0.9 MB (378 bytes/vertex)
  100x100 (10000 vertices): 4.7 MB (491 bytes/vertex)
  200x200 (40000 vertices): 18.2 MB (477 bytes/vertex)
```

**Insights:**
- **Linear scaling**: Memory grows proportionally with vertices
- **Constant factors**: Overhead per vertex/edge
- **Cache effects**: Performance degradation with size

### Concurrent Performance
```
Concurrent Large Graph Searches:
  1 threads: 0.3s, 35 searches/sec, 10/10 successful
  2 threads: 0.3s, 67 searches/sec, 20/20 successful
  4 threads: 0.3s, 136 searches/sec, 40/40 successful
  8 threads: 0.3s, 275 searches/sec, 80/80 successful
```

**Analysis:**
- **Scaling efficiency**: Throughput increase vs thread count
- **Memory contention**: Performance degradation with large graphs
- **System limits**: Maximum practical concurrency

## Performance Optimization Strategies for Large Graphs

### 1. Memory Layout Optimization
```cpp
// Compact state representation
struct CompactState {
    int32_t x, y;  // Use smaller types
    int64_t GetId() const { return static_cast<int64_t>(y) * 100000 + x; }
};

// Use float for edge weights when precision allows
using LargeGraph = Graph<CompactState, float, DefaultIndexer<CompactState>>;
```

### 2. Algorithmic Improvements
- **Bidirectional search**: Reduce search space exponentially
- **Hierarchical pathfinding**: Pre-compute shortcuts
- **Incremental algorithms**: Reuse computation between queries

### 3. Cache-Friendly Access Patterns
- **Locality of reference**: Process spatially close vertices together
- **Memory pooling**: Reduce allocation overhead
- **Data structure layout**: Minimize pointer chasing

### 4. Parallel Processing
- **Thread-safe contexts**: Enable concurrent searches
- **Work stealing**: Balance load across threads
- **Memory-aware scheduling**: Reduce contention

## Stress Testing Scenarios

### Memory Pressure Testing
```bash
# Gradually increase graph size until system limits
for size in 50000 100000 200000 500000; do
    echo "Testing $size vertices..."
    # Monitor memory usage and performance degradation
done
```

### Long-Running Stability
```bash
# Test system stability under sustained load
timeout 1h ./bin/test_large_scale_benchmarks
```

### Concurrent Stress Testing
```bash
# Multiple benchmark processes
for i in {1..4}; do
    ./bin/test_large_scale_benchmarks &
done
wait
```

## Troubleshooting Large-Scale Tests

### Common Issues

1. **Out of Memory (OOM)**
```
# Symptoms: Process killed, system freezing
# Solutions: Reduce graph size, increase swap, use smaller data types
```

2. **Excessive Swap Usage**
```
# Check swap usage
swapon --show
free -h

# Reduce graph size or increase RAM
```

3. **Long Execution Times**
```
# Use timeouts and progress monitoring
timeout 30m ./bin/test_large_scale_benchmarks

# Consider algorithmic improvements for large graphs
```

4. **Inconsistent Results**
```
# Ensure consistent system state
echo 3 > /proc/sys/vm/drop_caches  # Clear caches
systemctl stop unnecessary-services
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

## Integration with CI/CD

### Automated Testing Strategy
```yaml
# Example CI configuration
large_scale_tests:
  runs-on: ubuntu-latest-8core
  timeout-minutes: 60
  steps:
    - name: Check available memory
      run: free -h
    - name: Run large-scale tests
      run: |
        cd build
        timeout 45m ../scripts/run_large_scale_tests.sh
    - name: Archive results
      uses: actions/upload-artifact@v2
      with:
        name: large-scale-results
        path: performance_results/
```

### Regression Detection
```bash
# Compare with baseline
../scripts/compare_performance.py \
    performance_results/baseline_large_scale.txt \
    performance_results/latest_large_scale.txt

# Alert on significant regressions (>20% slower)
```

## Expected Performance Characteristics

### Time Complexity Validation

| Operation | Expected | Large-Scale Observation |
|-----------|----------|------------------------|
| Graph Construction | O(V + E) | Linear scaling confirmed |
| Dijkstra Search | O((V + E) log V) | ~O(V^1.2) on dense graphs |
| BFS | O(V + E) | Linear with graph size |
| DFS | O(V + E) | Linear, but high constant |

### Memory Complexity

| Graph Type | Vertices | Expected Memory | Observed |
|------------|----------|----------------|----------|
| Road Network | 100K | ~40-60 MB | 42.3 MB ✓ |
| Social Network | 100K | ~50-80 MB | Varies by degree |
| Clustered | 100K | ~60-100 MB | High due to density |

This large-scale testing framework provides the foundation for understanding real-world performance characteristics and validating optimizations on production-sized graphs.