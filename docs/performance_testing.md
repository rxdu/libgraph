# Performance Testing Guide

This document explains how to use the performance testing framework to quantitatively evaluate optimization improvements.

## Overview

The performance testing suite measures baseline performance for the key bottlenecks identified in the TODO.md:

1. **Edge Lookup Performance** - Measures O(n) linear search times 
2. **Vertex Removal Performance** - Measures O(m²) removal complexity
3. **Search Context Performance** - Measures allocation/context reuse overhead
4. **Concurrent Search Performance** - Measures threading scalability

## Quick Start

### 1. Run Baseline Measurements

```bash
cd build
../scripts/run_performance_tests.sh
```

This will:
- Build the performance benchmarks if needed
- Collect system information
- Run comprehensive benchmarks  
- Save timestamped results to `performance_results/`

### 2. Implement Optimizations

Make your performance improvements to the codebase.

### 3. Run Performance Tests Again

```bash
../scripts/run_performance_tests.sh
```

### 4. Compare Results

```bash
# Automatic comparison with detailed analysis
../scripts/compare_performance.py baseline_old.txt baseline_new.txt

# Manual comparison
diff -u baseline_old.txt baseline_new.txt
```

## Benchmark Categories

### Edge Lookup Benchmarks

**What it measures**: Time to find edges from vertices using current O(n) linear search
**Scenarios tested**:
- Sparse graphs (10% edge density)
- Medium density (50% edge density) 
- Dense graphs (90% edge density)

**Optimization target**: Replace with O(1) hash-based lookup

### Vertex Removal Benchmarks  

**What it measures**: Time to remove vertices with all incoming/outgoing edges
**Scenarios tested**:
- Star graphs (worst case - central vertex connected to all others)
- Dense grid graphs (typical case)
- Different graph sizes (50, 100, 200+ vertices)

**Optimization target**: Reduce from O(m²) to O(m) complexity

### Search Context Benchmarks

**What it measures**: Memory allocation overhead and context reuse benefits
**Scenarios tested**:
- New context creation per search
- Context reuse across searches
- Memory allocation scaling with graph size

**Optimization target**: Memory pooling and context reuse patterns

### Concurrent Search Benchmarks

**What it measures**: Throughput scaling with multiple threads
**Scenarios tested**:
- 1, 2, 4, 8 concurrent threads
- Realistic search workloads
- Thread safety validation

**Optimization target**: Better concurrent performance patterns

## Interpreting Results

### Key Metrics to Track

- **Edge Lookups**: μs/lookup (lower is better)
- **Vertex Removal**: ms per removal (lower is better)
- **Context Creation**: ms/search (lower is better) 
- **Concurrent Throughput**: searches/sec (higher is better)

### Expected Improvements

| Optimization | Metric | Expected Improvement |
|-------------|--------|---------------------|
| Hash-based edge lookup | Edge Lookups | 10-100x faster |
| Better vertex removal | Vertex Removal | 2-10x faster |
| Memory pooling | Context Creation | 20-50% faster |
| Context reuse | Context Reuse | 30-70% faster |

## Performance Testing Best Practices

### 1. Consistent Environment

- Run tests on same machine with same load
- Use Release build mode for accurate measurements
- Close unnecessary applications
- Run multiple times and average results

### 2. Meaningful Workloads

The benchmarks use realistic graph structures:
- Grid graphs (common in pathfinding)
- Random graphs (general graph algorithms)
- Star graphs (worst-case scenarios)

### 3. Statistical Significance

- Each benchmark runs 100-1000 iterations
- Results are averaged for stability
- Fixed random seeds ensure reproducibility

## Adding New Benchmarks

To add benchmarks for new optimizations:

1. Add test category to `test_performance_benchmarks.cpp`
2. Update `compare_performance.py` parsing patterns
3. Document expected improvements

Example structure:
```cpp
class NewOptimizationBenchmark {
public:
    static void RunBenchmarks() {
        // Test different scenarios
        // Measure performance with PerformanceTimer
        // Output in consistent format
    }
};
```

## Automated Performance Tracking

The framework is designed for CI/CD integration:

- Deterministic results (fixed random seeds)
- Machine-readable output formats  
- Regression detection capabilities
- Historical trend tracking

## Files Overview

- `test_performance_benchmarks.cpp` - Main benchmark implementation
- `run_performance_tests.sh` - Test runner script
- `compare_performance.py` - Result comparison tool
- `performance_results/` - Timestamped results directory
- `system_info_*.txt` - System configuration snapshots
- `baseline_*.txt` - Benchmark results

This framework provides the foundation for quantitative performance evaluation and ensures optimizations deliver measurable improvements.