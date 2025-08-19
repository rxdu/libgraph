#!/bin/bash

# Unified Performance Benchmark Runner
# Outputs comprehensive results to a single text file

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
BUILD_DIR="$PROJECT_ROOT/build"
RESULTS_DIR="$PROJECT_ROOT/performance_results"

# Create results directory if it doesn't exist
mkdir -p "$RESULTS_DIR"

# Get current timestamp for result files
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
UNIFIED_RESULTS_FILE="$RESULTS_DIR/unified_benchmark_results_$TIMESTAMP.txt"

echo "Running Unified Performance Benchmark Suite"
echo "============================================"
echo ""

# Check if build directory exists
if [[ ! -d "$BUILD_DIR" ]]; then
    echo "Error: Build directory not found at $BUILD_DIR"
    echo "Please run 'mkdir build && cd build && cmake .. && make' first"
    exit 1
fi

# Check if unified benchmark executable exists
UNIFIED_EXE="$BUILD_DIR/bin/test_unified_benchmarks"
if [[ ! -f "$UNIFIED_EXE" ]]; then
    echo "Building unified benchmarks..."
    cd "$BUILD_DIR"
    make test_unified_benchmarks
    if [[ $? -ne 0 ]]; then
        echo "Error: Failed to build unified benchmarks"
        exit 1
    fi
    echo "✓ Build completed successfully"
    echo ""
fi

# Check available memory
AVAILABLE_MEMORY_KB=$(awk '/MemAvailable/ {print $2}' /proc/meminfo 2>/dev/null || echo "0")
AVAILABLE_MEMORY_MB=$((AVAILABLE_MEMORY_KB / 1024))

echo "System Information:"
echo "-------------------"
echo "Available memory: ${AVAILABLE_MEMORY_MB} MB"
echo "CPU cores: $(nproc)"
echo "Build directory: $BUILD_DIR"
echo "Results file: $UNIFIED_RESULTS_FILE"
echo ""

if [[ $AVAILABLE_MEMORY_MB -lt 2048 ]]; then
    echo "WARNING: Less than 2GB memory available."
    echo "Some large-scale tests may fail or run slowly."
    echo ""
fi

# Run the unified benchmark and save to single file
echo "Running comprehensive benchmarks..."
echo "This combines micro-benchmarks and large-scale tests in one report."
echo "Estimated time: 2-5 minutes depending on your system."
echo ""

cd "$BUILD_DIR"

# Set timeout for safety (10 minutes)
TIMEOUT="600"

if timeout "$TIMEOUT" "$UNIFIED_EXE" > "$UNIFIED_RESULTS_FILE" 2>&1; then
    echo "✓ Unified benchmarks completed successfully!"
    echo ""
    echo "Results Summary:"
    echo "=================="
    
    # Extract key metrics for quick overview
    echo ""
    echo "Graph Construction Performance:"
    grep -A 1 "Construction time:" "$UNIFIED_RESULTS_FILE" | head -6 | sed 's/^/  /'
    
    echo ""
    echo "Search Performance (100K vertices):"
    grep -A 3 "316x316.*vertices" "$UNIFIED_RESULTS_FILE" | sed 's/^/  /'
    
    echo ""
    echo "Memory Efficiency:"
    grep "bytes/vertex" "$UNIFIED_RESULTS_FILE" | head -3 | sed 's/^/  /'
    
    echo ""
    echo "Concurrent Scaling:"
    grep "threads:.*searches/sec" "$UNIFIED_RESULTS_FILE" | head -4 | sed 's/^/  /'
    
    echo ""
    echo "=================================================================================
Full detailed report saved to:
  $UNIFIED_RESULTS_FILE

File size: $(stat -c%s "$UNIFIED_RESULTS_FILE" | numfmt --to=iec)

This single file contains:
  ✓ Micro-benchmarks (edge lookup, vertex removal, context operations)
  ✓ Large-scale benchmarks (realistic graph sizes and workloads)  
  ✓ Memory scaling analysis (usage patterns by graph size)
  ✓ Concurrent performance analysis (threading scalability)
  ✓ Optimization recommendations with expected improvements

Usage:
  # View full report
  cat $UNIFIED_RESULTS_FILE

  # View specific sections
  grep -A 20 'SECTION 1: MICRO-BENCHMARKS' $UNIFIED_RESULTS_FILE
  grep -A 20 'SECTION 2: LARGE-SCALE BENCHMARKS' $UNIFIED_RESULTS_FILE
  grep -A 20 'SECTION 3: SUMMARY' $UNIFIED_RESULTS_FILE

  # Compare with future optimizations
  # 1. Save this file as your baseline
  # 2. Implement optimizations
  # 3. Run this script again
  # 4. Use diff or comparison tools on the result files

Performance Optimization Targets (from report):
  📊 Edge Lookup: Current O(n) → Target O(1) hash-based
  🗑️ Vertex Removal: Current O(m²) → Target O(m) bidirectional refs
  💾 Memory Pooling: Reduce allocation overhead by 20-50%
  🔄 Context Reuse: Systematic reuse patterns for 30-70% improvement
=================================================================================="

else
    EXIT_CODE=$?
    echo "✗ Unified benchmark failed or timed out!"
    echo ""
    
    if [[ $EXIT_CODE -eq 124 ]]; then
        echo "Benchmark timed out after $TIMEOUT seconds."
        echo "This might indicate:"
        echo "  - Insufficient memory for large-scale tests"
        echo "  - System under high load"
        echo "  - Need to reduce test scope"
    else
        echo "Benchmark failed with exit code: $EXIT_CODE"
    fi
    
    echo ""
    echo "Partial results (if any) saved to: $UNIFIED_RESULTS_FILE"
    echo ""
    echo "Troubleshooting:"
    echo "  - Ensure at least 2GB available memory"
    echo "  - Close other applications to free resources"
    echo "  - Try running with smaller graph sizes"
    echo "  - Check system logs for memory issues"
    
    exit 1
fi