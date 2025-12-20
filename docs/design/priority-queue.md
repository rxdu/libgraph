# DynamicPriorityQueue Implementation and Design

## Overview

The `DynamicPriorityQueue` is a critical data structure in the libgraph library that enables efficient graph search algorithms like A* and Dijkstra. Unlike standard priority queues, it supports **dynamic priority updates** - the ability to change an element's priority after insertion, which is essential for optimal pathfinding.

## Why Dynamic Priority Updates Matter

In graph search algorithms:
- **Dijkstra**: When we find a shorter path to a vertex, we need to update its distance (priority)
- **A***: When we discover a better path, we need to update the f-cost (g + h)
- Without updates, we'd have duplicate vertices in the queue, leading to inefficiency and incorrect results

## Implementation Design

### Core Data Structure

```cpp
template <typename T, typename Comparator, typename ItemIndexer>
class DynamicPriorityQueue {
private:
    std::vector<T> array_;                              // Binary heap array (index 0 unused)
    std::unordered_map<int64_t, std::size_t> element_map_;  // Maps element ID to heap position
    std::size_t element_num_;                           // Current number of elements
};
```

### Binary Heap Layout

```
Position 0: Sentinel (unused, simplifies parent comparison)
Position 1: Root (min/max element)
For element at position i:
  - Parent: i/2
  - Left child: 2*i
  - Right child: 2*i + 1
```

**Example Min-Heap:**
```
Array indices: [0] [1] [2] [3] [4] [5] [6]
Array values:  [-] [2] [5] [8] [9] [7] [10]

Tree structure:
        2 (1)
       /     \
     5 (2)   8 (3)
    /   \    /
   9(4) 7(5) 10(6)
```

## Key Operations

### 1. Push (Insert)
```cpp
void Push(const T& element)
```
- **If new element**: Add to end, percolate up, update map
- **If exists**: Call Update() instead
- **Complexity**: O(log n) for new, O(log n) for update

### 2. Pop (Extract Min/Max)
```cpp
T Pop()
```
1. Save root element (min/max)
2. Remove from element_map_
3. Move last element to root
4. Percolate down to restore heap property
5. Return saved element
- **Complexity**: O(log n)

### 3. Update (Change Priority)
```cpp
void Update(const T& element)
```
1. Find element position via element_map_
2. Compare new vs old priority
3. If decreased: percolate up
4. If increased: percolate down
- **Complexity**: O(log n) instead of O(n) linear search

### 4. Contains (Check Existence)
```cpp
bool Contains(const T& element)
```
- Direct lookup in element_map_
- **Complexity**: O(1) average case

## Critical Bug Fixes (Aug 2025)

### Bug 1: Memory Leak in DeleteMin()

**Before (Buggy):**
```cpp
void DeleteMin() {
    if (Empty()) return;
    array_[1] = std::move(array_[element_num_--]);
    PercolateDown(1);
    // BUG: Never removed array_[1] from element_map_!
}
```

**After (Fixed):**
```cpp
void DeleteMin() {
    if (Empty()) return;
    
    // Remove the min element from map
    element_map_.erase(GetItemIndex(array_[1]));
    
    if (element_num_ > 1) {
        array_[1] = std::move(array_[element_num_]);
        element_map_[GetItemIndex(array_[1])] = 1;
    }
    element_num_--;
    
    if (element_num_ > 0) {
        PercolateDown(1);
    }
}
```

**Impact:**
- **Memory**: Prevented unbounded growth of element_map_
- **Correctness**: Contains() now correctly returns false for popped elements
- **Performance**: Avoided map bloat that would slow down lookups

### Bug 2: Missing Map Updates in PercolateUp()

**Before (Buggy):**
```cpp
void PercolateUp(const T& element, std::size_t index) {
    for (; Compare(element, array_[index / 2]); index /= 2) {
        array_[index] = std::move(array_[index / 2]);
        // BUG: Never updated element_map_ for moved elements!
    }
    array_[index] = element;
    element_map_[GetItemIndex(element)] = index;  // Only final position
}
```

**After (Fixed):**
```cpp
void PercolateUp(const T& element, std::size_t index) {
    array_[0] = element;  // Sentinel for cleaner loop
    
    while (index > 1 && Compare(element, array_[index / 2])) {
        array_[index] = std::move(array_[index / 2]);
        element_map_[GetItemIndex(array_[index])] = index;  // Update each move
        index /= 2;
    }
    
    array_[index] = element;
    element_map_[GetItemIndex(element)] = index;
}
```

**Impact:**
- **Correctness**: Update() now finds elements at correct positions
- **Search Algorithms**: A* and Dijkstra can properly update vertex priorities

### Bug 3: Missing Map Updates in PercolateDown()

**Before (Buggy):**
```cpp
void PercolateDown(std::size_t index) {
    // ... moving elements down
    array_[index] = std::move(array_[child]);
    // BUG: element_map_ not updated for moved elements
}
```

**After (Fixed):**
```cpp
void PercolateDown(std::size_t index) {
    T tmp = std::move(array_[index]);
    
    while (index * 2 <= element_num_) {
        // ... find smaller child
        if (Compare(array_[child], tmp)) {
            array_[index] = std::move(array_[child]);
            element_map_[GetItemIndex(array_[index])] = index;  // Update map
            index = child;
        } else {
            break;
        }
    }
    
    array_[index] = std::move(tmp);
    element_map_[GetItemIndex(array_[index])] = index;  // Final position
}
```

## Impact on Search Algorithms

### Without These Fixes

1. **Incorrect Path Costs**: 
   - Update() might modify wrong vertex due to stale map
   - Could lead to suboptimal or incorrect paths

2. **Algorithm Failures**:
   - Contains() returning true for already-processed vertices
   - Infinite loops in worst case

3. **Memory Issues**:
   - Continuous memory growth in long-running searches
   - Performance degradation over time

### With These Fixes

1. **Correct Optimal Paths**:
   - Dijkstra guarantees shortest path
   - A* guarantees optimal path with admissible heuristic

2. **Predictable Performance**:
   - Consistent O(log n) operations
   - No memory leaks

3. **Thread Safety Ready**:
   - Clean state management enables SearchContext usage
   - Multiple concurrent searches possible

## Usage Example

```cpp
// Custom element with ID for indexing
struct Vertex {
    int64_t id;
    double cost;
    
    int64_t GetId() const { return id; }
};

// Comparator for min-heap based on cost
struct VertexCompare {
    bool operator()(const Vertex& a, const Vertex& b) const {
        return a.cost < b.cost;  // Min-heap
    }
};

// Usage in Dijkstra-like algorithm
DynamicPriorityQueue<Vertex, VertexCompare> pq;

// Initial vertices
pq.Push(Vertex{1, 0.0});    // Start vertex
pq.Push(Vertex{2, INF});
pq.Push(Vertex{3, INF});

// Process vertices
while (!pq.Empty()) {
    Vertex current = pq.Pop();
    
    // Process neighbors
    for (auto& neighbor : GetNeighbors(current)) {
        double new_cost = current.cost + edge_weight;
        
        if (new_cost < neighbor.cost) {
            neighbor.cost = new_cost;
            pq.Update(neighbor);  // Dynamic update!
        }
    }
}
```

## Performance Characteristics

| Operation | Time Complexity | Space Complexity |
|-----------|----------------|------------------|
| Push (new) | O(log n) | O(1) amortized |
| Push (update) | O(log n) | O(1) |
| Pop | O(log n) | O(1) |
| Update | O(log n) | O(1) |
| Contains | O(1) average | O(1) |
| Peek | O(1) | O(1) |

**Space Usage**: O(n) for heap array + O(n) for element map = O(n) total

## Testing and Validation

The implementation includes comprehensive tests:

1. **Basic Operations**: Push, Pop, Peek, Contains
2. **Heap Property**: Maintains min/max ordering
3. **Update Correctness**: Elements move to correct positions
4. **Map Consistency**: element_map_ always synchronized with array_
5. **Stress Testing**: 1000+ operations with random priorities
6. **Integration**: Works correctly with A* and Dijkstra

## Design Trade-offs

### Why Not std::priority_queue?
- No update operation
- No contains check
- Would require delete + re-insert (inefficient)

### Why Maintain element_map_?
- **Pro**: O(1) contains check, O(log n) updates
- **Con**: Extra O(n) memory
- **Verdict**: Essential for graph algorithms

### Why Index 0 Sentinel?
- Simplifies parent comparison: `Compare(element, array_[index/2])`
- No special case for root
- Minor memory waste (1 element)

## Conclusion

The DynamicPriorityQueue is a carefully designed data structure that enables efficient graph search algorithms. The recent bug fixes ensure:

1. **Correctness**: Proper maintenance of the element-position mapping
2. **Efficiency**: No memory leaks or performance degradation
3. **Reliability**: Search algorithms produce optimal paths

This implementation represents a production-ready priority queue suitable for real-world graph applications, robotics path planning, and network routing algorithms.