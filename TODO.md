# TODO List

- [] A* and Dijkstra algorithms currently assume double type cost. Generic type cost with proper comparator defined should also be allowed.
- [] Refactor iterators and fix const_iterator for Vertex and Edge
- [] Update edges_to and vertices_from data structure for higher efficiency removal
- [*] Default indexer doesn't work if State is a std::shared_ptr<T> type
- [*] Dynamic priority queue
- [*] Improve unit test coverage
- [*] Issue: state type cannot be std::shared_ptr<T>
- [*] Convenience functions to access vertex information
- [*] Implement iterators for vertex and edge to unify the accessing interface
- [*] Update unit tests for basic function test