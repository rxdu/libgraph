# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [3.1.0] - 2025-12-21

### Added
- **PathResult struct**: Rich search results with `total_cost`, `nodes_expanded`, and `found` flag
- **SearchLimits struct**: Early termination via `max_expansions` or `timeout_ms`
- **Multi-goal search**: `Dijkstra::SearchMultiGoal()` and `AStar::SearchMultiGoal()` find path to nearest goal
- **Nodes expanded counter**: `context.GetNodesExpanded()` for search diagnostics
- **Debug cost assertions**: Validate non-negative, finite costs in debug builds (`debug_checks.hpp`)
- **Production readiness documentation**: `docs/PRODUCTION_READINESS.md`
- **14 new unit tests** for production features (317 total tests)

### Changed
- **Deterministic tie-breaking**: Priority queue comparators now break ties by `vertex_id` for reproducible results

### Fixed
- **Iterator invalidation**: `Edge` and `Vertex::vertices_from` now use stable `Vertex*` pointers instead of iterators that could invalidate on `unordered_map` rehash

## [3.0.1] - 2025-08-19

### Fixed
- Codecov configuration updates

## [3.0.0] - 2025-08-19

### Added
- Unified search framework with CRTP strategy pattern
- Thread-safe concurrent searches via external `SearchContext`
- Generic cost type support with `CostTraits`
- `DynamicPriorityQueue` for efficient priority updates
- Comprehensive exception handling with custom exception types
- Structure validation for graphs and trees
- Extensive documentation and tutorials

### Changed
- Major API redesign for thread safety and generic costs
- Search algorithms now accept `SearchContext&` for thread-safe operation
- Improved memory management with RAII patterns

## [2.x] - Previous Releases

See git history for earlier changes.
