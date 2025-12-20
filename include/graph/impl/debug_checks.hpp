/*
 * debug_checks.hpp
 *
 * Created on: Dec 2025
 * Description: Debug-only assertions for cost validation
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 *
 * These checks are compiled away in release mode (when NDEBUG is defined).
 * They help catch common errors during development:
 * - Negative edge costs (invalid for Dijkstra/A*)
 * - NaN or Inf costs (undefined behavior in comparisons)
 * - Negative heuristic values (violates non-negativity assumption)
 * - Inadmissible heuristics (optional, can be checked separately)
 */

#ifndef DEBUG_CHECKS_HPP
#define DEBUG_CHECKS_HPP

#include <cassert>
#include <cmath>
#include <type_traits>
#include <limits>

namespace xmotion {
namespace debug {

/**
 * @brief Check if a floating-point value is finite (not NaN or Inf)
 *
 * Only performs the check for floating-point types.
 * For integral types, always returns true.
 */
template <typename T>
inline typename std::enable_if<std::is_floating_point<T>::value, bool>::type
IsFinite(T value) noexcept {
    return std::isfinite(value);
}

template <typename T>
inline typename std::enable_if<!std::is_floating_point<T>::value, bool>::type
IsFinite(T /*value*/) noexcept {
    return true;  // Non-floating-point types are always "finite"
}

/**
 * @brief Check if a value is non-negative
 *
 * For unsigned types, always returns true.
 */
template <typename T>
inline typename std::enable_if<std::is_unsigned<T>::value, bool>::type
IsNonNegative(T /*value*/) noexcept {
    return true;  // Unsigned types are always non-negative
}

template <typename T>
inline typename std::enable_if<std::is_signed<T>::value && std::is_arithmetic<T>::value, bool>::type
IsNonNegative(T value) noexcept {
    return value >= T{0};
}

template <typename T>
inline typename std::enable_if<!std::is_arithmetic<T>::value, bool>::type
IsNonNegative(T value) noexcept {
    // For custom cost types, assume non-negative if they have a default constructor
    // that represents zero and support comparison
    return !(value < T{});
}

/**
 * @brief Validate an edge cost in debug mode
 *
 * Asserts that:
 * 1. Cost is finite (not NaN or Inf for floating-point types)
 * 2. Cost is non-negative (required for Dijkstra/A*)
 *
 * These checks are compiled away when NDEBUG is defined.
 */
template <typename CostType>
inline typename std::enable_if<std::is_arithmetic<CostType>::value>::type
AssertValidEdgeCost(CostType cost, const char* /*context*/ = nullptr) noexcept {
#ifndef NDEBUG
    assert(IsFinite(cost) && "Edge cost must be finite (not NaN or Inf)");
    assert(IsNonNegative(cost) && "Edge cost must be non-negative for Dijkstra/A*");
#else
    (void)cost;
#endif
}

template <typename CostType>
inline typename std::enable_if<!std::is_arithmetic<CostType>::value>::type
AssertValidEdgeCost(CostType /*cost*/, const char* /*context*/ = nullptr) noexcept {
    // No validation for non-arithmetic types
}

/**
 * @brief Validate a heuristic value in debug mode
 *
 * Asserts that:
 * 1. Heuristic is finite (not NaN or Inf for floating-point types)
 * 2. Heuristic is non-negative (required property)
 *
 * Note: This does NOT check admissibility (h(n) <= actual cost to goal)
 * as that requires knowledge of the true cost.
 */
template <typename CostType>
inline typename std::enable_if<std::is_arithmetic<CostType>::value>::type
AssertValidHeuristic(CostType heuristic, const char* /*context*/ = nullptr) noexcept {
#ifndef NDEBUG
    assert(IsFinite(heuristic) && "Heuristic must be finite (not NaN or Inf)");
    assert(IsNonNegative(heuristic) && "Heuristic must be non-negative");
#else
    (void)heuristic;
#endif
}

template <typename CostType>
inline typename std::enable_if<!std::is_arithmetic<CostType>::value>::type
AssertValidHeuristic(CostType /*heuristic*/, const char* /*context*/ = nullptr) noexcept {
    // No validation for non-arithmetic types
}

/**
 * @brief Validate a g-cost (path cost from start) in debug mode
 */
template <typename CostType>
inline typename std::enable_if<std::is_arithmetic<CostType>::value>::type
AssertValidGCost(CostType g_cost, const char* /*context*/ = nullptr) noexcept {
#ifndef NDEBUG
    assert(IsFinite(g_cost) && "G-cost must be finite (not NaN or Inf)");
    assert(IsNonNegative(g_cost) && "G-cost must be non-negative");
#else
    (void)g_cost;
#endif
}

template <typename CostType>
inline typename std::enable_if<!std::is_arithmetic<CostType>::value>::type
AssertValidGCost(CostType /*g_cost*/, const char* /*context*/ = nullptr) noexcept {
    // No validation for non-arithmetic types
}

/**
 * @brief Validate that a new g-cost is not worse than infinity
 *
 * This catches cases where costs might overflow or accumulate incorrectly.
 */
template <typename CostType>
inline typename std::enable_if<std::is_floating_point<CostType>::value>::type
AssertCostNotOverflowed(CostType cost, const char* /*context*/ = nullptr) noexcept {
#ifndef NDEBUG
    assert(!std::isinf(cost) && "Cost has overflowed to infinity");
#else
    (void)cost;
#endif
}

template <typename CostType>
inline typename std::enable_if<std::is_integral<CostType>::value>::type
AssertCostNotOverflowed(CostType cost, const char* /*context*/ = nullptr) noexcept {
#ifndef NDEBUG
    // For integral types, check against max value (potential overflow indicator)
    assert(cost < std::numeric_limits<CostType>::max() &&
           "Cost may have overflowed (at max value)");
#else
    (void)cost;
#endif
}

template <typename CostType>
inline typename std::enable_if<!std::is_arithmetic<CostType>::value>::type
AssertCostNotOverflowed(CostType /*cost*/, const char* /*context*/ = nullptr) noexcept {
    // No validation for non-arithmetic types
}

}  // namespace debug
}  // namespace xmotion

#endif /* DEBUG_CHECKS_HPP */
