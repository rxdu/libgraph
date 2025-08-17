/*
 * exceptions.hpp
 *
 * Custom exception hierarchy for libgraph
 * Provides detailed error information for better debugging and error handling
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#ifndef GRAPH_EXCEPTIONS_HPP
#define GRAPH_EXCEPTIONS_HPP

#include <stdexcept>
#include <string>
#include <cstdint>

namespace xmotion {

/**
 * @brief Base exception class for all graph-related errors
 * 
 * This provides a common base for all graph library exceptions,
 * allowing users to catch all graph errors with a single catch block.
 */
class GraphException : public std::runtime_error {
public:
    explicit GraphException(const std::string& message) 
        : std::runtime_error("Graph Error: " + message) {}
        
    explicit GraphException(const char* message)
        : std::runtime_error(std::string("Graph Error: ") + message) {}
};

/**
 * @brief Exception thrown when invalid arguments are passed to graph operations
 * 
 * Examples:
 * - Null graph pointer to search algorithms
 * - Invalid vertex IDs
 * - Negative edge weights where not allowed
 */
class InvalidArgumentError : public GraphException {
public:
    explicit InvalidArgumentError(const std::string& message)
        : GraphException("Invalid Argument - " + message) {}
        
    explicit InvalidArgumentError(const char* message)
        : GraphException(std::string("Invalid Argument - ") + message) {}
};

/**
 * @brief Exception thrown when attempting operations on non-existent vertices or edges
 * 
 * Examples:
 * - Accessing vertex that doesn't exist in graph
 * - Removing edge that doesn't exist
 * - Path reconstruction from unreachable vertex
 */
class ElementNotFoundError : public GraphException {
private:
    int64_t element_id_;
    std::string element_type_;
    
public:
    ElementNotFoundError(const std::string& element_type, int64_t element_id)
        : GraphException(element_type + " with ID " + std::to_string(element_id) + " not found"),
          element_id_(element_id), element_type_(element_type) {}
          
    ElementNotFoundError(const std::string& element_type, const std::string& message)
        : GraphException(element_type + " not found: " + message),
          element_id_(-1), element_type_(element_type) {}
    
    int64_t GetElementId() const noexcept { return element_id_; }
    const std::string& GetElementType() const noexcept { return element_type_; }
};

/**
 * @brief Exception thrown when graph structure constraints are violated
 * 
 * Examples:
 * - Adding edge that would create cycle in tree
 * - Tree operations that violate tree properties
 * - Graph modifications that break class invariants
 */
class StructureViolationError : public GraphException {
private:
    std::string constraint_;
    
public:
    explicit StructureViolationError(const std::string& constraint, const std::string& message)
        : GraphException("Structure violation (" + constraint + "): " + message),
          constraint_(constraint) {}
          
    const std::string& GetConstraint() const noexcept { return constraint_; }
};

/**
 * @brief Exception thrown when search algorithms encounter invalid conditions
 * 
 * Examples:
 * - Invalid heuristic function in A*
 * - Search context corruption
 * - Algorithm-specific constraint violations
 */
class SearchError : public GraphException {
private:
    std::string algorithm_;
    
public:
    SearchError(const std::string& algorithm, const std::string& message)
        : GraphException("Search error in " + algorithm + ": " + message),
          algorithm_(algorithm) {}
          
    const std::string& GetAlgorithm() const noexcept { return algorithm_; }
};

/**
 * @brief Exception thrown when graph operations would cause out-of-memory conditions
 * 
 * Examples:
 * - Graph too large for available memory
 * - Search context allocation failure
 * - Priority queue memory exhaustion
 */
class MemoryError : public GraphException {
private:
    size_t requested_size_;
    
public:
    explicit MemoryError(const std::string& message)
        : GraphException("Memory error: " + message), requested_size_(0) {}
        
    MemoryError(const std::string& message, size_t requested_size)
        : GraphException("Memory error: " + message + " (requested: " + 
                        std::to_string(requested_size) + " bytes)"),
          requested_size_(requested_size) {}
          
    size_t GetRequestedSize() const noexcept { return requested_size_; }
};

/**
 * @brief Exception thrown when attempting unsupported operations
 * 
 * Examples:
 * - Concurrent write operations on thread-safe contexts
 * - Operations not supported in current configuration
 * - Feature not yet implemented
 */
class UnsupportedOperationError : public GraphException {
private:
    std::string operation_;
    
public:
    explicit UnsupportedOperationError(const std::string& operation)
        : GraphException("Unsupported operation: " + operation),
          operation_(operation) {}
          
    UnsupportedOperationError(const std::string& operation, const std::string& reason)
        : GraphException("Unsupported operation '" + operation + "': " + reason),
          operation_(operation) {}
          
    const std::string& GetOperation() const noexcept { return operation_; }
};

/**
 * @brief Exception thrown when graph data is corrupted or inconsistent
 * 
 * Examples:
 * - Corrupted internal data structures
 * - Inconsistent vertex/edge relationships
 * - Failed data integrity checks
 */
class DataCorruptionError : public GraphException {
private:
    std::string corruption_type_;
    
public:
    explicit DataCorruptionError(const std::string& corruption_type)
        : GraphException("Data corruption detected: " + corruption_type),
          corruption_type_(corruption_type) {}
          
    DataCorruptionError(const std::string& corruption_type, const std::string& details)
        : GraphException("Data corruption (" + corruption_type + "): " + details),
          corruption_type_(corruption_type) {}
          
    const std::string& GetCorruptionType() const noexcept { return corruption_type_; }
};

} // namespace xmotion

#endif /* GRAPH_EXCEPTIONS_HPP */