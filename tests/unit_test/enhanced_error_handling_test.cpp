/*
 * enhanced_error_handling_test.cpp
 *
 * Test enhanced error handling with custom exception types
 */

#include <iostream>
#include <stdexcept>

#include "gtest/gtest.h"

#include "graph/graph.hpp"
#include "graph/exceptions.hpp"
#include "graph/search/dijkstra.hpp"

using namespace xmotion;

struct ErrorTestState {
    int64_t id;
    ErrorTestState(int64_t id_) : id(id_) {}
    int64_t GetId() const { return id; }
};

class EnhancedErrorHandlingTest : public ::testing::Test {
protected:
    Graph<ErrorTestState, double> graph;
    
    void SetUp() override {
        // Create a simple test graph
        graph.AddVertex(ErrorTestState(1));
        graph.AddVertex(ErrorTestState(2));
        graph.AddEdge(ErrorTestState(1), ErrorTestState(2), 1.0);
    }
};

TEST_F(EnhancedErrorHandlingTest, ElementNotFoundError) {
    // Test GetVertexSafe with non-existent vertex
    EXPECT_THROW({
        graph.GetVertexSafe(999);
    }, ElementNotFoundError);
    
    try {
        graph.GetVertexSafe(999);
        FAIL() << "Expected ElementNotFoundError";
    } catch (const ElementNotFoundError& e) {
        EXPECT_EQ(e.GetElementId(), 999);
        EXPECT_EQ(e.GetElementType(), "Vertex");
        EXPECT_NE(std::string(e.what()).find("Vertex with ID 999 not found"), std::string::npos);
    }
}

TEST_F(EnhancedErrorHandlingTest, InvalidArgumentError) {
    // Test that InvalidArgumentError can be thrown and caught
    EXPECT_THROW({
        throw InvalidArgumentError("Test invalid argument");
    }, InvalidArgumentError);
    
    try {
        throw InvalidArgumentError("Test invalid argument");
        FAIL() << "Expected InvalidArgumentError";
    } catch (const InvalidArgumentError& e) {
        EXPECT_NE(std::string(e.what()).find("Invalid Argument - Test invalid argument"), std::string::npos);
    }
}

TEST_F(EnhancedErrorHandlingTest, ValidateEdgeWeightNaN) {
    // Test edge weight validation with NaN
    EXPECT_THROW({
        graph.ValidateEdgeWeight(std::numeric_limits<double>::quiet_NaN());
    }, InvalidArgumentError);
    
    try {
        graph.ValidateEdgeWeight(std::numeric_limits<double>::quiet_NaN());
        FAIL() << "Expected InvalidArgumentError for NaN";
    } catch (const InvalidArgumentError& e) {
        EXPECT_NE(std::string(e.what()).find("Edge weight cannot be NaN"), std::string::npos);
    }
}

TEST_F(EnhancedErrorHandlingTest, ValidateEdgeWeightInfinity) {
    // Test edge weight validation with infinity
    EXPECT_THROW({
        graph.ValidateEdgeWeight(std::numeric_limits<double>::infinity());
    }, InvalidArgumentError);
    
    try {
        graph.ValidateEdgeWeight(std::numeric_limits<double>::infinity());
        FAIL() << "Expected InvalidArgumentError for infinity";
    } catch (const InvalidArgumentError& e) {
        EXPECT_NE(std::string(e.what()).find("Edge weight cannot be infinite"), std::string::npos);
    }
}

TEST_F(EnhancedErrorHandlingTest, ValidateStructureSuccess) {
    // Test successful structure validation
    EXPECT_NO_THROW({
        graph.ValidateStructure();
    });
}

TEST_F(EnhancedErrorHandlingTest, GraphExceptionHierarchy) {
    // Test that custom exceptions derive from GraphException
    try {
        graph.GetVertexSafe(999);
        FAIL() << "Expected exception";
    } catch (const GraphException& e) {
        // Should catch ElementNotFoundError as GraphException
        EXPECT_NE(std::string(e.what()).find("Graph Error:"), std::string::npos);
    }
    
    try {
        graph.ValidateEdgeWeight(std::numeric_limits<double>::quiet_NaN());
        FAIL() << "Expected exception";
    } catch (const GraphException& e) {
        // Should catch InvalidArgumentError as GraphException
        EXPECT_NE(std::string(e.what()).find("Graph Error:"), std::string::npos);
    }
}

TEST_F(EnhancedErrorHandlingTest, ExceptionDetails) {
    // Test that exceptions contain helpful details
    try {
        throw StructureViolationError("tree property", "Adding edge would create cycle");
        FAIL() << "Expected StructureViolationError";
    } catch (const StructureViolationError& e) {
        EXPECT_EQ(e.GetConstraint(), "tree property");
        EXPECT_NE(std::string(e.what()).find("Structure violation (tree property): Adding edge would create cycle"), std::string::npos);
    }
    
    try {
        throw SearchError("Dijkstra", "Invalid heuristic function");
        FAIL() << "Expected SearchError";
    } catch (const SearchError& e) {
        EXPECT_EQ(e.GetAlgorithm(), "Dijkstra");
        EXPECT_NE(std::string(e.what()).find("Search error in Dijkstra: Invalid heuristic function"), std::string::npos);
    }
}

TEST_F(EnhancedErrorHandlingTest, MemoryErrorDetails) {
    // Test MemoryError with size information
    try {
        throw MemoryError("Allocation failed", 1024);
        FAIL() << "Expected MemoryError";
    } catch (const MemoryError& e) {
        EXPECT_EQ(e.GetRequestedSize(), 1024);
        EXPECT_NE(std::string(e.what()).find("requested: 1024 bytes"), std::string::npos);
    }
}

TEST_F(EnhancedErrorHandlingTest, UnsupportedOperationError) {
    // Test UnsupportedOperationError
    try {
        throw UnsupportedOperationError("concurrent writes", "Not supported in current implementation");
        FAIL() << "Expected UnsupportedOperationError";
    } catch (const UnsupportedOperationError& e) {
        EXPECT_EQ(e.GetOperation(), "concurrent writes");
        EXPECT_NE(std::string(e.what()).find("Unsupported operation 'concurrent writes': Not supported in current implementation"), std::string::npos);
    }
}