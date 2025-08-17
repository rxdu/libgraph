/*
 * attributes.hpp
 *
 * Created on: Aug 2025
 * Description: Attribute storage system for vertices and edges
 *
 * Copyright (c) 2015-2025 Ruixiang Du (rdu)
 */

#ifndef GRAPH_ATTRIBUTES_HPP
#define GRAPH_ATTRIBUTES_HPP

#include <unordered_map>
#include <string>
#include <memory>
#include <typeinfo>
#include <stdexcept>

namespace xmotion {

/**
 * @brief Type-erased attribute storage for graph elements
 * 
 * Provides a flexible attribute system that works with C++11.
 * Stores arbitrary typed values associated with string keys.
 */
class AttributeMap {
private:
    // Base class for type-erased storage
    struct AttributeBase {
        virtual ~AttributeBase() = default;
        virtual const std::type_info& type() const = 0;
        virtual std::unique_ptr<AttributeBase> clone() const = 0;
    };
    
    // Typed attribute storage
    template<typename T>
    struct AttributeHolder : AttributeBase {
        T value;
        
        explicit AttributeHolder(const T& v) : value(v) {}
        explicit AttributeHolder(T&& v) : value(std::move(v)) {}
        
        const std::type_info& type() const override {
            return typeid(T);
        }
        
        std::unique_ptr<AttributeBase> clone() const override {
            return std::unique_ptr<AttributeBase>(new AttributeHolder<T>(value));
        }
    };
    
    std::unordered_map<std::string, std::unique_ptr<AttributeBase>> attributes_;
    
public:
    AttributeMap() = default;
    
    // Deep copy constructor
    AttributeMap(const AttributeMap& other) {
        for (const auto& pair : other.attributes_) {
            attributes_[pair.first] = pair.second->clone();
        }
    }
    
    // Copy assignment
    AttributeMap& operator=(const AttributeMap& other) {
        if (this != &other) {
            attributes_.clear();
            for (const auto& pair : other.attributes_) {
                attributes_[pair.first] = pair.second->clone();
            }
        }
        return *this;
    }
    
    // Move operations
    AttributeMap(AttributeMap&&) = default;
    AttributeMap& operator=(AttributeMap&&) = default;
    
    /**
     * @brief Set an attribute value
     * @tparam T Type of the attribute value
     * @param key Attribute name
     * @param value Attribute value
     */
    template<typename T>
    void SetAttribute(const std::string& key, const T& value) {
        attributes_[key] = std::unique_ptr<AttributeBase>(
            new AttributeHolder<T>(value));
    }
    
    /**
     * @brief Set an attribute value (move version)
     */
    template<typename T>
    void SetAttribute(const std::string& key, T&& value) {
        attributes_[key] = std::unique_ptr<AttributeBase>(
            new AttributeHolder<T>(std::move(value)));
    }
    
    /**
     * @brief Get an attribute value
     * @tparam T Expected type of the attribute
     * @param key Attribute name
     * @return Reference to the attribute value
     * @throws std::out_of_range if key doesn't exist
     * @throws std::bad_cast if type doesn't match
     */
    template<typename T>
    const T& GetAttribute(const std::string& key) const {
        auto it = attributes_.find(key);
        if (it == attributes_.end()) {
            throw std::out_of_range("Attribute '" + key + "' not found");
        }
        
        auto* holder = dynamic_cast<const AttributeHolder<T>*>(it->second.get());
        if (!holder) {
            throw std::bad_cast();
        }
        
        return holder->value;
    }
    
    /**
     * @brief Get an attribute value with default
     * @tparam T Expected type of the attribute
     * @param key Attribute name
     * @param default_value Default value if attribute doesn't exist
     * @return Attribute value or default
     */
    template<typename T>
    T GetAttributeOr(const std::string& key, const T& default_value) const {
        try {
            return GetAttribute<T>(key);
        } catch (...) {
            return default_value;
        }
    }
    
    /**
     * @brief Check if an attribute exists
     * @param key Attribute name
     * @return true if attribute exists
     */
    bool HasAttribute(const std::string& key) const {
        return attributes_.find(key) != attributes_.end();
    }
    
    /**
     * @brief Remove an attribute
     * @param key Attribute name
     * @return true if attribute was removed, false if it didn't exist
     */
    bool RemoveAttribute(const std::string& key) {
        return attributes_.erase(key) > 0;
    }
    
    /**
     * @brief Clear all attributes
     */
    void ClearAttributes() {
        attributes_.clear();
    }
    
    /**
     * @brief Get the number of attributes
     * @return Number of stored attributes
     */
    size_t AttributeCount() const {
        return attributes_.size();
    }
    
    /**
     * @brief Check if there are no attributes
     * @return true if no attributes are stored
     */
    bool IsEmpty() const {
        return attributes_.empty();
    }
    
    /**
     * @brief Get all attribute keys
     * @return Vector of attribute keys
     */
    std::vector<std::string> GetAttributeKeys() const {
        std::vector<std::string> keys;
        keys.reserve(attributes_.size());
        for (const auto& pair : attributes_) {
            keys.push_back(pair.first);
        }
        return keys;
    }
};

} // namespace xmotion

#endif // GRAPH_ATTRIBUTES_HPP