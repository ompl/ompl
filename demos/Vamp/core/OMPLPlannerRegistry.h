#pragma once

#include "VampOMPLInterfaces.h"
#include <ompl/geometric/planners/informedtrees/BITstar.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <functional>
#include <map>
#include <string>
#include <memory>
#include <mutex>
#include <iostream>

namespace vamp_ompl {

/**
 * @brief Planner factory function type
 * 
 * This function type creates OMPL planners and optionally configures them
 * with custom parameters. The function receives space information and
 * a parameter map, returning a configured planner instance.
 */
using PlannerAllocatorFunction = std::function<ob::PlannerPtr(
    const ob::SpaceInformationPtr& si,
    const std::map<std::string, std::string>& parameters)>;

/**
 * @brief Singleton registry for OMPL planner management
 * 
 */
class PlannerRegistry {
public:
    /**
     * @brief Get singleton instance (thread-safe)
     */
    static PlannerRegistry& getInstance() {
        static PlannerRegistry instance;
        return instance;
    }

    /**
     * @brief Register a planner with factory function
     * @param name Unique planner identifier
     * @param allocator Factory function that creates and configures the planner
     * 
     * Example:
     * ```cpp
     * PlannerRegistry::getInstance().registerPlanner("MyRRT*", 
     *     [](const ob::SpaceInformationPtr& si, const auto& params) {
     *         auto planner = std::make_shared<og::RRTstar>(si);
     *         // Configure parameters using OMPL's ParamSet
     *         for (const auto& [key, value] : params) {
     *             if (planner->params().hasParam(key)) {
     *                 planner->params().setParam(key, value);
     *             }
     *         }
     *         return planner;
     *     });
     * ```
     */
    void registerPlanner(const std::string& name, PlannerAllocatorFunction allocator) {
        std::lock_guard<std::mutex> lock(mutex_);
        planner_allocators_[name] = std::move(allocator);
        std::cout << "Registered planner: " << name << std::endl;
    }

    /**
     * @brief Create planner instance with parameters
     * @param name Planner identifier
     * @param si OMPL space information
     * @param parameters Parameter map for planner configuration
     * @return Configured planner instance
     * @throws VampConfigurationError if planner not found
     */
    ob::PlannerPtr createPlanner(const std::string& name, 
                                const ob::SpaceInformationPtr& si,
                                const std::map<std::string, std::string>& parameters = {}) {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = planner_allocators_.find(name);
        if (it == planner_allocators_.end()) {
            throw VampConfigurationError("Unknown planner: '" + name + "'. Available planners: " + 
                                       getAvailablePlannerNames());
        }
        return it->second(si, parameters);
    }

    /**
     * @brief Check if planner is registered
     * @param name Planner identifier
     * @return true if planner is available
     */
    bool isPlannerRegistered(const std::string& name) const {
        std::lock_guard<std::mutex> lock(mutex_);
        return planner_allocators_.find(name) != planner_allocators_.end();
    }

    /**
     * @brief Get list of registered planner names
     * @return Vector of available planner identifiers
     */
    std::vector<std::string> getRegisteredPlanners() const {
        std::lock_guard<std::mutex> lock(mutex_);
        std::vector<std::string> planners;
        planners.reserve(planner_allocators_.size());
        for (const auto& [name, allocator] : planner_allocators_) {
            planners.push_back(name);
        }
        return planners;
    }

    /**
     * @brief Get comma-separated list of available planners (for error messages)
     */
    std::string getAvailablePlannerNames() const {
        std::lock_guard<std::mutex> lock(mutex_);
        std::string names;
        for (const auto& [name, allocator] : planner_allocators_) {
            if (!names.empty()) names += ", ";
            names += name;
        }
        return names;
    }

    /**
     * @brief Get number of registered planners
     */
    size_t getPlannerCount() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return planner_allocators_.size();
    }

private:
    mutable std::mutex mutex_;
    std::map<std::string, PlannerAllocatorFunction> planner_allocators_;

    /**
     * @brief Private constructor - registers built-in OMPL planners
     * 
     * This follows the same pattern as RobotRegistry, automatically registering
     * commonly used planners while allowing runtime extension.
     */
    PlannerRegistry() {
        registerBuiltInPlanners();
    }

    /**
     * @brief Register commonly used OMPL planners
     * 
     * This method registers the core planners that work well with VAMP.
     * Users can register additional planners at runtime without modifying this code.
     */
    void registerBuiltInPlanners() {
        // RRT-Connect: Fast, bidirectional tree planner
        registerPlanner("RRT-Connect", [](const ob::SpaceInformationPtr& si, const auto& params) {
            auto planner = std::make_shared<ompl::geometric::RRTConnect>(si);
            applyParameters(planner, params);
            return planner;
        });

        // BIT*: Batch Informed Trees - asymptotically optimal
        registerPlanner("BIT*", [](const ob::SpaceInformationPtr& si, const auto& params) {
            auto planner = std::make_shared<ompl::geometric::BITstar>(si);
            applyParameters(planner, params);
            return planner;
        });

        // PRM: Probabilistic Roadmap - good for complex environments
        registerPlanner("PRM", [](const ob::SpaceInformationPtr& si, const auto& params) {
            auto planner = std::make_shared<ompl::geometric::PRM>(si);
            applyParameters(planner, params);
            return planner;
        });
    }

    /**
     * @brief Apply parameters to planner using OMPL's ParamSet introspection
     * @param planner OMPL planner instance
     * @param parameters Parameter map to apply
     * 
     * This helper function uses OMPL's built-in parameter system for
     * type-safe parameter configuration with automatic validation.
     */
    static void applyParameters(const ob::PlannerPtr& planner, 
                               const std::map<std::string, std::string>& parameters) {
        for (const auto& [param_name, param_value] : parameters) {
            if (planner->params().hasParam(param_name)) {
                bool success = planner->params().setParam(param_name, param_value);
                if (!success) {
                    std::cerr << "Warning: Failed to set parameter " << param_name 
                             << " = " << param_value << " for planner " 
                             << planner->getName() << std::endl;
                }
            } else {
                std::cerr << "Warning: Parameter " << param_name 
                         << " not found for planner " << planner->getName() << std::endl;
            }
        }
    }

    // Prevent copying
    PlannerRegistry(const PlannerRegistry&) = delete;
    PlannerRegistry& operator=(const PlannerRegistry&) = delete;
};

/**
 * @brief Convenience function to register a planner
 * @param name Planner identifier
 * @param allocator Factory function
 * 
 * This provides a cleaner interface for planner registration:
 * ```cpp
 * registerPlanner("MyPlanner", [](const auto& si, const auto& params) {
 *     return std::make_shared<MyPlanner>(si);
 * });
 * ```
 */
inline void registerPlanner(const std::string& name, PlannerAllocatorFunction allocator) {
    PlannerRegistry::getInstance().registerPlanner(name, std::move(allocator));
}

/**
 * @brief Convenience function to create a planner
 * @param name Planner identifier
 * @param si Space information
 * @param parameters Parameter map
 * @return Configured planner instance
 */
inline ob::PlannerPtr createPlannerByName(const std::string& name,
                                         const ob::SpaceInformationPtr& si,
                                         const std::map<std::string, std::string>& parameters = {}) {
    return PlannerRegistry::getInstance().createPlanner(name, si, parameters);
}

/**
 * @brief Get list of all available planner names
 * @return Vector of registered planner identifiers
 */
inline std::vector<std::string> getAllPlannerNames() {
    return PlannerRegistry::getInstance().getRegisteredPlanners();
}

} // namespace vamp_ompl 