#pragma once

#include "VampOMPLInterfaces.h"
#include "VampUtils.h"
#include "VampParameterManager.h"
#include <ompl/base/Planner.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/ProjectionEvaluator.h>
#include <ompl/base/spaces/RealVectorStateProjections.h>
#include <ompl/geometric/SimpleSetup.h>

// Include ALL available OMPL geometric planners - Comprehensive Coverage
// RRT Family - Complete coverage
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/RRTXstatic.h>
#include <ompl/geometric/planners/rrt/RRTsharp.h>
#include <ompl/geometric/planners/rrt/SORRTstar.h>
#include <ompl/geometric/planners/rrt/TRRT.h>
#include <ompl/geometric/planners/rrt/BiTRRT.h>
#include <ompl/geometric/planners/rrt/LBTRRT.h>
#include <ompl/geometric/planners/rrt/LazyRRT.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
// #include <ompl/geometric/planners/rrt/STRRTstar.h>  // Commented out - requires SpaceTimeStateSpace
#include <ompl/geometric/planners/rrt/TSRRT.h>
#include <ompl/geometric/planners/xxl/XXL.h>
#include "VampXXLDecomposition.h"
#include "VampTaskSpaceConfig.h"
#include <ompl/geometric/planners/rrt/VFRRT.h>
#include <ompl/geometric/planners/rrt/pRRT.h>
#include <ompl/geometric/planners/rrt/LazyLBTRRT.h>

// PRM Family - Complete coverage
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/prm/LazyPRM.h>
#include <ompl/geometric/planners/prm/LazyPRMstar.h>
#include <ompl/geometric/planners/prm/SPARS.h>
#include <ompl/geometric/planners/prm/SPARStwo.h>

// Informed Trees Family - Complete coverage
#include <ompl/geometric/planners/informedtrees/BITstar.h>
#include <ompl/geometric/planners/informedtrees/ABITstar.h>
#include <ompl/geometric/planners/informedtrees/AITstar.h>
#include <ompl/geometric/planners/informedtrees/EITstar.h>
#include <ompl/geometric/planners/informedtrees/EIRMstar.h>

// KPIECE Family - Complete coverage
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/kpiece/BKPIECE1.h>
#include <ompl/geometric/planners/kpiece/LBKPIECE1.h>

// EST Family - Complete coverage
#include <ompl/geometric/planners/est/EST.h>
#include <ompl/geometric/planners/est/BiEST.h>
#include <ompl/geometric/planners/est/ProjEST.h>

// SBL Family - Complete coverage
#include <ompl/geometric/planners/sbl/SBL.h>
#include <ompl/geometric/planners/sbl/pSBL.h>

// FMT Family - Complete coverage
#include <ompl/geometric/planners/fmt/FMT.h>
#include <ompl/geometric/planners/fmt/BFMT.h>

// Other Single-Query Planners
#include <ompl/geometric/planners/sst/SST.h>
#include <ompl/geometric/planners/stride/STRIDE.h>
#include <ompl/geometric/planners/pdst/PDST.h>

// RLRT Family - Complete coverage
#include <ompl/geometric/planners/rlrt/RLRT.h>
#include <ompl/geometric/planners/rlrt/BiRLRT.h>

// Anytime/Multi-threaded Planners
#include <ompl/geometric/planners/cforest/CForest.h>
#include <ompl/geometric/planners/AnytimePathShortening.h>

// Experience-based Planners
#include <ompl/geometric/planners/experience/LightningRetrieveRepair.h>
#include <ompl/geometric/planners/experience/ThunderRetrieveRepair.h>
#include "VampExperiencePlanners.h"

#include <memory>
#include <map>
#include <set>
#include <string>
#include <functional>
#include <vector>
#include <iostream>

namespace vamp_ompl {

namespace og = ompl::geometric;
namespace ob = ompl::base;

/**
 * @brief planner registry with parameter optimization
 * 
 */
class VampPlannerRegistry {
public:
    using PlannerCreatorFunction = std::function<std::shared_ptr<ob::Planner>(const ob::SpaceInformationPtr&)>;
    
    /**
     * @brief Simple configuration for planner creation
     */
    struct PlannerCreationConfig {
        std::string planner_name;
        std::string robot_name;             ///< Robot name for custom projection lookup
        std::map<std::string, std::string> user_parameters;
        bool apply_projections = true;      ///< Auto-apply projections for projection-based planners
        
        PlannerCreationConfig(const std::string& name) : planner_name(name) {}
        PlannerCreationConfig(const std::string& name, const std::string& robot) 
            : planner_name(name), robot_name(robot) {}
    };
    
    /**
     * @brief Get singleton instance
     */
    static VampPlannerRegistry& getInstance() {
        static VampPlannerRegistry instance;
        return instance;
    }
    
    /**
     * @brief Create planner with user parameters (OMPL defaults if no user parameters)
     * @param config Configuration containing planner name and optional user parameters
     * @param spaceInfo Space information for the planner
     * @return Planner instance with user parameters applied (or OMPL defaults)
     * @throws VampConfigurationError if planner name is unknown
     */
    std::shared_ptr<ob::Planner> createPlanner(const PlannerCreationConfig& config, 
                                               const ob::SpaceInformationPtr& spaceInfo) {
        // Create base planner
        auto it = planner_creators_.find(config.planner_name);
        if (it == planner_creators_.end()) {
            throw VampConfigurationError("Unknown planner: '" + config.planner_name + 
                                       "'. Available planners: " + getAvailablePlannerNames());
        }
        
        auto planner = it->second(spaceInfo);
        
        // Apply user parameters if provided (OMPL defaults used if not specified)
        if (!config.user_parameters.empty()) {
            applyUserParameters(planner.get(), config.user_parameters);
        }
        
        // Apply projections for projection-based planners (with custom projection support)
        if (config.apply_projections) {
            applyProjectionIfNeeded(planner.get(), spaceInfo, config.planner_name, config.robot_name);
        }
        
        return planner;
    }
    
    /**
     * @brief Create planner by name with OMPL defaults (convenience method)
     * @param plannerName Name of the planner to create
     * @param spaceInfo Space information for the planner
     * @return Planner instance with OMPL defaults
     * @throws VampConfigurationError if planner name is unknown
     */
    std::shared_ptr<ob::Planner> createPlanner(const std::string& plannerName, 
                                               const ob::SpaceInformationPtr& spaceInfo) {
        PlannerCreationConfig config(plannerName);
        return createPlanner(config, spaceInfo);
    }
    
    /**
     * @brief Register a custom planner
     * @param plannerName Unique name for the planner
     * @param creatorFunction Factory function to create the planner
     */
    void registerPlanner(const std::string& plannerName, PlannerCreatorFunction creatorFunction) {
        planner_creators_[plannerName] = std::move(creatorFunction);
    }
    
    /**
     * @brief Get list of all available planner names
     * @return Comma-separated list of planner names
     */
    std::string getAvailablePlannerNames() const {
        std::vector<std::string> names;
        for (const auto& [name, _] : planner_creators_) {
            names.push_back(name);
        }
        
        std::sort(names.begin(), names.end());
        
        std::string result;
        for (size_t i = 0; i < names.size(); ++i) {
            if (i > 0) result += ", ";
            result += names[i];
        }
        return result;
    }
    
    /**
     * @brief Get all available planner names as vector
     * @return Vector of planner names
     */
    std::vector<std::string> getAvailablePlannerNamesVector() const {
        std::vector<std::string> names;
        for (const auto& [name, _] : planner_creators_) {
            names.push_back(name);
        }
        std::sort(names.begin(), names.end());
        return names;
    }
    
    /**
     * @brief Check if a planner is available
     * @param plannerName Name to check
     * @return True if planner is available
     */
    bool isPlannerAvailable(const std::string& plannerName) const {
        return planner_creators_.find(plannerName) != planner_creators_.end();
    }
    
    /**
     * @brief Get planner count
     * @return Number of available planners
     */
    size_t getPlannerCount() const {
        return planner_creators_.size();
    }
    
    /**
     * @brief Get list of planner names
     * @return Vector of available planner names
     */
    std::vector<std::string> getPlannerNames() const {
        std::vector<std::string> names;
        names.reserve(planner_creators_.size());
        for (const auto& [name, creator] : planner_creators_) {
            names.push_back(name);
        }
        return names;
    }
    
    /**
     * @brief Create default projection for manipulators
     * 
     * 
     * 
     * @param space State space (must be RealVectorStateSpace) 
     * @param robot_dimension Full dimensionality of the robot
     * @return Memory-safe projection evaluator instance
     */
    static ob::ProjectionEvaluatorPtr createOptimalManipulatorProjection(
        const ob::StateSpacePtr& space, unsigned int robot_dimension) {
        
        try {
            // Cast to RealVectorStateSpace - required for manipulators
            auto real_space = std::dynamic_pointer_cast<ob::RealVectorStateSpace>(space);
            if (!real_space) {
                std::cerr << "Warning: Optimal projection requires RealVectorStateSpace" << std::endl;
                return nullptr;
            }
            
            // MEMORY SAFETY: Copy bounds data to avoid dangling references
            const auto& original_bounds = real_space->getBounds();
            ob::RealVectorBounds safe_bounds(original_bounds.low.size());
            for (size_t i = 0; i < original_bounds.low.size(); ++i) {
                safe_bounds.setLow(i, original_bounds.low[i]);
                safe_bounds.setHigh(i, original_bounds.high[i]);
            }
            
            // Create a COPY of the state space with copied bounds to avoid reference issues
            auto safe_space = std::make_shared<ob::RealVectorStateSpace>(robot_dimension);
            safe_space->setBounds(safe_bounds);
            
            // Optimal projection dimension: first k coordinates where k = min(3, n)
            unsigned int projection_dim = std::min(3u, robot_dimension);
            
            // Create projection coordinate indices
            std::vector<unsigned int> projection_coords;
            projection_coords.reserve(projection_dim);
            for (unsigned int i = 0; i < projection_dim; ++i) {
                projection_coords.push_back(i);
            }
            
            // Adaptive cell sizing based on joint limits
            std::vector<double> cell_sizes;
            cell_sizes.reserve(projection_dim);
            
            const double resolution_factor = 50.0;  // Empirically optimal for most manipulators
            
            for (unsigned int i = 0; i < projection_dim; ++i) {
                if (i < safe_bounds.low.size()) {
                    // Adaptive sizing: larger ranges get larger cells
                    double range = safe_bounds.high[i] - safe_bounds.low[i];
                    double cell_size = range / resolution_factor;
                    // Clamp to reasonable bounds
                    cell_size = std::max(0.01, std::min(0.5, cell_size));
                    cell_sizes.push_back(cell_size);
                } else {
                    // Fallback for unbounded dimensions
                    cell_sizes.push_back(0.1);
                }
            }
            
            // Use the SAFE copy of state space, not the original
            return std::make_shared<ob::RealVectorOrthogonalProjectionEvaluator>(
                safe_space, cell_sizes, projection_coords);
                
        } catch (const std::exception& e) {
            std::cerr << "Warning: Failed to create memory-safe projection: " << e.what() << std::endl;
            return nullptr;
        }
    }
    
    /**
     * @brief Register custom projection function
     * 
     * Advanced users can register custom projection functions for specialized applications.
     * 
     * Registration Options:
     * 1. Robot-wide default: projection_name = "default"
     * 2. Planner-specific: projection_name = "<PLANNER_NAME>_projection"
     * 
     * Example Usage:
     * ```cpp
     * auto& registry = VampPlannerRegistry::getInstance();
     * 
     * 
     * // Planner-specific custom projection  
     * auto kpiece_proj = registry.createCustomOrthogonalProjection(space, {1,3,5}, {0.05, 0.1, 0.15});
     * registry.registerCustomProjection("panda", "KPIECE_projection", kpiece_proj);
     * 
     * // Then create planner normally - custom projection will be applied automatically
     * auto planner = registry.createPlanner("KPIECE", spaceInfo);
     * ```
     * 
     * @param robot_name Robot identifier
     * @param projection_name "default" for robot-wide, or "<PLANNER_NAME>_projection" for specific planner
     * @param projection_function Custom projection evaluator
     */
    void registerCustomProjection(const std::string& robot_name,
                                 const std::string& projection_name,
                                 ob::ProjectionEvaluatorPtr projection_function) {
        custom_projections_[robot_name][projection_name] = projection_function;
        std::cout << "Registered custom projection '" << projection_name 
                  << "' for robot '" << robot_name << "'" << std::endl;
    }
    
    /**
     * @brief Create custom workspace projection (Helper for advanced users)
     * 
     * Creates a projection onto specified workspace coordinates, useful for
     * end-effector or specific joint projections.
     * 
     * @param space State space
     * @param robot_dimension Full robot dimension
     * @param coordinates Which coordinates to project onto (e.g., {0,1,2} for first 3 joints)
     * @param cell_sizes Custom cell sizes (optional, auto-computed if empty)
     * @return Custom projection evaluator
     */
    static ob::ProjectionEvaluatorPtr createCustomOrthogonalProjection(
        const ob::StateSpacePtr& space, 
        const std::vector<unsigned int>& coordinates,
        const std::vector<double>& cell_sizes = {}) {
        
        try {
            auto real_space = std::dynamic_pointer_cast<ob::RealVectorStateSpace>(space);
            if (!real_space) return nullptr;
            
            // MEMORY SAFETY: Create independent copy of state space
            const auto& original_bounds = real_space->getBounds();
            ob::RealVectorBounds safe_bounds(original_bounds.low.size());
            for (size_t i = 0; i < original_bounds.low.size(); ++i) {
                safe_bounds.setLow(i, original_bounds.low[i]);
                safe_bounds.setHigh(i, original_bounds.high[i]);
            }
            
            auto safe_space = std::make_shared<ob::RealVectorStateSpace>(space->getDimension());
            safe_space->setBounds(safe_bounds);
            
            std::vector<double> final_cell_sizes = cell_sizes;
            if (final_cell_sizes.empty()) {
                // Auto-compute cell sizes based on joint bounds
                final_cell_sizes.reserve(coordinates.size());
                for (auto coord : coordinates) {
                    if (coord < safe_bounds.low.size()) {
                        double range = safe_bounds.high[coord] - safe_bounds.low[coord];
                        final_cell_sizes.push_back(range / 50.0);  // Reasonable default
                    } else {
                        final_cell_sizes.push_back(0.1);
                    }
                }
            }
            
            // Use safe copy, not original space
            return std::make_shared<ob::RealVectorOrthogonalProjectionEvaluator>(
                safe_space, final_cell_sizes, coordinates);
                
        } catch (const std::exception& e) {
            std::cerr << "Warning: Failed to create memory-safe custom projection: " << e.what() << std::endl;
            return nullptr;
        }
    }
    
    /**
     * @brief Apply custom projection to specific planner instance (Direct Method)
     * 
     * For users who want direct control over projection application.
     * 
     * Usage:
     * ```cpp
     * auto planner = registry.createPlanner("KPIECE", spaceInfo);
     * auto custom_proj = registry.createCustomOrthogonalProjection(space, {1,2,4});
     * registry.applyCustomProjectionToplanner(planner.get(), custom_proj, "KPIECE");
     * ```
     * 
     * @param planner Planner instance to modify
     * @param projection Custom projection to apply
     * @param planner_name Name of planner (for validation)
     * @return True if projection was successfully applied
     */
    bool applyCustomProjectionToPlanner(ob::Planner* planner,
                                       ob::ProjectionEvaluatorPtr projection,
                                       const std::string& planner_name) {
        if (!planner || !projection) return false;
        
        try {
            // Apply projection using safe dynamic casting
            if (auto kpiece = dynamic_cast<og::KPIECE1*>(planner)) {
                kpiece->setProjectionEvaluator(projection);
                std::cout << "Applied custom projection to " << planner_name << std::endl;
                return true;
            } else if (auto bkpiece = dynamic_cast<og::BKPIECE1*>(planner)) {
                bkpiece->setProjectionEvaluator(projection);
                std::cout << "Applied custom projection to " << planner_name << std::endl;
                return true;
            } else if (auto lbkpiece = dynamic_cast<og::LBKPIECE1*>(planner)) {
                lbkpiece->setProjectionEvaluator(projection);
                std::cout << "Applied custom projection to " << planner_name << std::endl;
                return true;
            } else if (auto sbl = dynamic_cast<og::SBL*>(planner)) {
                sbl->setProjectionEvaluator(projection);
                std::cout << "Applied custom projection to " << planner_name << std::endl;
                return true;
            } else if (auto psbl = dynamic_cast<og::pSBL*>(planner)) {
                psbl->setProjectionEvaluator(projection);
                std::cout << "Applied custom projection to " << planner_name << std::endl;
                return true;
            } else if (auto projest = dynamic_cast<og::ProjEST*>(planner)) {
                projest->setProjectionEvaluator(projection);
                std::cout << "Applied custom projection to " << planner_name << std::endl;
                return true;
            } else if (auto pdst = dynamic_cast<og::PDST*>(planner)) {
                pdst->setProjectionEvaluator(projection);
                std::cout << "Applied custom projection to " << planner_name << std::endl;
                return true;
            } else if (auto stride = dynamic_cast<og::STRIDE*>(planner)) {
                stride->setProjectionEvaluator(projection);
                std::cout << "Applied custom projection to " << planner_name << std::endl;
                return true;
            }
            
            std::cerr << "Warning: " << planner_name << " is not a projection-based planner" << std::endl;
            return false;
            
        } catch (const std::exception& e) {
            std::cerr << "Error applying custom projection: " << e.what() << std::endl;
            return false;
        }
    }
    



private:
    std::map<std::string, PlannerCreatorFunction> planner_creators_;
    std::set<std::string> projection_based_planners_;
    std::map<std::string, std::map<std::string, ob::ProjectionEvaluatorPtr>> custom_projections_;
    
    /**
     * @brief Private constructor registers all OMPL geometric planners
     */
    VampPlannerRegistry() {
        registerAllOMPLPlanners();
        initializeProjectionMetadata();
        // Initialize experience planners system
        vamp_ompl::initializeExperiencePlanners();
    }
    
    /**
     * @brief Apply smart parameters based on optimization level
     * @param planner Planner to configure
     * @param config Creation configuration
     */
    void applyUserParameters(ob::Planner* planner, const std::map<std::string, std::string>& user_parameters) {
        if (!planner || user_parameters.empty()) {
            return;
        }
        
        try {
            for (const auto& [param_name, param_value] : user_parameters) {
                try {
                    planner->params().setParam(param_name, param_value);
                } catch (const std::exception& e) {
                    std::cout << "Parameter warning for " << param_name << ": " << e.what() << std::endl;
                }
            }
        } catch (const std::exception& e) {
            std::cerr << "Warning: Could not apply parameters: " << e.what() << std::endl;
        }
    }
    
    /**
     * @brief Apply projections for projection-based planners (with custom projection support)
     * 
     * This method intelligently applies projections in the following priority order:
     * 1. Custom projections (if registered for this robot+planner combination)
     * 2. default projection
     * 
     * @param planner Planner instance
     * @param spaceInfo Space information
     * @param plannerName Planner name
     * @param robotName Robot name (for custom projection lookup)
     */
    void applyProjectionIfNeeded(ob::Planner* planner, 
                                const ob::SpaceInformationPtr& spaceInfo,
                                const std::string& plannerName,
                                const std::string& robotName = "") {
        if (projection_based_planners_.find(plannerName) == projection_based_planners_.end()) {
            return;  // Not a projection-based planner
        }
        
        try {
            // IMPORTANT: Check if planner already has a projection (e.g., from state space default)
            // This prevents double-application and memory conflicts
            bool already_has_projection = false;
            
            if (auto kpiece = dynamic_cast<og::KPIECE1*>(planner)) {
                already_has_projection = (kpiece->getProjectionEvaluator() != nullptr);
            } else if (auto bkpiece = dynamic_cast<og::BKPIECE1*>(planner)) {
                already_has_projection = (bkpiece->getProjectionEvaluator() != nullptr);
            } else if (auto lbkpiece = dynamic_cast<og::LBKPIECE1*>(planner)) {
                already_has_projection = (lbkpiece->getProjectionEvaluator() != nullptr);
            } else if (auto sbl = dynamic_cast<og::SBL*>(planner)) {
                already_has_projection = (sbl->getProjectionEvaluator() != nullptr);
            } else if (auto psbl = dynamic_cast<og::pSBL*>(planner)) {
                already_has_projection = (psbl->getProjectionEvaluator() != nullptr);
            } else if (auto projest = dynamic_cast<og::ProjEST*>(planner)) {
                already_has_projection = (projest->getProjectionEvaluator() != nullptr);
            } else if (auto pdst = dynamic_cast<og::PDST*>(planner)) {
                already_has_projection = (pdst->getProjectionEvaluator() != nullptr);
            } else if (auto stride = dynamic_cast<og::STRIDE*>(planner)) {
                already_has_projection = (stride->getProjectionEvaluator() != nullptr);
            }
            
            if (already_has_projection) {
                std::cout << "Skipping projection for " << plannerName 
                         << " - already has projection (likely from state space default)" << std::endl;
                return;
            }
            
            ob::ProjectionEvaluatorPtr projection = nullptr;
            std::string projection_source = "default";
            
            // PRIORITY 1: Check for custom projections
            if (!robotName.empty() && custom_projections_.count(robotName)) {
                auto& robot_projections = custom_projections_[robotName];
                
                // Check for planner-specific custom projection
                std::string planner_key = plannerName + "_projection";
                if (robot_projections.count(planner_key)) {
                    projection = robot_projections[planner_key];
                    projection_source = "custom (planner-specific)";
                }
                // Check for robot-wide custom projection
                else if (robot_projections.count("default")) {
                    projection = robot_projections["default"];
                    projection_source = "custom (robot-default)";
                }
            }
            
            // PRIORITY 2: Use default projection for manually created planners
            if (!projection) {
                unsigned int robot_dimension = spaceInfo->getStateSpace()->getDimension();
                projection = createOptimalManipulatorProjection(
                    spaceInfo->getStateSpace(), robot_dimension);
                projection_source = "optimal default (manual)";
            }
                
            if (projection) {
                // Apply projection using safe dynamic casting
                bool applied = false;
                if (auto kpiece = dynamic_cast<og::KPIECE1*>(planner)) {
                    kpiece->setProjectionEvaluator(projection);
                    applied = true;
                } else if (auto bkpiece = dynamic_cast<og::BKPIECE1*>(planner)) {
                    bkpiece->setProjectionEvaluator(projection);
                    applied = true;
                } else if (auto lbkpiece = dynamic_cast<og::LBKPIECE1*>(planner)) {
                    lbkpiece->setProjectionEvaluator(projection);
                    applied = true;
                } else if (auto sbl = dynamic_cast<og::SBL*>(planner)) {
                    sbl->setProjectionEvaluator(projection);
                    applied = true;
                } else if (auto psbl = dynamic_cast<og::pSBL*>(planner)) {
                    psbl->setProjectionEvaluator(projection);
                    applied = true;
                } else if (auto projest = dynamic_cast<og::ProjEST*>(planner)) {
                    projest->setProjectionEvaluator(projection);
                    applied = true;
                } else if (auto pdst = dynamic_cast<og::PDST*>(planner)) {
                    pdst->setProjectionEvaluator(projection);
                    applied = true;
                } else if (auto stride = dynamic_cast<og::STRIDE*>(planner)) {
                    stride->setProjectionEvaluator(projection);
                    applied = true;
                }
                
                if (applied) {
                    std::cout << "Applied " << projection_source << " projection to " 
                             << plannerName << " (instance: " << projection.get() << ")" << std::endl;
                }
            }
        } catch (const std::exception& e) {
            std::cerr << "Warning: Could not set projection for " << plannerName 
                      << ": " << e.what() << std::endl;
        }
    }
    
    /**
     * @brief Initialize projection metadata
     */
    void initializeProjectionMetadata() {
        // Projection-based planners that need automatic projection setup
        projection_based_planners_ = {
            "KPIECE", "BKPIECE", "LBKPIECE", "SBL", "pSBL", 
            "ProjEST", "STRIDE", "PDST"
        };
    }
    
    /**
     * @brief Register all available OMPL geometric planners
     */
    void registerAllOMPLPlanners() {
        // RRT Family
        registerPlanner("RRT", [](const ob::SpaceInformationPtr& si) {
            return std::make_shared<og::RRT>(si);
        });
        registerPlanner("RRT-Connect", [](const ob::SpaceInformationPtr& si) {
            return std::make_shared<og::RRTConnect>(si);
        });
        registerPlanner("RRT*", [](const ob::SpaceInformationPtr& si) {
            return std::make_shared<og::RRTstar>(si);
        });
        registerPlanner("RRTXstatic", [](const ob::SpaceInformationPtr& si) {
            return std::make_shared<og::RRTXstatic>(si);
        });
        registerPlanner("RRTsharp", [](const ob::SpaceInformationPtr& si) {
            return std::make_shared<og::RRTsharp>(si);
        });
        registerPlanner("SORRTstar", [](const ob::SpaceInformationPtr& si) {
            return std::make_shared<og::SORRTstar>(si);
        });
        registerPlanner("TRRT", [](const ob::SpaceInformationPtr& si) {
            return std::make_shared<og::TRRT>(si);
        });
        registerPlanner("BiTRRT", [](const ob::SpaceInformationPtr& si) {
            return std::make_shared<og::BiTRRT>(si);
        });
        registerPlanner("LBTRRT", [](const ob::SpaceInformationPtr& si) {
            return std::make_shared<og::LBTRRT>(si);
        });
        registerPlanner("LazyRRT", [](const ob::SpaceInformationPtr& si) {
            return std::make_shared<og::LazyRRT>(si);
        });
        registerPlanner("InformedRRTstar", [](const ob::SpaceInformationPtr& si) {
            return std::make_shared<og::InformedRRTstar>(si);
        });
        // STRRTstar commented out - requires SpaceTimeStateSpace, not compatible with RealVectorStateSpace
        // registerPlanner("STRRTstar", [](const ob::SpaceInformationPtr& si) {
        //     return std::make_shared<og::STRRTstar>(si);
        // });
        registerPlanner("pRRT", [](const ob::SpaceInformationPtr& si) {
            return std::make_shared<og::pRRT>(si);
        });
        registerPlanner("LazyLBTRRT", [](const ob::SpaceInformationPtr& si) {
            return std::make_shared<og::LazyLBTRRT>(si);
        });
        
        // Task-Space RRT - Advanced manipulation planning
        registerPlanner("TSRRT", [](const ob::SpaceInformationPtr& si) {
            try {
                // Create default task space configuration for manipulators
                auto task_config = VampTaskSpaceFactory::createDefault(si);
                
                if (!task_config) {
                    throw std::runtime_error("Failed to create TSRRT task space configuration");
                }
                
                return std::make_shared<og::TSRRT>(si, task_config);
            } catch (const std::exception& e) {
                OMPL_ERROR("TSRRT planner creation failed: %s", e.what());
                throw;
            }
        });
        
        // Note: VFRRT requires vector field configuration and is not 
        // suitable for basic registry without additional configuration infrastructure
        
        // PRM Family
        registerPlanner("PRM", [](const ob::SpaceInformationPtr& si) {
            return std::make_shared<og::PRM>(si);
        });
        registerPlanner("PRMstar", [](const ob::SpaceInformationPtr& si) {
            return std::make_shared<og::PRMstar>(si);
        });
        registerPlanner("LazyPRM", [](const ob::SpaceInformationPtr& si) {
            return std::make_shared<og::LazyPRM>(si);
        });
        registerPlanner("LazyPRMstar", [](const ob::SpaceInformationPtr& si) {
            return std::make_shared<og::LazyPRMstar>(si);
        });
        registerPlanner("SPARS", [](const ob::SpaceInformationPtr& si) {
            return std::make_shared<og::SPARS>(si);
        });
        registerPlanner("SPARStwo", [](const ob::SpaceInformationPtr& si) {
            return std::make_shared<og::SPARStwo>(si);
        });
        
        // Informed Trees Family
        registerPlanner("BIT*", [](const ob::SpaceInformationPtr& si) {
            return std::make_shared<og::BITstar>(si);
        });
        registerPlanner("ABITstar", [](const ob::SpaceInformationPtr& si) {
            return std::make_shared<og::ABITstar>(si);
        });
        registerPlanner("AIT*", [](const ob::SpaceInformationPtr& si) {
            return std::make_shared<og::AITstar>(si);
        });
        registerPlanner("EIT*", [](const ob::SpaceInformationPtr& si) {
            return std::make_shared<og::EITstar>(si);
        });
        registerPlanner("EIRMstar", [](const ob::SpaceInformationPtr& si) {
            return std::make_shared<og::EIRMstar>(si);
        });
        
        // KPIECE Family
        registerPlanner("KPIECE", [](const ob::SpaceInformationPtr& si) {
            return std::make_shared<og::KPIECE1>(si);
        });
        registerPlanner("BKPIECE", [](const ob::SpaceInformationPtr& si) {
            return std::make_shared<og::BKPIECE1>(si);
        });
        registerPlanner("LBKPIECE", [](const ob::SpaceInformationPtr& si) {
            return std::make_shared<og::LBKPIECE1>(si);
        });
        
        // EST Family
        registerPlanner("EST", [](const ob::SpaceInformationPtr& si) {
            return std::make_shared<og::EST>(si);
        });
        registerPlanner("BiEST", [](const ob::SpaceInformationPtr& si) {
            return std::make_shared<og::BiEST>(si);
        });
        registerPlanner("ProjEST", [](const ob::SpaceInformationPtr& si) {
            return std::make_shared<og::ProjEST>(si);
        });
        
        // SBL Family
        registerPlanner("SBL", [](const ob::SpaceInformationPtr& si) {
            return std::make_shared<og::SBL>(si);
        });
        registerPlanner("pSBL", [](const ob::SpaceInformationPtr& si) {
            return std::make_shared<og::pSBL>(si);
        });
        
        // FMT Family
        registerPlanner("FMT", [](const ob::SpaceInformationPtr& si) {
            return std::make_shared<og::FMT>(si);
        });
        registerPlanner("BFMT", [](const ob::SpaceInformationPtr& si) {
            return std::make_shared<og::BFMT>(si);
        });
        
        // RLRT Family
        registerPlanner("RLRT", [](const ob::SpaceInformationPtr& si) {
            return std::make_shared<og::RLRT>(si);
        });
        registerPlanner("BiRLRT", [](const ob::SpaceInformationPtr& si) {
            return std::make_shared<og::BiRLRT>(si);
        });
        
        // XXL Family - High-dimensional manipulation planners
        registerPlanner("XXL", [](const ob::SpaceInformationPtr& si) {
            try {
                // Create default decomposition for manipulators
                auto decomposition = VampXXLDecompositionFactory::createDefault(si);
                
                if (!decomposition) {
                    throw std::runtime_error("Failed to create XXL decomposition");
                }
                
                return std::make_shared<og::XXL>(si, decomposition);
            } catch (const std::exception& e) {
                OMPL_ERROR("XXL planner creation failed: %s", e.what());
                throw;
            }
        });
        
        // Anytime/Multi-threaded Planners
        registerPlanner("CForest", [](const ob::SpaceInformationPtr& si) {
            return std::make_shared<og::CForest>(si);
        });
        registerPlanner("AnytimePathShortening", [](const ob::SpaceInformationPtr& si) {
            return std::make_shared<og::AnytimePathShortening>(si);
        });
        
        // Note: Experience-based planners require experience databases
        // LightningRetrieveRepair requires LightningDBPtr
        // ThunderRetrieveRepair requires ThunderDBPtr
        // These are not suitable for basic registry without database setup
        
        // Experience-based planners WITH DEFAULT DATABASES
        // Users can still register custom databases programmatically
        registerPlanner("Lightning", [](const ob::SpaceInformationPtr& si) {
            // Use default configuration
            auto config = vamp_ompl::ExperienceConfig::createDefault("default", "default");
            return std::static_pointer_cast<ob::Planner>(
                vamp_ompl::VampExperiencePlanners::getInstance().createLightningPlanner(si, config));
        });
        
        registerPlanner("Thunder", [](const ob::SpaceInformationPtr& si) {
            // Use default configuration
            auto config = vamp_ompl::ExperienceConfig::createDefault("default", "default");
            return std::static_pointer_cast<ob::Planner>(
                vamp_ompl::VampExperiencePlanners::getInstance().createThunderPlanner(si, config));
        });
        
        // OMPL-standard names for compatibility
        registerPlanner("LightningRetrieveRepair", [](const ob::SpaceInformationPtr& si) {
            auto config = vamp_ompl::ExperienceConfig::createDefault("default", "default");
            return std::static_pointer_cast<ob::Planner>(
                vamp_ompl::VampExperiencePlanners::getInstance().createLightningPlanner(si, config));
        });
        
        registerPlanner("ThunderRetrieveRepair", [](const ob::SpaceInformationPtr& si) {
            auto config = vamp_ompl::ExperienceConfig::createDefault("default", "default");
            return std::static_pointer_cast<ob::Planner>(
                vamp_ompl::VampExperiencePlanners::getInstance().createThunderPlanner(si, config));
        });
        
        // Other Single-Query Planners
        registerPlanner("SST", [](const ob::SpaceInformationPtr& si) {
            return std::make_shared<og::SST>(si);
        });
        registerPlanner("STRIDE", [](const ob::SpaceInformationPtr& si) {
            return std::make_shared<og::STRIDE>(si);
        });
        registerPlanner("PDST", [](const ob::SpaceInformationPtr& si) {
            return std::make_shared<og::PDST>(si);
        });
        
        std::cout << "VampPlannerRegistry: Registered " << getPlannerCount() 
                  << " OMPL geometric planners" << std::endl;
    }
};

/**
 * @brief Convenience function to create an optimized planner by name (DEFAULT: ENHANCED)
 * @param plannerName Name of the planner
 * @param spaceInfo Space information
 * @return Enhanced planner instance
 */
inline std::shared_ptr<ob::Planner> createPlannerByName(const std::string& plannerName, 
                                                        const ob::SpaceInformationPtr& spaceInfo) {
    return VampPlannerRegistry::getInstance().createPlanner(plannerName, spaceInfo);
}

/**
 * @brief Create planner with configuration (user parameters + OMPL defaults)
 * @param config Planner creation configuration
 * @param spaceInfo Space information
 * @return Planner instance with user parameters applied (or OMPL defaults)
 */
inline std::shared_ptr<ob::Planner> createPlannerByConfig(const VampPlannerRegistry::PlannerCreationConfig& config, 
                                                          const ob::SpaceInformationPtr& spaceInfo) {
    return VampPlannerRegistry::getInstance().createPlanner(config, spaceInfo);
}



/**
 * @brief Convenience function to register a custom planner
 * @param plannerName Name for the planner
 * @param creatorFunction Factory function
 */
inline void registerCustomPlanner(const std::string& plannerName, 
                                 VampPlannerRegistry::PlannerCreatorFunction creatorFunction) {
    VampPlannerRegistry::getInstance().registerPlanner(plannerName, std::move(creatorFunction));
}

/**
 * @brief Get all available planner names
 * @return Vector of planner names
 */
inline std::vector<std::string> getAllPlannerNames() {
    return VampPlannerRegistry::getInstance().getAvailablePlannerNamesVector();
}

/**
 * @brief Create planner with custom projection (Convenience Function)
 * 
 * Complete workflow for creating a planner with custom projection:
 * 
 * Usage Example:
 * ```cpp
 * // Create custom projection
 * auto custom_proj = VampPlannerRegistry::createCustomOrthogonalProjection(
 *     spaceInfo->getStateSpace(), {1, 3, 5}, {0.05, 0.1, 0.15});
 * 
 * // Create planner with custom projection applied
 * auto planner = createPlannerWithCustomProjection("KPIECE", spaceInfo, custom_proj);
 * ```
 * 
 * @param plannerName Name of planner to create
 * @param spaceInfo Space information
 * @param customProjection Custom projection to apply
 * @return Planner with custom projection applied
 */
inline std::shared_ptr<ob::Planner> createPlannerWithCustomProjection(
    const std::string& plannerName,
    const ob::SpaceInformationPtr& spaceInfo,
    ob::ProjectionEvaluatorPtr customProjection) {
    
    auto& registry = VampPlannerRegistry::getInstance();
    
    // Create planner without auto-projection
    VampPlannerRegistry::PlannerCreationConfig config(plannerName);
    config.apply_projections = false;  // We'll apply custom projection manually
    auto planner = registry.createPlanner(config, spaceInfo);
    
    // Apply custom projection
    if (planner && customProjection) {
        registry.applyCustomProjectionToPlanner(planner.get(), customProjection, plannerName);
    }
    
    return planner;
}

/**
 * @brief Register and use custom projection in one call (Convenience Function)
 * 
 * Ultimate convenience function for custom projections:
 * 
 * Usage Example:
 * ```cpp
 * // Option 1: Custom coordinates projection
 * auto planner = registerAndCreatePlannerWithProjection(
 *     "KPIECE", spaceInfo, "my_robot", {1, 3, 5});
 * 
 * // Option 2: Custom coordinates + cell sizes
 * auto planner = registerAndCreatePlannerWithProjection(
 *     "SBL", spaceInfo, "my_robot", {0, 2, 4}, {0.1, 0.05, 0.2});
 * ```
 * 
 * @param plannerName Name of planner
 * @param spaceInfo Space information  
 * @param robotName Robot identifier
 * @param coordinates Projection coordinates
 * @param cellSizes Custom cell sizes (optional)
 * @return Planner with custom projection
 */
inline std::shared_ptr<ob::Planner> registerAndCreatePlannerWithProjection(
    const std::string& plannerName,
    const ob::SpaceInformationPtr& spaceInfo, 
    const std::string& robotName,
    const std::vector<unsigned int>& coordinates,
    const std::vector<double>& cellSizes = {}) {
    
    auto& registry = VampPlannerRegistry::getInstance();
    
    // Create custom projection
    auto customProjection = VampPlannerRegistry::createCustomOrthogonalProjection(
        spaceInfo->getStateSpace(), coordinates, cellSizes);
    
    if (!customProjection) {
        std::cerr << "Failed to create custom projection, using default" << std::endl;
        return registry.createPlanner(plannerName, spaceInfo);
    }
    
    // Register custom projection for this robot+planner combination
    std::string projection_key = plannerName + "_projection";
    registry.registerCustomProjection(robotName, projection_key, customProjection);
    
    // Create planner - custom projection will be applied automatically
    VampPlannerRegistry::PlannerCreationConfig config(plannerName, robotName);
    return registry.createPlanner(config, spaceInfo);
}



} // namespace vamp_ompl
