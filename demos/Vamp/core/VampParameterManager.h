#pragma once

#include "VampOMPLInterfaces.h"
#include <ompl/base/Planner.h>
#include <map>
#include <string>
#include <variant>
#include <vector>
#include <memory>
#include <typeinfo>
#include <iostream>
#include <sstream>

namespace vamp_ompl {

namespace ob = ompl::base;

/**
 * @brief parameter value supporting multiple types with validation
 * 
 * This variant-based system provides type safety and automatic conversion
 * while maintaining flexibility for different parameter types.
 */
using ParameterValue = std::variant<bool, int, float, double, std::string>;

/**
 * @brief Parameter validation rules and metadata
 */
struct ParameterMetadata {
    std::string description;
    ParameterValue default_value;
    std::vector<ParameterValue> valid_values;  // Empty means any value allowed
    std::pair<ParameterValue, ParameterValue> range;  // For numeric types
    bool has_range = false;
    
    ParameterMetadata() = default;
    
    ParameterMetadata(const std::string& desc, const ParameterValue& default_val)
        : description(desc), default_value(default_val) {}
    
    ParameterMetadata(const std::string& desc, const ParameterValue& default_val, 
                     const std::vector<ParameterValue>& valid_vals)
        : description(desc), default_value(default_val), valid_values(valid_vals) {}
    
    ParameterMetadata(const std::string& desc, const ParameterValue& default_val, 
                     const ParameterValue& min_val, const ParameterValue& max_val)
        : description(desc), default_value(default_val), range({min_val, max_val}), has_range(true) {}
};

/**
 * @brief parameter management system with validation and type safety
 * 
 * This system provides:
 * - Type-safe parameter handling with automatic conversion
 * - Parameter validation with range and value checking
 * - Default value management with overrides
 * - Documentation and metadata for each parameter
 * - User-friendly error messages with suggestions
 */
class VampParameterManager {
public:
    using ParameterMap = std::map<std::string, ParameterValue>;
    using MetadataMap = std::map<std::string, ParameterMetadata>;
    
    /**
     * @brief parameter configuration for planners
     */
    struct PlannerParameterConfig {
        std::string planner_name;
        ParameterMap user_parameters;      // User-specified parameters
        ParameterMap default_parameters;   // Default parameters for this planner
        MetadataMap metadata;              // Parameter validation and documentation
        
        /**
         * @brief Get final parameter value with priority: user > default
         * @param param_name Parameter name
         * @return Final parameter value
         */
        ParameterValue getFinalParameter(const std::string& param_name) const {
            if (user_parameters.find(param_name) != user_parameters.end()) {
                return user_parameters.at(param_name);
            }
            if (default_parameters.find(param_name) != default_parameters.end()) {
                return default_parameters.at(param_name);
            }
            if (metadata.find(param_name) != metadata.end()) {
                return metadata.at(param_name).default_value;
            }
            throw VampConfigurationError("Unknown parameter: " + param_name);
        }
        
        /**
         * @brief Get all available parameters with their values
         * @return Map of all parameters
         */
        ParameterMap getAllParameters() const {
            ParameterMap result;
            
            // Start with metadata defaults
            for (const auto& [name, meta] : metadata) {
                result[name] = meta.default_value;
            }
            
            // Override with default parameters
            for (const auto& [name, value] : default_parameters) {
                result[name] = value;
            }
            
            // Override with user parameters (highest priority)
            for (const auto& [name, value] : user_parameters) {
                result[name] = value;
            }
            
            return result;
        }
    };
    
    /**
     * @brief Get singleton instance
     */
    static VampParameterManager& getInstance() {
        static VampParameterManager instance;
        return instance;
    }
    
    /**
     * @brief Create parameter configuration for a planner
     * @param planner_name Name of the planner
     * @param user_params User-specified parameters
     * @return Complete parameter configuration
     */
    PlannerParameterConfig createParameterConfig(const std::string& planner_name,
                                                const std::map<std::string, std::string>& user_params = {}) {
        PlannerParameterConfig config;
        config.planner_name = planner_name;
        
        // Get metadata and defaults for this planner
        config.metadata = getPlannerMetadata(planner_name);
        config.default_parameters = getDefaultParameters(planner_name);
        
        // Convert and validate user parameters
        config.user_parameters = convertAndValidateUserParameters(planner_name, user_params);
        
        return config;
    }
    
    /**
     * @brief Apply parameters to a planner with comprehensive validation
     * @param planner Planner to configure
     * @param config Parameter configuration
     * @return Success status and any error messages
     */
    struct ParameterApplicationResult {
        bool success = true;
        std::vector<std::string> warnings;
        std::vector<std::string> errors;
        std::map<std::string, std::string> applied_parameters;
        std::map<std::string, std::string> failed_parameters;
    };
    
    ParameterApplicationResult applyParameters(ob::Planner* planner, 
                                             const PlannerParameterConfig& config) {
        ParameterApplicationResult result;
        
        if (!planner) {
            result.success = false;
            result.errors.push_back("Planner is null");
            return result;
        }
        
        auto all_params = config.getAllParameters();
        
        for (const auto& [param_name, param_value] : all_params) {
            try {
                std::string param_str = parameterValueToString(param_value);
                bool success = planner->params().setParam(param_name, param_str);
                
                if (success) {
                    result.applied_parameters[param_name] = param_str;
                } else {
                    result.failed_parameters[param_name] = param_str;
                    result.warnings.push_back("Parameter '" + param_name + "' not recognized by planner");
                }
            } catch (const std::exception& e) {
                result.failed_parameters[param_name] = "conversion_error";
                result.errors.push_back("Failed to apply parameter '" + param_name + "': " + e.what());
            }
        }
        
        if (!result.errors.empty()) {
            result.success = false;
        }
        
        return result;
    }
    
    /**
     * @brief Get available parameters for a planner with documentation
     * @param planner_name Name of the planner
     * @return Map of parameter names to metadata
     */
    MetadataMap getAvailableParameters(const std::string& planner_name) {
        return getPlannerMetadata(planner_name);
    }
    
    /**
     * @brief Validate parameter value against metadata
     * @param param_name Parameter name
     * @param param_value Parameter value
     * @param metadata Parameter metadata
     * @return True if valid
     */
    static bool validateParameter(const std::string& /* param_name */, 
                                const ParameterValue& param_value,
                                const ParameterMetadata& metadata) {
        // Check valid values if specified
        if (!metadata.valid_values.empty()) {
            auto it = std::find(metadata.valid_values.begin(), metadata.valid_values.end(), param_value);
            if (it == metadata.valid_values.end()) {
                return false;
            }
        }
        
        // Check range if specified
        if (metadata.has_range) {
            return isValueInRange(param_value, metadata.range.first, metadata.range.second);
        }
        
        return true;
    }
    
private:
    std::map<std::string, MetadataMap> planner_metadata_;
    std::map<std::string, ParameterMap> default_parameters_;
    
    VampParameterManager() {
        initializePlannerMetadata();
        initializeDefaultParameters();
    }
    
    /**
     * @brief Initialize parameter metadata for all planners
     */
    void initializePlannerMetadata() {
        // RRT Family - Based on actual OMPL declareParam calls
        planner_metadata_["RRT"] = {
            {"range", ParameterMetadata("Maximum step size for tree extension", 0.1, 0.01, 1.0)},
            {"goal_bias", ParameterMetadata("Probability of selecting goal state", 0.05, 0.0, 1.0)},
            {"intermediate_states", ParameterMetadata("Compute intermediate states during motion", true)}
        };
        
        planner_metadata_["RRT-Connect"] = {
            {"range", ParameterMetadata("Maximum step size for tree extension", 0.1, 0.01, 1.0)}
        };
        
        planner_metadata_["RRT*"] = {
            {"range", ParameterMetadata("Maximum step size for tree extension", 0.1, 0.01, 1.0)},
            {"goal_bias", ParameterMetadata("Probability of selecting goal state", 0.05, 0.0, 1.0)},
            {"rewire_factor", ParameterMetadata("Multiplicative factor for rewiring radius", 1.1, 1.0, 3.0)},
            {"use_k_nearest", ParameterMetadata("Use k-nearest instead of radius", true)},
            {"delay_collision_checking", ParameterMetadata("Delay collision checking for better performance", true)},
            {"tree_pruning", ParameterMetadata("Enable tree pruning", false)},
            {"prune_threshold", ParameterMetadata("Threshold for pruning", 0.05, 0.0, 1.0)},
            {"pruned_measure", ParameterMetadata("Use pruned measure", false)},
            {"informed_sampling", ParameterMetadata("Use informed sampling", true)},
            {"sample_rejection", ParameterMetadata("Use sample rejection", false)},
            {"new_state_rejection", ParameterMetadata("Use new state rejection", false)},
            {"use_admissible_heuristic", ParameterMetadata("Use admissible heuristic", false)},
            {"ordered_sampling", ParameterMetadata("Use ordered sampling", false)},
            {"ordering_batch_size", ParameterMetadata("Batch size for ordering", 100, 1, 1000000)},
            {"focus_search", ParameterMetadata("Focus search using heuristic", true)},
            {"number_sampling_attempts", ParameterMetadata("Number of sampling attempts", 100, 10, 100000)}
        };
        
        // InformedRRTstar has subset of RRT* parameters - only the ones it actually supports
        planner_metadata_["InformedRRTstar"] = {
            {"range", ParameterMetadata("Maximum step size for tree extension", 0.1, 0.01, 1.0)},
            {"goal_bias", ParameterMetadata("Probability of selecting goal state", 0.05, 0.0, 1.0)},
            {"rewire_factor", ParameterMetadata("Multiplicative factor for rewiring radius", 1.1, 1.0, 3.0)},
            {"use_k_nearest", ParameterMetadata("Use k-nearest instead of radius", true)},
            {"delay_collision_checking", ParameterMetadata("Delay collision checking for better performance", true)},
            {"prune_threshold", ParameterMetadata("Threshold for pruning", 0.05, 0.0, 1.0)},
            {"number_sampling_attempts", ParameterMetadata("Number of sampling attempts", 100, 10, 100000)}
        };
        
        planner_metadata_["TRRT"] = {
            {"range", ParameterMetadata("Maximum step size for tree extension", 0.1, 0.01, 1.0)},
            {"goal_bias", ParameterMetadata("Probability of selecting goal state", 0.05, 0.0, 1.0)},
            {"temp_change_factor", ParameterMetadata("Temperature change factor", 0.1, 0.01, 1.0)},
            {"init_temperature", ParameterMetadata("Initial temperature", 10e-6, 1e-10, 1.0)},
            {"frontier_threshold", ParameterMetadata("Frontier threshold", 0.0, 0.0, 1.0)},
            {"frontier_node_ratio", ParameterMetadata("Frontier node ratio", 0.1, 0.0, 1.0)},
            {"cost_threshold", ParameterMetadata("Cost threshold for transition test", 1e300, 0.0, 1e300)}
        };
        
        // BIT* Family - Based on actual OMPL parameters
        planner_metadata_["BIT*"] = {
            {"rewire_factor", ParameterMetadata("Multiplicative factor for rewiring radius", 1.1, 1.0, 3.0)},
            {"samples_per_batch", ParameterMetadata("Number of samples per batch", 100, 1, 10000)},
            {"use_k_nearest", ParameterMetadata("Use k-nearest instead of radius", true)},
            {"use_graph_pruning", ParameterMetadata("Enable graph pruning", true)},
            {"prune_threshold_as_fractional_cost_change", ParameterMetadata("Prune threshold as fractional cost change", 0.05, 0.0, 1.0)},
            {"delay_rewiring_to_first_solution", ParameterMetadata("Delay rewiring until first solution", false)},
            {"use_just_in_time_sampling", ParameterMetadata("Use just-in-time sampling", true)},
            {"drop_unconnected_samples_on_prune", ParameterMetadata("Drop unconnected samples on prune", false)},
            {"stop_on_each_solution_improvement", ParameterMetadata("Stop on each solution improvement", false)},
            {"use_strict_queue_ordering", ParameterMetadata("Use strict queue ordering", false)},
            {"find_approximate_solutions", ParameterMetadata("Find approximate solutions", false)}
        };
        
        planner_metadata_["ABITstar"] = planner_metadata_["BIT*"];  // Inherits BIT* parameters
        
        // KPIECE Family - Based on actual OMPL parameters
        planner_metadata_["KPIECE"] = {
            {"range", ParameterMetadata("Maximum step size for tree extension", 0.1, 0.01, 1.0)},
            {"goal_bias", ParameterMetadata("Probability of selecting goal state", 0.05, 0.0, 1.0)},
            {"border_fraction", ParameterMetadata("Fraction of time focused on border", 0.9, 0.0, 1.0)},
            {"failed_expansion_score_factor", ParameterMetadata("Factor for failed expansion score", 0.5, 0.0, 1.0)},
            {"min_valid_path_fraction", ParameterMetadata("Minimum valid path fraction", 0.2, 0.0, 1.0)}
        };
        
        // BKPIECE - Only has range, border_fraction, failed_expansion_score_factor, min_valid_path_fraction (no goal_bias)
        planner_metadata_["BKPIECE"] = {
            {"range", ParameterMetadata("Maximum step size for tree extension", 0.1, 0.01, 1.0)},
            {"border_fraction", ParameterMetadata("Fraction of time focused on border", 0.9, 0.0, 1.0)},
            {"failed_expansion_score_factor", ParameterMetadata("Factor for failed expansion score", 0.5, 0.0, 1.0)},
            {"min_valid_path_fraction", ParameterMetadata("Minimum valid path fraction", 0.2, 0.0, 1.0)}
        };
        
        // LBKPIECE - Only has range, border_fraction, min_valid_path_fraction (no goal_bias, no failed_expansion_score_factor)
        planner_metadata_["LBKPIECE"] = {
            {"range", ParameterMetadata("Maximum step size for tree extension", 0.1, 0.01, 1.0)},
            {"border_fraction", ParameterMetadata("Fraction of time focused on border", 0.9, 0.0, 1.0)},
            {"min_valid_path_fraction", ParameterMetadata("Minimum valid path fraction", 0.2, 0.0, 1.0)}
        };
        
        // PRM Family - Based on actual OMPL parameters
        planner_metadata_["PRM"] = {
            {"max_nearest_neighbors", ParameterMetadata("Maximum nearest neighbors", 10, 1, 100)}
        };
        
        planner_metadata_["LazyPRM"] = planner_metadata_["PRM"];
        
        // PRMstar and LazyPRMstar automatically compute optimal connections, no max_nearest_neighbors
        planner_metadata_["PRMstar"] = {};  // No parameters in OMPL
        planner_metadata_["LazyPRMstar"] = {};  // No parameters in OMPL
        
        // EST Family - Based on actual OMPL parameters
        planner_metadata_["EST"] = {
            {"range", ParameterMetadata("Maximum step size for tree extension", 0.1, 0.01, 1.0)},
            {"goal_bias", ParameterMetadata("Probability of selecting goal state", 0.05, 0.0, 1.0)}
        };
        
        // BiEST - Only has range parameter (no goal_bias)
        planner_metadata_["BiEST"] = {
            {"range", ParameterMetadata("Maximum step size for tree extension", 0.1, 0.01, 1.0)}
        };
        
        // ProjEST - Same as EST
        planner_metadata_["ProjEST"] = planner_metadata_["EST"];
        
        // SBL Family
        planner_metadata_["SBL"] = {
            {"range", ParameterMetadata("Maximum step size for tree extension", 0.1, 0.01, 1.0)}
        };
        planner_metadata_["pSBL"] = planner_metadata_["SBL"];
        
        // FMT Family - Based on actual OMPL parameters
        planner_metadata_["FMT"] = {
            {"num_samples", ParameterMetadata("Number of samples", 1000, 10, 100000)},
            {"radius_multiplier", ParameterMetadata("Radius multiplier", 1.1, 1.0, 5.0)},
            {"use_k_nearest", ParameterMetadata("Use k-nearest instead of radius", true)},
            {"cache_cc", ParameterMetadata("Cache collision checks", true)},
            {"heuristics", ParameterMetadata("Use heuristics", true)},
            {"extended_fmt", ParameterMetadata("Use extended FMT", true)}
        };
        
        // BFMT has different parameters - need to check
        planner_metadata_["BFMT"] = {
            {"num_samples", ParameterMetadata("Number of samples", 1000, 10, 100000)},
            {"radius_multiplier", ParameterMetadata("Radius multiplier", 1.1, 1.0, 5.0)},
            {"nearest_k", ParameterMetadata("Use k-nearest", true)},
            {"cache_cc", ParameterMetadata("Cache collision checks", true)},
            {"heuristics", ParameterMetadata("Use heuristics", true)},
            {"extended_fmt", ParameterMetadata("Use extended FMT", true)}
        };
        
        // SST
        planner_metadata_["SST"] = {
            {"range", ParameterMetadata("Maximum step size", 0.1, 0.01, 1.0)},
            {"goal_bias", ParameterMetadata("Goal bias", 0.05, 0.0, 1.0)},
            {"selection_radius", ParameterMetadata("Selection radius", 0.1, 0.01, 1.0)},
            {"pruning_radius", ParameterMetadata("Pruning radius", 0.1, 0.01, 1.0)}
        };
        
        // Initialize specialized planners with complex parameter sets
        initializeSpecializedPlannerMetadata();
    }
    
    /**
     * @brief Initialize parameter metadata for specialized planners
     * 
     * This method handles planners with complex parameter sets that require
     * specialized configuration, including multi-query planners (SPARS family),
     * meta-planners, and planners with minimal parameter requirements.
     */
    void initializeSpecializedPlannerMetadata() {
        // SPARS Family
        planner_metadata_["SPARS"] = {
            {"stretch_factor", ParameterMetadata("Stretch factor", 3.0, 1.0, 10.0)},
            {"sparse_delta_fraction", ParameterMetadata("Sparse delta fraction", 0.25, 0.01, 1.0)},
            {"dense_delta_fraction", ParameterMetadata("Dense delta fraction", 0.001, 0.0001, 0.1)},
            {"max_failures", ParameterMetadata("Maximum failures", 1000, 1, 10000)}
        };
        planner_metadata_["SPARStwo"] = planner_metadata_["SPARS"];
        
        // Planners with minimal or no specific parameters
        planner_metadata_["BiTRRT"] = {
            {"range", ParameterMetadata("Maximum step size for tree extension", 0.1, 0.01, 1.0)}
        };
        
        planner_metadata_["LBTRRT"] = {
            {"range", ParameterMetadata("Maximum step size for tree extension", 0.1, 0.01, 1.0)}
        };
        
        // XXL commented out - requires special decomposition setup not implemented
        // planner_metadata_["XXL"] = {
        //     {"rand_walk_rate", ParameterMetadata("Random walk rate", 0.05, 0.0, 1.0)}
        // };
        
        // Many planners have no specific parameters beyond what OMPL provides by default
        // RRTXstatic - Advanced RRT* variant with many parameters
        planner_metadata_["RRTXstatic"] = {
            {"range", ParameterMetadata("Maximum step size", 0.3, 0.01, 1.0)},
            {"goal_bias", ParameterMetadata("Goal bias", 0.05, 0.0, 1.0)},
            {"epsilon", ParameterMetadata("Minimum cost improvement threshold", 0.01, 0.001, 1.0)},
            {"rewire_factor", ParameterMetadata("Rewiring factor", 1.1, 1.0, 2.0)},
            {"use_k_nearest", ParameterMetadata("Use k-nearest instead of radius", true)},
            {"update_children", ParameterMetadata("Force cost propagation to children", true)},
            {"rejection_variant", ParameterMetadata("Rejection sampling variant (0-3)", 0, 0, 3)},
            {"rejection_variant_alpha", ParameterMetadata("Rejection sampling alpha", 0.0, 0.0, 1.0)},
            {"informed_sampling", ParameterMetadata("Use informed sampling", true)},
            {"sample_rejection", ParameterMetadata("Use sample rejection", true)},
            {"number_sampling_attempts", ParameterMetadata("Number of sampling attempts", 100, 10, 1000)}
        };
        
        // pRRT - Parallel RRT
        planner_metadata_["pRRT"] = {
            {"range", ParameterMetadata("Maximum step size", 0.3, 0.01, 1.0)},
            {"goal_bias", ParameterMetadata("Goal bias", 0.05, 0.0, 1.0)},
            {"thread_count", ParameterMetadata("Number of threads", 2, 1, 64)}
        };
        
        // STRIDE - Many parameters for GNAT configuration
        planner_metadata_["STRIDE"] = {
            {"range", ParameterMetadata("Maximum step size", 0.3, 0.01, 1.0)},
            {"goal_bias", ParameterMetadata("Goal bias", 0.05, 0.0, 1.0)},
            {"use_projected_distance", ParameterMetadata("Use projected distance", false)},
            {"degree", ParameterMetadata("GNAT degree", 16, 2, 20)},
            {"max_degree", ParameterMetadata("GNAT max degree", 18, 2, 20)},
            {"min_degree", ParameterMetadata("GNAT min degree", 12, 2, 20)},
            {"max_pts_per_leaf", ParameterMetadata("Max points per leaf", 6, 1, 200)},
            {"estimated_dimension", ParameterMetadata("Estimated dimension", 0.0, 1.0, 30.0)},
            {"min_valid_path_fraction", ParameterMetadata("Min valid path fraction", 0.05, 0.0, 1.0)}
        };
        
        // PDST - Simple planner with one parameter
        planner_metadata_["PDST"] = {
            {"goal_bias", ParameterMetadata("Goal bias", 0.05, 0.0, 1.0)}
        };
        
        // RRTsharp - RRTXstatic with epsilon=0 (inherits all RRTXstatic parameters except epsilon)
        planner_metadata_["RRTsharp"] = {
            {"range", ParameterMetadata("Maximum step size", 0.3, 0.01, 1.0)},
            {"goal_bias", ParameterMetadata("Goal bias", 0.05, 0.0, 1.0)},
            {"rewire_factor", ParameterMetadata("Rewiring factor", 1.1, 1.0, 2.0)},
            {"use_k_nearest", ParameterMetadata("Use k-nearest instead of radius", true)},
            {"update_children", ParameterMetadata("Force cost propagation to children", true)},
            {"rejection_variant", ParameterMetadata("Rejection sampling variant (0-3)", 0, 0, 3)},
            {"rejection_variant_alpha", ParameterMetadata("Rejection sampling alpha", 0.0, 0.0, 1.0)},
            {"informed_sampling", ParameterMetadata("Use informed sampling", true)},
            {"sample_rejection", ParameterMetadata("Use sample rejection", true)},
            {"number_sampling_attempts", ParameterMetadata("Number of sampling attempts", 100, 10, 1000)}
        };
        
        // SORRTstar - InformedRRTstar with ordered sampling (inherits many RRT* parameters)
        planner_metadata_["SORRTstar"] = {
            {"range", ParameterMetadata("Maximum step size", 0.3, 0.01, 1.0)},
            {"goal_bias", ParameterMetadata("Goal bias", 0.05, 0.0, 1.0)},
            {"rewire_factor", ParameterMetadata("Rewiring factor", 1.1, 1.0, 2.0)},
            {"use_k_nearest", ParameterMetadata("Use k-nearest instead of radius", true)},
            {"delay_collision_checking", ParameterMetadata("Delay collision checking", false)},
            {"tree_pruning", ParameterMetadata("Enable tree pruning", false)},
            {"prune_threshold", ParameterMetadata("Pruning threshold", 0.05, 0.0, 1.0)},
            {"pruned_measure", ParameterMetadata("Use pruned measure", false)},
            {"informed_sampling", ParameterMetadata("Use informed sampling", true)},
            {"sample_rejection", ParameterMetadata("Use sample rejection", true)},
            {"new_state_rejection", ParameterMetadata("Use new state rejection", false)},
            {"use_admissible_heuristic", ParameterMetadata("Use admissible heuristic", false)},
            {"ordering_batch_size", ParameterMetadata("Batch size for ordering", 100, 1, 1000)},
            {"focus_search", ParameterMetadata("Focus search", false)},
            {"number_sampling_attempts", ParameterMetadata("Number of sampling attempts", 100, 10, 1000)}
        };
        
        // LazyRRT - Simple lazy RRT with basic parameters
        planner_metadata_["LazyRRT"] = {
            {"range", ParameterMetadata("Maximum step size", 0.3, 0.01, 1.0)},
            {"goal_bias", ParameterMetadata("Goal bias", 0.05, 0.0, 1.0)}
        };
        
        // LazyLBTRRT - Lazy Lower Bound Tree RRT
        planner_metadata_["LazyLBTRRT"] = {
            {"range", ParameterMetadata("Maximum step size", 0.3, 0.01, 1.0)},
            {"goal_bias", ParameterMetadata("Goal bias", 0.05, 0.0, 1.0)},
            {"epsilon", ParameterMetadata("Approximation factor", 0.1, 0.0, 10.0)}
        };
        
        // AIT* - Adaptively Informed Trees with 6 parameters
        planner_metadata_["AIT*"] = {
            {"use_k_nearest", ParameterMetadata("Use k-nearest instead of radius", true)},
            {"rewire_factor", ParameterMetadata("Rewiring factor", 1.1, 1.0, 3.0)},
            {"samples_per_batch", ParameterMetadata("Samples per batch", 100, 1, 1000)},
            {"use_graph_pruning", ParameterMetadata("Enable graph pruning", true)},
            {"find_approximate_solutions", ParameterMetadata("Find approximate solutions", true)},
            {"set_max_num_goals", ParameterMetadata("Maximum number of goals", 1, 1, 1000)}
        };
        
        // EIT* - Effort Informed Trees with 6 parameters
        planner_metadata_["EIT*"] = {
            {"use_k_nearest", ParameterMetadata("Use k-nearest instead of radius", true)},
            {"rewire_factor", ParameterMetadata("Rewiring factor", 1.1, 1.0, 3.0)},
            {"batch_size", ParameterMetadata("Batch size", 100, 1, 10000)},
            {"use_graph_pruning", ParameterMetadata("Enable graph pruning", true)},
            {"find_approximate_solutions", ParameterMetadata("Find approximate solutions", true)},
            {"set_max_num_goals", ParameterMetadata("Maximum number of goals", 1, 1, 1000)}
        };
        
        // AnytimePathShortening - Meta-planner with 5 parameters
        planner_metadata_["AnytimePathShortening"] = {
            {"shortcut", ParameterMetadata("Enable shortcutting", true)},
            {"hybridize", ParameterMetadata("Enable hybridization", true)},
            {"max_hybrid_paths", ParameterMetadata("Max hybridization paths", 24, 0, 50)},
            {"num_planners", ParameterMetadata("Number of planners", 0, 0, 64)},
            {"planners", ParameterMetadata("Planner list", std::string(""))}
        };
        
        // XXL - High-dimensional manipulation planner with 1 parameter
        planner_metadata_["XXL"] = {
            {"rand_walk_rate", ParameterMetadata("Random walk rate for exploration", 0.05, 0.0, 1.0)}
        };
        
        // TSRRT - Task-space RRT with 2 parameters
        planner_metadata_["TSRRT"] = {
            {"range", ParameterMetadata("Maximum distance between consecutive states", 0.1, 0.001, 10.0)},
            {"goal_bias", ParameterMetadata("Probability of selecting goal state", 0.05, 0.0, 1.0)}
        };
        
        // EIRMstar - EIT* with additional start/goal pruning (inherits all EIT* parameters + 1)
        planner_metadata_["EIRMstar"] = {
            {"use_k_nearest", ParameterMetadata("Use k-nearest instead of radius", true)},
            {"rewire_factor", ParameterMetadata("Rewiring factor", 1.1, 1.0, 3.0)},
            {"batch_size", ParameterMetadata("Batch size", 100, 1, 10000)},
            {"use_graph_pruning", ParameterMetadata("Enable graph pruning", true)},
            {"find_approximate_solutions", ParameterMetadata("Find approximate solutions", true)},
            {"set_max_num_goals", ParameterMetadata("Maximum number of goals", 1, 1, 1000)},
            {"set_start_goal_pruning", ParameterMetadata("Start/goal pruning threshold", 1, 1, 100000)}
        };
        
        // RLRT - Range-Limited Random Tree with 3 parameters
        planner_metadata_["RLRT"] = {
            {"goal_bias", ParameterMetadata("Goal bias", 0.05, 0.0, 1.0)},
            {"range", ParameterMetadata("Maximum step size", 0.3, 0.01, 1.0)},
            {"keep_last_valid", ParameterMetadata("Keep last valid state", false)}
        };
        
        // BiRLRT - Bidirectional RLRT with 3 parameters
        planner_metadata_["BiRLRT"] = {
            {"range", ParameterMetadata("Maximum step size", 0.3, 0.01, 1.0)},
            {"max_dist_near", ParameterMetadata("Maximum distance for near states", 0.1, 0.0, 10.0)},
            {"keep_last_valid", ParameterMetadata("Keep last valid state", false)}
        };
        
        // CForest - Multi-threaded meta-planner with 2 parameters
        planner_metadata_["CForest"] = {
            {"focus_search", ParameterMetadata("Enable focused search", true)},
            {"num_threads", ParameterMetadata("Number of threads", 0, 0, 64)}
        };
    }
    
    void initializeDefaultParameters() {
        // Initialize default parameters based on metadata
        for (const auto& [planner_name, metadata] : planner_metadata_) {
            ParameterMap defaults;
            for (const auto& [param_name, meta] : metadata) {
                defaults[param_name] = meta.default_value;
            }
            default_parameters_[planner_name] = defaults;
        }
    }
    

    
    MetadataMap getPlannerMetadata(const std::string& planner_name) {
        auto it = planner_metadata_.find(planner_name);
        if (it != planner_metadata_.end()) {
            return it->second;
        }
        
        // Return empty metadata for unknown planners (they'll use OMPL defaults)
        return MetadataMap{};
    }
    
    ParameterMap getDefaultParameters(const std::string& planner_name) {
        auto it = default_parameters_.find(planner_name);
        if (it != default_parameters_.end()) {
            return it->second;
        }
        return ParameterMap{};
    }
    

    
    ParameterMap convertAndValidateUserParameters(const std::string& planner_name,
                                                 const std::map<std::string, std::string>& user_params) {
        ParameterMap result;
        auto metadata = getPlannerMetadata(planner_name);
        
        for (const auto& [param_name, param_str] : user_params) {
            try {
                // Convert string to appropriate type
                ParameterValue param_value = stringToParameterValue(param_str, param_name, metadata);
                
                // Validate if metadata exists
                if (metadata.find(param_name) != metadata.end()) {
                    if (!validateParameter(param_name, param_value, metadata.at(param_name))) {
                        throw VampConfigurationError("Invalid value for parameter '" + param_name + 
                                                   "': " + param_str);
                    }
                }
                
                result[param_name] = param_value;
            } catch (const std::exception& e) {
                throw VampConfigurationError("Error processing parameter '" + param_name + 
                                           "' with value '" + param_str + "': " + e.what());
            }
        }
        
        return result;
    }
    
    /**
     * @brief Convert string to appropriate parameter value type
     */
    ParameterValue stringToParameterValue(const std::string& str, const std::string& param_name,
                                        const MetadataMap& metadata) {
        // If we have metadata, use the default type
        if (metadata.find(param_name) != metadata.end()) {
            const auto& default_val = metadata.at(param_name).default_value;
            
            if (std::holds_alternative<bool>(default_val)) {
                return stringToBool(str);
            } else if (std::holds_alternative<int>(default_val)) {
                return std::stoi(str);
            } else if (std::holds_alternative<float>(default_val)) {
                return std::stof(str);
            } else if (std::holds_alternative<double>(default_val)) {
                return std::stod(str);
            }
        }
        
        if (str == "true" || str == "false") {
            return stringToBool(str);
        }
        
        // Try numeric conversion
        try {
            if (str.find('.') != std::string::npos) {
                return std::stod(str);  // Double for decimals
            } else {
                return std::stoi(str);  // Int for whole numbers
            }
        } catch (...) {
            return str;  // Keep as string if conversion fails
        }
    }
    
    bool stringToBool(const std::string& str) {
        std::string lower_str = str;
        std::transform(lower_str.begin(), lower_str.end(), lower_str.begin(), ::tolower);
        
        if (lower_str == "true" || lower_str == "1" || lower_str == "yes" || lower_str == "on") {
            return true;
        } else if (lower_str == "false" || lower_str == "0" || lower_str == "no" || lower_str == "off") {
            return false;
        } else {
            throw std::invalid_argument("Invalid boolean value: " + str);
        }
    }
    
    std::string parameterValueToString(const ParameterValue& value) {
        return std::visit([](auto&& arg) -> std::string {
            using T = std::decay_t<decltype(arg)>;
            if constexpr (std::is_same_v<T, bool>) {
                return arg ? "true" : "false";
            } else if constexpr (std::is_same_v<T, std::string>) {
                return arg;
            } else {
                return std::to_string(arg);
            }
        }, value);
    }
    
    static bool isValueInRange(const ParameterValue& value, const ParameterValue& min_val, const ParameterValue& max_val) {
        return std::visit([](auto&& val, auto&& min_v, auto&& max_v) -> bool {
            using T = std::decay_t<decltype(val)>;
            using MinT = std::decay_t<decltype(min_v)>;
            using MaxT = std::decay_t<decltype(max_v)>;
            
            if constexpr (std::is_arithmetic_v<T> && std::is_arithmetic_v<MinT> && std::is_arithmetic_v<MaxT>) {
                return static_cast<double>(val) >= static_cast<double>(min_v) && 
                       static_cast<double>(val) <= static_cast<double>(max_v);
            }
            return true; // Can't validate non-numeric types
        }, value, min_val, max_val);
    }
};

/**
 * @brief Convenience function to create parameter configuration
 * @param planner_name Name of the planner
 * @param user_params User-specified parameters
 * @return Complete parameter configuration
 */
inline VampParameterManager::PlannerParameterConfig createParameterConfiguration(
    const std::string& planner_name,
    const std::map<std::string, std::string>& user_params = {}) {
    return VampParameterManager::getInstance().createParameterConfig(planner_name, user_params);
}

/**
 * @brief Convenience function to apply parameters to a planner
 * @param planner Planner to configure
 * @param config Parameter configuration
 * @return Application result
 */
inline VampParameterManager::ParameterApplicationResult applyParametersToPlanner(
    ob::Planner* planner, 
    const VampParameterManager::PlannerParameterConfig& config) {
    return VampParameterManager::getInstance().applyParameters(planner, config);
}

} // namespace vamp_ompl
