/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, Rice University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Rice University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Sahruday Patti */
#pragma once

#include "VampOMPLPlanner.h"
#include "VampUtils.h"
#include "VampRobotRegistry.h"
#include "OMPLPlannerRegistry.h"
#include <ompl/tools/benchmark/Benchmark.h>
#include <ompl/geometric/SimpleSetup.h>
#include <memory>
#include <vector>
#include <string>
#include <map>
#include <fstream>
#include <filesystem>
#include <chrono>
#include <atomic>
#include <any>
#include <iomanip>
#include <sstream>

namespace vamp_ompl {
namespace benchmarking {

/**
 * @brief Benchmark configuration extending existing PlanningConfiguration
 * 
 * Uses composition to add benchmark-specific settings
 */
struct BenchmarkConfiguration {

    PlanningConfiguration base_config;
    
    // Benchmark-specific settings
    std::string experiment_name{"VAMP Benchmark"};
    std::vector<std::string> planner_names{"RRT-Connect", "BIT*"};
    std::map<std::string, std::map<std::string, std::string>> planner_parameters;
    unsigned int runs{50};
    double timeout{5.0};
    double memory_limit{4096.0};
    bool simplify_paths{true};
    bool display_progress{true};
    
    // Results export
    std::string results_file{"benchmark_results.log"};
    bool auto_generate_filename{true};
    
    /**
     * @brief Validate configuration using existing patterns
     */
    bool isValid() const {
        return base_config.isValid() && 
               !experiment_name.empty() && 
               !planner_names.empty() && 
               runs > 0 && timeout > 0.0;
    }
    
    /**
     * @brief Get validation errors using existing patterns
     */
    std::string getValidationErrors() const {
        std::string errors = base_config.getValidationErrors();
        if (experiment_name.empty()) errors += "Experiment name required. ";
        if (planner_names.empty()) errors += "At least one planner required. ";
        if (runs == 0) errors += "Runs must be > 0. ";
        if (timeout <= 0.0) errors += "Timeout must be > 0. ";
        return errors;
    }
};

/**
 * @brief benchmark manager following VampOMPLPlanner patterns
 * 
 * This class extends the VampOMPLPlanner pattern to add benchmarking capabilities
 */
template<typename Robot>
class VampBenchmarkManager {
public:
    static constexpr std::size_t robotDimension = Robot::dimension;

private:
    std::unique_ptr<VampOMPLPlanner<Robot>> planner_;
    std::string robot_name_;
    



public:
    /**
     * @brief Constructor using dependency injection
     */
    VampBenchmarkManager(std::unique_ptr<RobotConfig<Robot>> robot_configuration,
                        std::unique_ptr<EnvironmentFactory> environment_factory)
        : planner_(std::make_unique<VampOMPLPlanner<Robot>>(
            std::move(robot_configuration), std::move(environment_factory)))
        , robot_name_(planner_->get_robot_configuration().get_robot_name())
    {
    }

    /**
     * @brief Two-phase initialization
     */
    void initialize() {
        planner_->initialize();
    }

    /**
     * @brief Execute benchmark using OMPL-compliant infrastructure
     * @return Standard OMPL benchmark log file path for compatibility with ompl_benchmark_statistics.py
     */
    std::string executeBenchmark(const BenchmarkConfiguration& config) {
        if (!planner_->is_initialized()) {
            throw VampConfigurationError("Benchmark manager not initialized. Call initialize() first.");
        }

        try {
            // Use the planner's existing space information
            auto spaceInfo = planner_->get_space_information();
            
            // Create OMPL SimpleSetup using existing space information
            auto setup = std::make_shared<ompl::geometric::SimpleSetup>(spaceInfo);
            
            // Set start and goal states from robot configuration
            ompl::base::ScopedState<> start(spaceInfo);
            ompl::base::ScopedState<> goal(spaceInfo);
            
            auto startArray = planner_->get_robot_configuration().get_start_configuration_array();
            auto goalArray = planner_->get_robot_configuration().get_goal_configuration_array();
            
            for (std::size_t i = 0; i < startArray.size(); ++i) {
                start[i] = startArray[i];
                goal[i] = goalArray[i];
            }
            
            setup->setStartAndGoalStates(start, goal);
            
            // Create OMPL benchmark with proper experiment name
            ompl::tools::Benchmark benchmark(*setup, config.experiment_name);
            
            // Add VAMP-specific experiment metadata for analysis
            benchmark.addExperimentParameter("robot_name", "STRING", robot_name_);
            benchmark.addExperimentParameter("robot_dimension", "INTEGER", std::to_string(robotDimension));
            benchmark.addExperimentParameter("environment", "STRING", 
                planner_->get_environment_factory().get_environment_name());
            benchmark.addExperimentParameter("vamp_integration", "STRING", "true");
            
            // Add planners using existing space information
            addPlannersToOMPLBenchmark(benchmark, config, spaceInfo);
            
            // Execute OMPL benchmark with proper request configuration
            ompl::tools::Benchmark::Request omplRequest(
                config.timeout, config.memory_limit, config.runs,
                0.05, config.display_progress, true, config.simplify_paths);
            
            benchmark.benchmark(omplRequest);
            
            // Use OMPL's standard file saving - generates proper log format for database conversion
            std::string logFileName = generateBenchmarkFileName(config);
            benchmark.saveResultsToFile(logFileName.c_str());
            
            std::cout << " OMPL-compliant benchmark results saved to: " << logFileName << std::endl;
            std::cout << "   Run 'ompl_benchmark_statistics.py " << logFileName 
                      << " -d benchmark.db' to generate database" << std::endl;
            std::cout << "   Upload benchmark.db to http://plannerarena.org for visualization" << std::endl;
            
            return logFileName;
            
        } catch (const std::exception& e) {
            throw VampConfigurationError("Benchmark execution failed: " + std::string(e.what()));
        }
    }

    /**
     * @brief Get robot configuration
     */
    const RobotConfig<Robot>& get_robot_configuration() const {
        return planner_->get_robot_configuration();
    }

    /**
     * @brief Get environment factory
     */
    const EnvironmentFactory& get_environment_factory() const {
        return planner_->get_environment_factory();
    }

    /**
     * @brief Check initialization status
     */
    bool is_initialized() const {
        return planner_->is_initialized();
    }

    /**
     * @brief Print configuration summary
     */
    void printConfiguration() const {
        std::cout << "\n=== VAMP Benchmark Manager ===\n";
        std::cout << "Robot: " << robot_name_ << " (dim=" << robotDimension << ")\n";
        std::cout << "Environment: " << get_environment_factory().get_environment_name() << "\n";
        std::cout << "Initialized: " << (is_initialized() ? "Yes" : "No") << "\n";
        std::cout << std::string(30, '=') << std::endl;
    }



private:
    /**
     * @brief Add planners to OMPL benchmark using the planner registry
     */
    void addPlannersToOMPLBenchmark(ompl::tools::Benchmark& benchmark,
                                   const BenchmarkConfiguration& config,
                                   const std::shared_ptr<ompl::base::SpaceInformation>& spaceInfo) {
        
        for (const auto& plannerName : config.planner_names) {
            try {
                // Use the registry to create planners with parameters
                std::map<std::string, std::string> parameters;
                if (config.planner_parameters.count(plannerName)) {
                    parameters = config.planner_parameters.at(plannerName);
                }
                
                auto planner = createPlannerByName(plannerName, spaceInfo, parameters);
                benchmark.addPlanner(planner);
                std::cout << "Added planner: " << plannerName << std::endl;
                
            } catch (const VampConfigurationError& e) {
                std::cerr << "Warning: " << e.what() << std::endl;
                std::cerr << "Available planners: " << PlannerRegistry::getInstance().getAvailablePlannerNames() << std::endl;
            } catch (const std::exception& e) {
                std::cerr << "Warning: Failed to add planner " << plannerName 
                         << ": " << e.what() << std::endl;
            }
        }
    }

    /**
     * @brief Generate OMPL-compliant benchmark filename
     */
    std::string generateBenchmarkFileName(const BenchmarkConfiguration& config) {
        std::string filename = config.results_file;
        
        if (config.auto_generate_filename) {
            auto now = std::chrono::system_clock::now();
            auto time_t = std::chrono::system_clock::to_time_t(now);
            std::stringstream ss;
            ss << "vamp_benchmark_" << robot_name_ 
               << "_" << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S") << ".log";
            filename = ss.str();
        }
        
        // Ensure .log extension for OMPL compatibility
        if (filename.find(".log") == std::string::npos) {
            filename += ".log";
        }
        
        return filename;
    }
};

/**
 * @brief Load benchmark configuration
 */
inline bool loadBenchmarkConfiguration(const std::string& yaml_file, 
                                      BenchmarkConfiguration& config) {
    try {
        // YAML loader for base configuration
        if (!VampUtils::loadYamlConfig(yaml_file, config.base_config)) {
            std::cerr << "Failed to load base configuration from: " << yaml_file << std::endl;
            return false;
        }
        
        // Load benchmark-specific settings
        std::string resolvedPath = VampUtils::findYamlFile(yaml_file);
        YAML::Node root = YAML::LoadFile(resolvedPath);
        
        if (root["benchmark"]) {
            const auto& benchmark = root["benchmark"];
            
            if (benchmark["experiment_name"]) {
                config.experiment_name = benchmark["experiment_name"].as<std::string>();
            }
            if (benchmark["runs"]) {
                config.runs = benchmark["runs"].as<unsigned int>();
            }
            if (benchmark["timeout"]) {
                config.timeout = benchmark["timeout"].as<double>();
            }
            if (benchmark["memory_limit"]) {
                config.memory_limit = benchmark["memory_limit"].as<double>();
            }
            if (benchmark["planners"] && benchmark["planners"].IsSequence()) {
                config.planner_names.clear();
                for (const auto& planner : benchmark["planners"]) {
                    if (planner["name"]) {
                        config.planner_names.push_back(planner["name"].as<std::string>());
                    }
                }
            }
        }
        
        return config.isValid();
        
    } catch (const std::exception& e) {
        std::cerr << "Error loading benchmark configuration: " << e.what() << std::endl;
        return false;
    }
}



/**
 * @brief Helper function to create benchmark manager for a specific robot type
 * 
 * This function provides a way to create a typed benchmark manager
 * without going through the registry pattern.
 */
template<typename Robot>
std::shared_ptr<VampBenchmarkManager<Robot>> createBenchmarkManager(
    std::unique_ptr<RobotConfig<Robot>> robot_config,
    std::unique_ptr<EnvironmentFactory> env_factory) {
    
    return std::make_shared<VampBenchmarkManager<Robot>>(
        std::move(robot_config), std::move(env_factory));
}

} // namespace benchmarking
} // namespace vamp_ompl

// 
// Note: Benchmarking functionality is accessed through VampBenchmarkManager directly.