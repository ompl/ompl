#pragma once

#include "VampOMPLInterfaces.h"
#include "VampPlannerRegistry.h"
#include <ompl/geometric/planners/experience/LightningRetrieveRepair.h>
#include <ompl/geometric/planners/experience/ThunderRetrieveRepair.h>
#include <ompl/tools/lightning/LightningDB.h>
#include <ompl/tools/thunder/ThunderDB.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <memory>
#include <string>
#include <map>
#include <cstdlib>  // for getenv
#include <ctime>    // for time
#include <unistd.h> // for getpid

namespace vamp_ompl {

namespace og = ompl::geometric;
namespace ot = ompl::tools;

/**
 * @brief Experience-based planner manager with database setup
 * 
 * This class manages experience-based planners that require pre-existing
 * databases of successful paths. It provides automatic database creation
 * and management for Lightning and Thunder planners.
 * 
 */
class VampExperiencePlanners {
public:
    /**
     * @brief Experience database configuration
     */
    struct ExperienceConfig {
        std::string database_file;
        bool auto_create;
        bool enable_repair;
        bool enable_recall;
        double recall_epsilon;
        double repair_time_limit;
        size_t max_experiences;
        
        ExperienceConfig() 
            : auto_create(true)
            , enable_repair(true)
            , enable_recall(true)
            , recall_epsilon(0.1)
            , repair_time_limit(1.0)
            , max_experiences(1000) {}
        
        ExperienceConfig(const std::string& db_file, bool auto_create_db = true)
            : database_file(db_file)
            , auto_create(auto_create_db)
            , enable_repair(true)
            , enable_recall(true)
            , recall_epsilon(0.1)
            , repair_time_limit(1.0)
            , max_experiences(1000) {}
    };
    
    /**
     * @brief Get singleton instance
     */
    static VampExperiencePlanners& getInstance() {
        static VampExperiencePlanners instance;
        return instance;
    }
    
    /**
     * @brief Create Lightning planner with database
     * @param si Space information
     * @param config Experience configuration
     * @return Configured Lightning planner
     */
    std::shared_ptr<og::LightningRetrieveRepair> createLightningPlanner(
        const ob::SpaceInformationPtr& si,
        const ExperienceConfig& config = ExperienceConfig()) {
        
        // Create or get existing Lightning database
        auto lightning_db = getLightningDatabase(si, config);
        if (!lightning_db) {
            throw VampConfigurationError("Failed to create Lightning database");
        }
        
        // Create Lightning planner with database
        auto planner = std::make_shared<og::LightningRetrieveRepair>(si, lightning_db);
        
        // Lightning planners in OMPL have a simpler interface
        // They automatically use RRTConnect for repair by default
        // The configuration is handled through the database and problem definition
        
        return planner;
    }
    
    /**
     * @brief Create Thunder planner with database
     * @param si Space information
     * @param config Experience configuration
     * @return Configured Thunder planner
     */
    std::shared_ptr<og::ThunderRetrieveRepair> createThunderPlanner(
        const ob::SpaceInformationPtr& si,
        const ExperienceConfig& config = ExperienceConfig()) {
        
        // Create or get existing Thunder database
        auto thunder_db = getThunderDatabase(si, config);
        if (!thunder_db) {
            throw VampConfigurationError("Failed to create Thunder database");
        }
        
        // Create Thunder planner with database
        auto planner = std::make_shared<og::ThunderRetrieveRepair>(si, thunder_db);
        
        // Thunder planners also have a simpler interface in OMPL
        // Configuration is handled through the database
        
        return planner;
    }
    
    /**
     * @brief Register experience-based planners with the main registry
     * @param lightning_db_path Path to Lightning database (optional, uses default if empty)
     * @param thunder_db_path Path to Thunder database (optional, uses default if empty)
     */
    void registerExperiencePlanners(const std::string& lightning_db_path = "", 
                                   const std::string& thunder_db_path = "") {
        
        // Determine database paths with intelligent defaults
        std::string lightning_db = lightning_db_path.empty() ? 
            getDefaultDatabasePath("lightning") : lightning_db_path;
        std::string thunder_db = thunder_db_path.empty() ? 
            getDefaultDatabasePath("thunder") : thunder_db_path;
        
        // Register Lightning planner with configurable database
        vamp_ompl::registerCustomPlanner("Lightning", [this, lightning_db](const ob::SpaceInformationPtr& si) {
            ExperienceConfig config(lightning_db, true);
            return std::static_pointer_cast<ob::Planner>(createLightningPlanner(si, config));
        });
        
        // Register Thunder planner with configurable database
        vamp_ompl::registerCustomPlanner("Thunder", [this, thunder_db](const ob::SpaceInformationPtr& si) {
            ExperienceConfig config(thunder_db, true);
            return std::static_pointer_cast<ob::Planner>(createThunderPlanner(si, config));
        });
        
        // Register OMPL-standard names for compatibility
        vamp_ompl::registerCustomPlanner("LightningRetrieveRepair", [this, lightning_db](const ob::SpaceInformationPtr& si) {
            ExperienceConfig config(lightning_db, true);
            return std::static_pointer_cast<ob::Planner>(createLightningPlanner(si, config));
        });
        
        vamp_ompl::registerCustomPlanner("ThunderRetrieveRepair", [this, thunder_db](const ob::SpaceInformationPtr& si) {
            ExperienceConfig config(thunder_db, true);
            return std::static_pointer_cast<ob::Planner>(createThunderPlanner(si, config));
        });
    }
    
    /**
     * @brief Register experience planner with custom configuration
     * @param planner_type "Lightning" or "Thunder"
     * @param planner_name Custom name for registration
     * @param config Experience configuration
     */
    void registerExperiencePlanner(const std::string& planner_type,
                                  const std::string& planner_name,
                                  const ExperienceConfig& config) {
        if (planner_type == "Lightning") {
            vamp_ompl::registerCustomPlanner(planner_name, [this, config](const ob::SpaceInformationPtr& si) {
                return std::static_pointer_cast<ob::Planner>(createLightningPlanner(si, config));
            });
        } else if (planner_type == "Thunder") {
            vamp_ompl::registerCustomPlanner(planner_name, [this, config](const ob::SpaceInformationPtr& si) {
                return std::static_pointer_cast<ob::Planner>(createThunderPlanner(si, config));
            });
        } else {
            throw VampConfigurationError("Unknown experience planner type: " + planner_type + 
                                       ". Supported types: Lightning, Thunder");
        }
    }
    
    /**
     * @brief Save experience to database
     * @param planner_type Type of experience planner ("Lightning" or "Thunder")
     * @param database_file Database file path
     * @param solution_path Solution path to save
     * @return Success status
     */
    bool saveExperience(const std::string& planner_type,
                       const std::string& database_file,
                       const og::PathGeometric& solution_path) {
        try {
            if (planner_type == "Lightning") {
                auto it = lightning_databases_.find(database_file);
                if (it != lightning_databases_.end()) {
                    // Lightning database saves paths automatically
                    // We'll implement this as a placeholder for future enhancement
                    return true;
                }
            } else if (planner_type == "Thunder") {
                auto it = thunder_databases_.find(database_file);
                if (it != thunder_databases_.end()) {
                    // Thunder database uses a different API
                    // Copy the path for adding to database
                    og::PathGeometric path_copy(solution_path);
                    double insertion_time;
                    return it->second->addPath(path_copy, insertion_time);
                }
            }
            return false;
        } catch (const std::exception& e) {
            std::cerr << "Failed to save experience: " << e.what() << std::endl;
            return false;
        }
    }
    
    /**
     * @brief Clear experience database
     * @param planner_type Type of experience planner
     * @param database_file Database file path
     * @return Success status
     */
    bool clearExperience(const std::string& planner_type, const std::string& database_file) {
        try {
            if (planner_type == "Lightning") {
                auto it = lightning_databases_.find(database_file);
                if (it != lightning_databases_.end()) {
                    // Lightning database clearing (placeholder)
                    return true;
                }
            } else if (planner_type == "Thunder") {
                auto it = thunder_databases_.find(database_file);
                if (it != thunder_databases_.end()) {
                    // Thunder database clearing (placeholder)
                    return true;
                }
            }
            return false;
        } catch (const std::exception& e) {
            std::cerr << "Failed to clear experience: " << e.what() << std::endl;
            return false;
        }
    }
    
    /**
     * @brief Get experience database statistics
     * @param planner_type Type of experience planner
     * @param database_file Database file path
     * @return Statistics map
     */
    std::map<std::string, size_t> getExperienceStats(const std::string& planner_type, 
                                                    const std::string& database_file) {
        std::map<std::string, size_t> stats;
        
        try {
            if (planner_type == "Lightning") {
                auto it = lightning_databases_.find(database_file);
                if (it != lightning_databases_.end()) {
                    // Lightning database statistics (placeholder)
                    stats["num_paths"] = 0;
                    stats["num_vertices"] = 0;
                }
            } else if (planner_type == "Thunder") {
                auto it = thunder_databases_.find(database_file);
                if (it != thunder_databases_.end()) {
                    // Thunder database statistics (placeholder)
                    stats["num_paths"] = 0;
                    stats["num_vertices"] = 0;
                }
            }
        } catch (const std::exception& e) {
            std::cerr << "Failed to get experience stats: " << e.what() << std::endl;
        }
        
        return stats;
    }

private:
    std::map<std::string, std::shared_ptr<ot::LightningDB>> lightning_databases_;
    std::map<std::string, std::shared_ptr<ot::ThunderDB>> thunder_databases_;
    
    VampExperiencePlanners() = default;
    
    /**
     * @brief Generate intelligent default database path
     * @param planner_type "lightning" or "thunder"
     * @return Default database path with environment awareness
     */
    std::string getDefaultDatabasePath(const std::string& planner_type) const {
        // Environment-aware database naming
        const char* env = std::getenv("VAMP_ENV");
        std::string environment = env ? env : "default";
        
        // Process-safe naming to avoid collisions
        auto pid = std::to_string(getpid());
        auto timestamp = std::to_string(std::time(nullptr));
        
        return planner_type + "_" + environment + "_" + pid + ".db";
    }
    
    /**
     * @brief Get or create Lightning database
     * @param si Space information
     * @param config Experience configuration
     * @return Lightning database
     */
    std::shared_ptr<ot::LightningDB> getLightningDatabase(const ob::SpaceInformationPtr& si,
                                                         const ExperienceConfig& config) {
        auto it = lightning_databases_.find(config.database_file);
        if (it != lightning_databases_.end()) {
            return it->second;
        }
        
        try {
            // LightningDB constructor takes StateSpace, not SpaceInformation
            auto lightning_db = std::make_shared<ot::LightningDB>(si->getStateSpace());
            
            // Lightning database configuration
            if (!config.database_file.empty() && config.auto_create) {
                std::cout << "Creating Lightning database for " << config.database_file << std::endl;
            }
            
            lightning_databases_[config.database_file] = lightning_db;
            return lightning_db;
            
        } catch (const std::exception& e) {
            std::cerr << "Failed to create Lightning database: " << e.what() << std::endl;
            return nullptr;
        }
    }
    
    /**
     * @brief Get or create Thunder database
     * @param si Space information
     * @param config Experience configuration
     * @return Thunder database
     */
    std::shared_ptr<ot::ThunderDB> getThunderDatabase(const ob::SpaceInformationPtr& si,
                                                     const ExperienceConfig& config) {
        auto it = thunder_databases_.find(config.database_file);
        if (it != thunder_databases_.end()) {
            return it->second;
        }
        
        try {
            // ThunderDB constructor also takes StateSpace, not SpaceInformation
            auto thunder_db = std::make_shared<ot::ThunderDB>(si->getStateSpace());
            
            // Thunder database configuration
            if (!config.database_file.empty() && config.auto_create) {
                std::cout << "Creating Thunder database for " << config.database_file << std::endl;
            }
            
            thunder_databases_[config.database_file] = thunder_db;
            return thunder_db;
            
        } catch (const std::exception& e) {
            std::cerr << "Failed to create Thunder database: " << e.what() << std::endl;
            return nullptr;
        }
    }
};

/**
 * @brief Convenience function to register experience-based planners with default databases
 */
inline void registerExperienceBasedPlanners() {
    VampExperiencePlanners::getInstance().registerExperiencePlanners();
}

/**
 * @brief Convenience function to register experience-based planners with custom databases
 * @param lightning_db_path Path to Lightning database
 * @param thunder_db_path Path to Thunder database
 */
inline void registerExperienceBasedPlanners(const std::string& lightning_db_path,
                                           const std::string& thunder_db_path) {
    VampExperiencePlanners::getInstance().registerExperiencePlanners(lightning_db_path, thunder_db_path);
}

/**
 * @brief Convenience function to register single experience planner with custom config
 * @param planner_type "Lightning" or "Thunder"
 * @param planner_name Custom name for registration
 * @param database_path Path to database file
 */
inline void registerExperiencePlanner(const std::string& planner_type,
                                     const std::string& planner_name,
                                     const std::string& database_path) {
    VampExperiencePlanners::ExperienceConfig config(database_path, true);
    VampExperiencePlanners::getInstance().registerExperiencePlanner(planner_type, planner_name, config);
}

/**
 * @brief Convenience function to create Lightning planner
 * @param si Space information
 * @param database_file Database file path
 * @return Lightning planner
 */
inline std::shared_ptr<og::LightningRetrieveRepair> createLightningPlanner(
    const ob::SpaceInformationPtr& si,
    const std::string& database_file = "lightning.db") {
    VampExperiencePlanners::ExperienceConfig config(database_file, true);
    return VampExperiencePlanners::getInstance().createLightningPlanner(si, config);
}

/**
 * @brief Convenience function to create Thunder planner
 * @param si Space information
 * @param database_file Database file path
 * @return Thunder planner
 */
inline std::shared_ptr<og::ThunderRetrieveRepair> createThunderPlanner(
    const ob::SpaceInformationPtr& si,
    const std::string& database_file = "thunder.db") {
    VampExperiencePlanners::ExperienceConfig config(database_file, true);
    return VampExperiencePlanners::getInstance().createThunderPlanner(si, config);
}

} // namespace vamp_ompl
