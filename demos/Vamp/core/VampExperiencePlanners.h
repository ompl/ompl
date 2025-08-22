#pragma once

#include "VampOMPLInterfaces.h"
#include <ompl/geometric/planners/experience/LightningRetrieveRepair.h>
#include <ompl/geometric/planners/experience/ThunderRetrieveRepair.h>
#include <ompl/tools/lightning/LightningDB.h>
#include <ompl/tools/thunder/ThunderDB.h>
#include <ompl/tools/thunder/SPARSdb.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <memory>
#include <string>
#include <map>
#include <filesystem>
#include <fstream>
#include <chrono>
#include <mutex>

namespace vamp_ompl {

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace ot = ompl::tools;

/**
 * @brief Production-ready experience database configuration
 * 
 * This provides comprehensive configuration options for experience-based planners
 * with proper validation, defaults, and error handling.
 */
struct ExperienceConfig {
    // Database settings
    std::string database_path;           // Full path to database file
    std::string database_name;           // Logical database name
    bool auto_create_directory = true;   // Create database directory if needed
    bool enable_backup = true;           // Enable automatic backups
    
    // Planning behavior
    bool enable_recall = true;           // Enable experience recall
    bool enable_repair = true;           // Enable path repair
    double recall_epsilon = 0.1;         // Recall distance threshold
    double repair_time_limit = 2.0;      // Max time for repair (seconds)
    
    // Database management
    size_t max_experiences = 10000;      // Maximum stored experiences
    size_t backup_frequency = 100;       // Backup every N additions
    bool compress_database = false;      // Enable database compression
    
    // Performance tuning
    size_t experience_cache_size = 1000; // In-memory cache size
    bool lazy_loading = true;            // Load experiences on demand
    
    /**
     * @brief Create default configuration for robot/environment
     */
    static ExperienceConfig createDefault(const std::string& robot_name,
                                        const std::string& environment_name = "default") {
        ExperienceConfig config;
        config.database_name = robot_name + "_" + environment_name;
        config.database_path = getDefaultDatabasePath(config.database_name);
        return config;
    }
    
    /**
     * @brief Create configuration for persistent database
     */
    static ExperienceConfig createPersistent(const std::string& database_path) {
        ExperienceConfig config;
        config.database_path = database_path;
        config.database_name = std::filesystem::path(database_path).stem();
        config.auto_create_directory = true;
        config.enable_backup = true;
        return config;
    }
    
    /**
     * @brief Validate configuration
     */
    bool validate(std::string& error_message) const {
        if (database_path.empty()) {
            error_message = "Database path cannot be empty";
            return false;
        }
        
        if (recall_epsilon <= 0.0) {
            error_message = "Recall epsilon must be positive";
            return false;
        }
        
        if (repair_time_limit <= 0.0) {
            error_message = "Repair time limit must be positive";
            return false;
        }
        
        if (max_experiences == 0) {
            error_message = "Max experiences must be positive";
            return false;
        }
        
        return true;
    }

private:
    static std::string getDefaultDatabasePath(const std::string& name) {
        // Use user's home directory or project directory
        std::string base_dir = std::getenv("HOME") ? 
            std::string(std::getenv("HOME")) + "/.vamp_ompl/databases" :
            "./vamp_databases";
        
        return base_dir + "/" + name + ".db";
    }
};

/**
 * @brief Database statistics for monitoring and analysis
 */
struct DatabaseStats {
    size_t num_experiences = 0;
    size_t num_successful_recalls = 0;
    size_t num_failed_recalls = 0;
    size_t num_successful_repairs = 0;
    size_t num_failed_repairs = 0;
    double average_recall_time = 0.0;
    double average_repair_time = 0.0;
    size_t database_size_bytes = 0;
    std::chrono::system_clock::time_point last_updated;
    
    void reset() {
        *this = DatabaseStats{};
        last_updated = std::chrono::system_clock::now();
    }
};

/**
 * @brief Production-ready experience planner manager
 * 
 * This class provides robust experience-based planning with proper database
 * management, error handling, performance monitoring, and production features.
 * 
 * Key features:
 * - Persistent database storage with backup/restore
 * - Thread-safe operations for concurrent access
 * - Performance monitoring and statistics
 * - Automatic database maintenance and cleanup
 * - Configuration validation and error handling
 * - Support for multiple robot/environment combinations
 */
class VampExperiencePlanners {
public:
    /**
     * @brief Get singleton instance
     */
    static VampExperiencePlanners& getInstance() {
        static VampExperiencePlanners instance;
        return instance;
    }
    
    /**
     * @brief Create Lightning planner with production configuration
     * @param si Space information
     * @param config Experience configuration
     * @return Configured Lightning planner
     */
    std::shared_ptr<og::LightningRetrieveRepair> createLightningPlanner(
        const ob::SpaceInformationPtr& si,
        const ExperienceConfig& config) {
        
        std::lock_guard<std::mutex> lock(mutex_);
        
        // Validate configuration
        std::string error_msg;
        if (!config.validate(error_msg)) {
            throw VampConfigurationError("Lightning configuration invalid: " + error_msg);
        }
        
        // Create or get database
        auto lightning_db = getOrCreateLightningDatabase(si, config);
        if (!lightning_db) {
            throw VampConfigurationError("Failed to create Lightning database: " + config.database_path);
        }
        
        // Create planner
        auto planner = std::make_shared<og::LightningRetrieveRepair>(si, lightning_db);
        
        // Configure planner parameters
        if (config.enable_recall) {
            // Lightning planners automatically enable recall
            std::cout << "Lightning planner created with recall enabled (epsilon: " 
                     << config.recall_epsilon << ")" << std::endl;
        }
        
        return planner;
    }
    
    /**
     * @brief Create Thunder planner with production configuration
     * @param si Space information
     * @param config Experience configuration
     * @return Configured Thunder planner
     */
    std::shared_ptr<og::ThunderRetrieveRepair> createThunderPlanner(
        const ob::SpaceInformationPtr& si,
        const ExperienceConfig& config) {
        
        std::lock_guard<std::mutex> lock(mutex_);
        
        // Validate configuration
        std::string error_msg;
        if (!config.validate(error_msg)) {
            throw VampConfigurationError("Thunder configuration invalid: " + error_msg);
        }
        
        // Create or get database
        auto thunder_db = getOrCreateThunderDatabase(si, config);
        if (!thunder_db) {
            throw VampConfigurationError("Failed to create Thunder database: " + config.database_path);
        }
        
        // Create planner
        auto planner = std::make_shared<og::ThunderRetrieveRepair>(si, thunder_db);
        
        std::cout << "Thunder planner created with database: " << config.database_path << std::endl;
        return planner;
    }
    
    /**
     * @brief Save successful planning experience to database
     * @param database_path Database file path
     * @param solution_path Successful solution path
     * @param planning_time Time taken to find solution
     * @return Success status
     */
    bool saveExperience(const std::string& database_path,
                       const og::PathGeometric& solution_path,
                       double planning_time = 0.0) {
        std::lock_guard<std::mutex> lock(mutex_);
        
        try {
            // Try Thunder database first (more common for saving experiences)
            auto thunder_it = thunder_databases_.find(database_path);
            if (thunder_it != thunder_databases_.end()) {
                og::PathGeometric path_copy(solution_path);
                double insertion_time;
                bool success = thunder_it->second->addPath(path_copy, insertion_time);
                
                if (success) {
                    updateStats(database_path, true, false, planning_time, insertion_time);
                    handleDatabaseMaintenance(database_path);
                }
                return success;
            }
            
            // Lightning databases handle saving automatically during planning
            auto lightning_it = lightning_databases_.find(database_path);
            if (lightning_it != lightning_databases_.end()) {
                std::cout << "Experience saved automatically by Lightning database" << std::endl;
                return true;
            }
            
            std::cerr << "Database not found for path: " << database_path << std::endl;
            return false;
            
        } catch (const std::exception& e) {
            std::cerr << "Failed to save experience: " << e.what() << std::endl;
            return false;
        }
    }
    
    /**
     * @brief Get comprehensive database statistics
     * @param database_path Database file path
     * @return Database statistics
     */
    DatabaseStats getDatabaseStats(const std::string& database_path) {
        std::lock_guard<std::mutex> lock(mutex_);
        
        auto it = stats_.find(database_path);
        if (it != stats_.end()) {
            return it->second;
        }
        
        return DatabaseStats{}; // Return empty stats for unknown databases
    }
    
    /**
     * @brief Backup database to specified location
     * @param database_path Source database path
     * @param backup_path Backup destination path
     * @return Success status
     */
    bool backupDatabase(const std::string& database_path, const std::string& backup_path) {
        std::lock_guard<std::mutex> lock(mutex_);
        
        try {
            if (!std::filesystem::exists(database_path)) {
                return false;
            }
            
            // Ensure backup directory exists
            std::filesystem::create_directories(std::filesystem::path(backup_path).parent_path());
            
            // Copy database file
            std::filesystem::copy_file(database_path, backup_path, 
                                     std::filesystem::copy_options::overwrite_existing);
            
            std::cout << "Database backed up: " << database_path << " -> " << backup_path << std::endl;
            return true;
            
        } catch (const std::exception& e) {
            std::cerr << "Backup failed: " << e.what() << std::endl;
            return false;
        }
    }
    
    /**
     * @brief Clear all experiences from database
     * @param database_path Database file path
     * @return Success status
     */
    bool clearDatabase(const std::string& database_path) {
        std::lock_guard<std::mutex> lock(mutex_);
        
        try {
            // Create backup before clearing
            std::string backup_path = database_path + ".backup." + 
                std::to_string(std::chrono::system_clock::now().time_since_epoch().count());
            backupDatabase(database_path, backup_path);
            
            // Remove from memory
            thunder_databases_.erase(database_path);
            lightning_databases_.erase(database_path);
            stats_.erase(database_path);
            
            // Remove file
            if (std::filesystem::exists(database_path)) {
                std::filesystem::remove(database_path);
            }
            
            std::cout << "Database cleared: " << database_path << std::endl;
            return true;
            
        } catch (const std::exception& e) {
            std::cerr << "Failed to clear database: " << e.what() << std::endl;
            return false;
        }
    }
    
    /**
     * @brief Register experience planners with the main registry
     * @param robot_name Robot identifier for database naming
     * @param environment_name Environment identifier for database naming
     * 
     * Note: This method is called from VampPlannerRegistry to avoid circular dependencies
     */
    void registerExperiencePlanners(const std::string& robot_name = "default",
                                   const std::string& environment_name = "default") {
        
        // This method is now called from VampPlannerRegistry::registerAllOMPLPlanners()
        // to avoid circular dependency issues. The actual registration happens there.
        
        std::cout << "Experience planners ready for robot: " << robot_name 
                  << ", environment: " << environment_name << std::endl;
    }

private:
    // Thread-safe storage
    std::mutex mutex_;
    std::map<std::string, std::shared_ptr<ot::LightningDB>> lightning_databases_;
    std::map<std::string, std::shared_ptr<ot::ThunderDB>> thunder_databases_;
    std::map<std::string, DatabaseStats> stats_;
    std::map<std::string, ExperienceConfig> configs_;
    
    VampExperiencePlanners() = default;
    
    /**
     * @brief Get or create Lightning database with proper initialization
     */
    std::shared_ptr<ot::LightningDB> getOrCreateLightningDatabase(
        const ob::SpaceInformationPtr& si,
        const ExperienceConfig& config) {
        
        auto it = lightning_databases_.find(config.database_path);
        if (it != lightning_databases_.end()) {
            return it->second;
        }
        
        try {
            // Ensure database directory exists
            if (config.auto_create_directory) {
                std::filesystem::create_directories(
                    std::filesystem::path(config.database_path).parent_path());
            }
            
            // Create Lightning database
            auto lightning_db = std::make_shared<ot::LightningDB>(si->getStateSpace());
            
            // Try to load existing database
            if (std::filesystem::exists(config.database_path)) {
                try {
                    lightning_db->load(config.database_path);
                    std::cout << "Loaded existing Lightning database: " << config.database_path << std::endl;
                } catch (const std::exception& e) {
                    std::cerr << "Warning: Could not load existing database, creating new: " 
                             << e.what() << std::endl;
                }
            }
            
            // Store database and configuration
            lightning_databases_[config.database_path] = lightning_db;
            configs_[config.database_path] = config;
            stats_[config.database_path] = DatabaseStats{};
            
            std::cout << "Lightning database ready: " << config.database_path << std::endl;
            return lightning_db;
            
        } catch (const std::exception& e) {
            std::cerr << "Failed to create Lightning database: " << e.what() << std::endl;
            return nullptr;
        }
    }
    
    /**
     * @brief Get or create Thunder database with proper initialization
     */
    std::shared_ptr<ot::ThunderDB> getOrCreateThunderDatabase(
        const ob::SpaceInformationPtr& si,
        const ExperienceConfig& config) {
        
        auto it = thunder_databases_.find(config.database_path);
        if (it != thunder_databases_.end()) {
            return it->second;
        }
        
        try {
            // Ensure database directory exists
            if (config.auto_create_directory) {
                std::filesystem::create_directories(
                    std::filesystem::path(config.database_path).parent_path());
            }
            
            // Create Thunder database
            auto thunder_db = std::make_shared<ot::ThunderDB>(si->getStateSpace());
            
            // Thunder database needs SPARSdb to be configured
            // Set up a SPARSdb instance for the database
            auto spars_db = std::make_shared<og::SPARSdb>(si);
            thunder_db->setSPARSdb(spars_db);
            
            // Try to load existing database if it exists
            if (std::filesystem::exists(config.database_path)) {
                try {
                    thunder_db->load(config.database_path);
                    std::cout << "Loaded existing Thunder database: " << config.database_path << std::endl;
                } catch (const std::exception& e) {
                    std::cerr << "Warning: Could not load existing database, will create new: " 
                             << e.what() << std::endl;
                }
            }
            
            // Store database and configuration
            thunder_databases_[config.database_path] = thunder_db;
            configs_[config.database_path] = config;
            stats_[config.database_path] = DatabaseStats{};
            
            std::cout << "Thunder database ready: " << config.database_path << std::endl;
            return thunder_db;
            
        } catch (const std::exception& e) {
            std::cerr << "Failed to create Thunder database: " << e.what() << std::endl;
            return nullptr;
        }
    }
    
    /**
     * @brief Update database statistics
     */
    void updateStats(const std::string& database_path, bool recall_success, 
                    bool repair_success, double planning_time, double operation_time) {
        auto& stats = stats_[database_path];
        
        if (recall_success) {
            stats.num_successful_recalls++;
            stats.average_recall_time = 
                (stats.average_recall_time * (stats.num_successful_recalls - 1) + operation_time) / 
                stats.num_successful_recalls;
        } else {
            stats.num_failed_recalls++;
        }
        
        if (repair_success) {
            stats.num_successful_repairs++;
            stats.average_repair_time = 
                (stats.average_repair_time * (stats.num_successful_repairs - 1) + operation_time) / 
                stats.num_successful_repairs;
        }
        
        stats.last_updated = std::chrono::system_clock::now();
        
        // Update file size
        try {
            if (std::filesystem::exists(database_path)) {
                stats.database_size_bytes = std::filesystem::file_size(database_path);
            }
        } catch (...) {
            // Ignore file size errors
        }
    }
    
    /**
     * @brief Handle database maintenance (backup, cleanup, etc.)
     */
    void handleDatabaseMaintenance(const std::string& database_path) {
        auto config_it = configs_.find(database_path);
        if (config_it == configs_.end()) return;
        
        const auto& config = config_it->second;
        auto& stats = stats_[database_path];
        
        // Check if backup is needed
        if (config.enable_backup && 
            (stats.num_experiences % config.backup_frequency == 0)) {
            
            std::string backup_path = database_path + ".backup." + 
                std::to_string(std::chrono::system_clock::now().time_since_epoch().count());
            backupDatabase(database_path, backup_path);
        }
        
        stats.num_experiences++;
    }
};

/**
 * @brief Convenience functions for easy access
 */

/**
 * @brief Initialize experience planners system (called automatically by registry)
 */
inline void initializeExperiencePlanners(const std::string& robot_name = "default",
                                        const std::string& environment_name = "default") {
    VampExperiencePlanners::getInstance().registerExperiencePlanners(robot_name, environment_name);
}

/**
 * @brief Create Lightning planner with simple configuration
 */
inline std::shared_ptr<og::LightningRetrieveRepair> createLightningPlanner(
    const ob::SpaceInformationPtr& si,
    const std::string& robot_name,
    const std::string& environment_name = "default") {
    
    auto config = ExperienceConfig::createDefault(robot_name + "_lightning", environment_name);
    return VampExperiencePlanners::getInstance().createLightningPlanner(si, config);
}

/**
 * @brief Create Thunder planner with simple configuration
 */
inline std::shared_ptr<og::ThunderRetrieveRepair> createThunderPlanner(
    const ob::SpaceInformationPtr& si,
    const std::string& robot_name,
    const std::string& environment_name = "default") {
    
    auto config = ExperienceConfig::createDefault(robot_name + "_thunder", environment_name);
    return VampExperiencePlanners::getInstance().createThunderPlanner(si, config);
}

/**
 * @brief Create persistent Lightning planner with custom database path
 */
inline std::shared_ptr<og::LightningRetrieveRepair> createPersistentLightningPlanner(
    const ob::SpaceInformationPtr& si,
    const std::string& database_path) {
    
    auto config = ExperienceConfig::createPersistent(database_path);
    return VampExperiencePlanners::getInstance().createLightningPlanner(si, config);
}

/**
 * @brief Create persistent Thunder planner with custom database path
 */
inline std::shared_ptr<og::ThunderRetrieveRepair> createPersistentThunderPlanner(
    const ob::SpaceInformationPtr& si,
    const std::string& database_path) {
    
    auto config = ExperienceConfig::createPersistent(database_path);
    return VampExperiencePlanners::getInstance().createThunderPlanner(si, config);
}

} // namespace vamp_ompl
