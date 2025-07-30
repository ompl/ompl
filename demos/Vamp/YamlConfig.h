#pragma once

#include <string>
#include <iostream>
#include <fstream>
#include <map>
#include <sstream>
#include <vector>
#include <algorithm>

namespace vamp_ompl {

/**
 * @brief Simple YAML parser for VAMP configuration files
 * This is a lightweight parser that handles the specific structure of our config files
 */
class SimpleYamlParser {
public:
    struct YamlNode {
        std::string value;
        std::map<std::string, YamlNode> children;
        
        // Check if node exists and has a value
        bool exists() const { return !value.empty() || !children.empty(); }
        
        // Get string value with default
        std::string getString(const std::string& default_value = "") const {
            return value.empty() ? default_value : value;
        }
        
        // Get double value with default
        double getDouble(double default_value = 0.0) const {
            if (value.empty()) return default_value;
            try {
                return std::stod(value);
            } catch (...) {
                return default_value;
            }
        }
        
        // Get bool value with default
        bool getBool(bool default_value = false) const {
            if (value.empty()) return default_value;
            std::string lower_value = value;
            std::transform(lower_value.begin(), lower_value.end(), lower_value.begin(), ::tolower);
            return lower_value == "true" || lower_value == "yes" || lower_value == "1" || lower_value == "on";
        }
        
        // Access child nodes
        const YamlNode& operator[](const std::string& key) const {
            static YamlNode empty_node;
            auto it = children.find(key);
            return it != children.end() ? it->second : empty_node;
        }
    };

private:
    YamlNode root_;
    
    std::string trim(const std::string& str) {
        size_t first = str.find_first_not_of(' ');
        if (first == std::string::npos) return "";
        size_t last = str.find_last_not_of(' ');
        return str.substr(first, (last - first + 1));
    }
    
    std::string removeComment(const std::string& line) {
        size_t comment_pos = line.find('#');
        if (comment_pos != std::string::npos) {
            return line.substr(0, comment_pos);
        }
        return line;
    }
    
    int getIndentLevel(const std::string& line) {
        int indent = 0;
        for (char c : line) {
            if (c == ' ') indent++;
            else break;
        }
        return indent;
    }
    
    std::pair<std::string, std::string> parseKeyValue(const std::string& line) {
        size_t colon_pos = line.find(':');
        if (colon_pos == std::string::npos) {
            return std::make_pair(trim(line), "");
        }
        
        std::string key = trim(line.substr(0, colon_pos));
        std::string value = trim(line.substr(colon_pos + 1));
        
        // Remove quotes if present
        if (!value.empty() && ((value.front() == '"' && value.back() == '"') ||
                               (value.front() == '\'' && value.back() == '\''))) {
            value = value.substr(1, value.length() - 2);
        }
        
        return std::make_pair(key, value);
    }

public:
    bool parseFile(const std::string& filename) {
        std::ifstream file(filename);
        if (!file.is_open()) {
            std::cerr << "Could not open YAML file: " << filename << std::endl;
            return false;
        }
        
        std::string line;
        std::vector<YamlNode*> node_stack;
        std::vector<int> indent_stack;
        
        node_stack.push_back(&root_);
        indent_stack.push_back(-1);
        
        while (std::getline(file, line)) {
            // Remove comments and trim
            std::string original_line = line;
            line = removeComment(line);
            
            // Skip empty lines
            if (trim(line).empty()) continue;
            
            int current_indent = getIndentLevel(original_line);
            line = trim(line);
            
            // Parse key-value pair
            auto [key, value] = parseKeyValue(line);
            if (key.empty()) continue;
            
            // Adjust stack based on indentation - pop until we find the right parent
            while (indent_stack.size() > 1 && current_indent <= indent_stack.back()) {
                node_stack.pop_back();
                indent_stack.pop_back();
            }
            
            // Add new node
            YamlNode& new_node = node_stack.back()->children[key];
            new_node.value = value;
            
            // Debug output
            if (node_stack.size() <= 3) {  // Only show first few levels to avoid spam
                std::string indent_str(node_stack.size() * 2, ' ');
                std::cout << "🔍 Parse: " << indent_str << "'" << key << "' = '" << value << "' (indent=" << current_indent << ", stack_size=" << node_stack.size() << ")" << std::endl;
            }
            
            // If this could have children (no value or empty value), add to stack
            if (value.empty()) {
                node_stack.push_back(&new_node);
                indent_stack.push_back(current_indent);
            }
        }
        
        return true;
    }
    
    const YamlNode& getRoot() const { return root_; }
    const YamlNode& operator[](const std::string& key) const { return root_[key]; }
};

/**
 * @brief Configuration structure that mirrors the YAML file
 */
struct VampYamlConfig {
    // Planning configuration
    struct {
        struct {
            std::string name = "panda";
            std::string urdf_path = "";
        } robot;
        
        struct {
            std::string name = "sphere_cage";
            std::string description = "";
        } environment;
        
        struct {
            std::string name = "BIT*";
            double planning_time = 1.0;
            double simplification_time = 0.5;
            bool optimize_path = false;
        } planner;
        
        struct {
            bool write_path = false;
            std::string description = "VAMP-OMPL Demo";
        } output;
    } planning;
    
    // Visualization configuration
    struct {
        bool enabled = false;
        bool auto_start = true;
        
        struct {
            double duration = 10.0;
            bool loop = false;
            bool draw_trajectory = true;
        } animation;
        
        struct {
            bool gui = true;
            bool interactive_mode = false;
            bool verbose = false;
        } display;
        
        struct {
            std::string name = "";
            std::string urdf_path = "";
        } robot;
        
        struct {
            std::string name = "";
        } environment;
    } visualization;
    
    /**
     * @brief Load configuration from YAML file
     */
    bool loadFromFile(const std::string& filename) {
        SimpleYamlParser parser;
        if (!parser.parseFile(filename)) {
            std::cout << "🔍 Debug: parseFile failed!" << std::endl;
            return false;
        }
        
        const auto& root = parser.getRoot();
        std::cout << "🔍 Debug: Root has " << root.children.size() << " children" << std::endl;
        std::cout << "🔍 Debug: Root children keys: ";
        for (const auto& [key, value] : root.children) {
            std::cout << "'" << key << "' ";
        }
        std::cout << std::endl;
        
        // Parse planning configuration
        const auto& plan = root["planning"];
        std::cout << "🔍 Debug: Planning node exists: " << plan.exists() << std::endl;
        planning.robot.name = plan["robot"]["name"].getString(planning.robot.name);
        planning.robot.urdf_path = plan["robot"]["urdf_path"].getString(planning.robot.urdf_path);
        
        planning.environment.name = plan["environment"]["name"].getString(planning.environment.name);
        planning.environment.description = plan["environment"]["description"].getString(planning.environment.description);
        
        planning.planner.name = plan["planner"]["name"].getString(planning.planner.name);
        planning.planner.planning_time = plan["planner"]["planning_time"].getDouble(planning.planner.planning_time);
        planning.planner.simplification_time = plan["planner"]["simplification_time"].getDouble(planning.planner.simplification_time);
        planning.planner.optimize_path = plan["planner"]["optimize_path"].getBool(planning.planner.optimize_path);
        
        planning.output.write_path = plan["output"]["write_path"].getBool(planning.output.write_path);
        planning.output.description = plan["output"]["description"].getString(planning.output.description);
        
        // Parse visualization configuration
        const auto& viz = root["visualization"];
        visualization.enabled = viz["enabled"].getBool(visualization.enabled);
        visualization.auto_start = viz["auto_start"].getBool(visualization.auto_start);
        
        visualization.animation.duration = viz["animation"]["duration"].getDouble(visualization.animation.duration);
        visualization.animation.loop = viz["animation"]["loop"].getBool(visualization.animation.loop);
        visualization.animation.draw_trajectory = viz["animation"]["draw_trajectory"].getBool(visualization.animation.draw_trajectory);
        
        visualization.display.gui = viz["display"]["gui"].getBool(visualization.display.gui);
        visualization.display.interactive_mode = viz["display"]["interactive_mode"].getBool(visualization.display.interactive_mode);
        visualization.display.verbose = viz["display"]["verbose"].getBool(visualization.display.verbose);
        
        visualization.robot.name = viz["robot"]["name"].getString(visualization.robot.name);
        visualization.robot.urdf_path = viz["robot"]["urdf_path"].getString(visualization.robot.urdf_path);
        
        visualization.environment.name = viz["environment"]["name"].getString(visualization.environment.name);
        
        return true;
    }
    
    /**
     * @brief Print configuration for debugging
     */
    void print() const {
        std::cout << "\n=== YAML Configuration ===" << std::endl;
        std::cout << "Planning:" << std::endl;
        std::cout << "  Robot: " << planning.robot.name << std::endl;
        std::cout << "  Environment: " << planning.environment.name << std::endl;
        std::cout << "  Planner: " << planning.planner.name << std::endl;
        std::cout << "  Planning time: " << planning.planner.planning_time << "s" << std::endl;
        std::cout << "  Write path: " << (planning.output.write_path ? "yes" : "no") << std::endl;
        
        std::cout << "Visualization:" << std::endl;
        std::cout << "  Enabled: " << (visualization.enabled ? "yes" : "no") << std::endl;
        if (visualization.enabled) {
            std::cout << "  Duration: " << visualization.animation.duration << "s" << std::endl;
            std::cout << "  Loop: " << (visualization.animation.loop ? "yes" : "no") << std::endl;
            std::cout << "  GUI: " << (visualization.display.gui ? "yes" : "no") << std::endl;
        }
        std::cout << "=========================" << std::endl;
    }
};

} // namespace vamp_ompl 