#include <vector>
#include <array>
#include <utility>
#include <iostream>
#include <iomanip>
#include <random>
#include <algorithm>
#include <numeric>
#include <cmath>
#include <chrono>

#include <vamp/collision/factory.hh>
#include <vamp/planning/validate.hh>
#include <vamp/planning/rrtc.hh>
#include <vamp/planning/simplify.hh>
#include <vamp/robots/panda.hh>
#include <vamp/random/halton.hh>

using Robot = vamp::robots::Panda;
static constexpr const std::size_t rake = vamp::FloatVectorWidth;
using EnvironmentInput = vamp::collision::Environment<float>;
using EnvironmentVector = vamp::collision::Environment<vamp::FloatVector<rake>>;
using RRTC = vamp::planning::RRTC<Robot, rake, Robot::resolution>;

// Start and goal configurations
static constexpr Robot::ConfigurationArray start = {0., -0.785, 0., -2.356, 0., 1.571, 0.785};
static constexpr Robot::ConfigurationArray goal = {2.35, 1., 0., -0.8, 0, 2.5, 0.785};

// Spheres for the cage problem - (x, y, z) center coordinates with fixed, common radius defined below
static const std::vector<std::array<float, 3>> problem = {
    {0.55, 0, 0.25},
    {0.35, 0.35, 0.25},
    {0, 0.55, 0.25},
    {-0.55, 0, 0.25},
    {-0.35, -0.35, 0.25},
    {0, -0.55, 0.25},
    {0.35, -0.35, 0.25},
    {0.35, 0.35, 0.8},
    {0, 0.55, 0.8},
    {-0.35, 0.35, 0.8},
    {-0.55, 0, 0.8},
    {-0.35, -0.35, 0.8},
    {0, -0.55, 0.8},
    {0.35, -0.35, 0.8},
};

// Radius for obstacle spheres
static constexpr float radius = 0.2;

auto main(int argc, char **argv) -> int
{
    // Benchmark parameters (similar to Python version)
    int n_trials = 100;
    float variation = 0.01f;
    
    // Parse command line arguments if provided
    if (argc > 1) n_trials = std::atoi(argv[1]);
    if (argc > 2) variation = std::atof(argv[2]);
    
    std::cout << "Running benchmark with " << n_trials << " trials, variation=" 
              << variation << ", radius=" << radius << std::endl;
    
    // Random number generator for variations
    std::mt19937 rng_var(0);
    std::uniform_real_distribution<float> dist(-variation, variation);
    
    // Statistics collection
    std::vector<double> planning_times;
    std::vector<double> simplification_times;
    std::vector<double> initial_costs;
    std::vector<double> simplified_costs;
    
    // Run benchmark trials
    for (int trial = 0; trial < n_trials; ++trial)
    {
        // Create varied environment for this trial
        auto obstacles = problem;
        std::shuffle(obstacles.begin(), obstacles.end(), rng_var);
        
        EnvironmentInput environment;
        for (auto sphere : obstacles)
        {
            // Add random variation
            sphere[0] += dist(rng_var);
            sphere[1] += dist(rng_var);
            sphere[2] += dist(rng_var);
            
            environment.spheres.emplace_back(vamp::collision::factory::sphere::array(sphere, radius));
        }
        
        environment.sort();
        auto env_v = EnvironmentVector(environment);
        
        // Validate start and goal
        auto start_config = Robot::Configuration(start);
        auto goal_config = Robot::Configuration(goal);
        
        // Use fkcc to check collision for single configurations
        typename Robot::template ConfigurationBlock<rake> start_block;
        typename Robot::template ConfigurationBlock<rake> goal_block;
        for (auto i = 0U; i < Robot::dimension; ++i)
        {
            start_block[i] = start_config.broadcast(i);
            goal_block[i] = goal_config.broadcast(i);
        }
        
        bool start_valid = env_v.attachments ? Robot::template fkcc_attach<rake>(env_v, start_block) :
                                                Robot::template fkcc<rake>(env_v, start_block);
        bool goal_valid = env_v.attachments ? Robot::template fkcc_attach<rake>(env_v, goal_block) :
                                               Robot::template fkcc<rake>(env_v, goal_block);
        
        if (!start_valid || !goal_valid)
        {
            continue;
        }
        
        // Create RNG for planning
        auto rng = std::make_shared<vamp::rng::Halton<Robot>>();
        
        // Setup RRTC and plan
        vamp::planning::RRTCSettings rrtc_settings;
        rrtc_settings.range = 2.683307;
        
        auto plan_start = std::chrono::steady_clock::now();
        auto result = RRTC::solve(start_config, goal_config, env_v, rrtc_settings, rng);
        auto plan_end = std::chrono::steady_clock::now();
        
        // If successful
        if (result.path.size() > 0)
        {
            auto planning_time_us = std::chrono::duration_cast<std::chrono::microseconds>(
                plan_end - plan_start).count();
            planning_times.push_back(static_cast<double>(planning_time_us));
            
            // Compute initial path cost
            double initial_cost = result.path.cost();
            initial_costs.push_back(initial_cost);
            
            // Simplify path with default settings
            vamp::planning::SimplifySettings simplify_settings;
            auto simp_start = std::chrono::steady_clock::now();
            auto simplify_result = vamp::planning::simplify<Robot, rake, Robot::resolution>(
                result.path, env_v, simplify_settings, rng);
            auto simp_end = std::chrono::steady_clock::now();
            
            auto simplification_time_us = std::chrono::duration_cast<std::chrono::microseconds>(
                simp_end - simp_start).count();
            simplification_times.push_back(static_cast<double>(simplification_time_us));
            
            // Compute simplified path cost
            double simplified_cost = simplify_result.path.cost();
            simplified_costs.push_back(simplified_cost);
        }
    }
    
    // Compute and print statistics
    if (!planning_times.empty())
    {
        auto compute_stats = [](const std::vector<double>& data) {
            double mean = std::accumulate(data.begin(), data.end(), 0.0) / data.size();
            std::vector<double> sorted = data;
            std::sort(sorted.begin(), sorted.end());
            double median = sorted[sorted.size() / 2];
            double min = sorted.front();
            double max = sorted.back();
            
            double sq_sum = 0.0;
            for (auto v : data) sq_sum += (v - mean) * (v - mean);
            double std = std::sqrt(sq_sum / data.size());
            
            return std::make_tuple(mean, std, min, median, max);
        };
        
        auto [time_mean, time_std, time_min, time_med, time_max] = compute_stats(planning_times);
        auto [simp_mean, simp_std, simp_min, simp_med, simp_max] = compute_stats(simplification_times);
        auto [init_mean, init_std, init_min, init_med, init_max] = compute_stats(initial_costs);
        auto [final_mean, final_std, final_min, final_med, final_max] = compute_stats(simplified_costs);
        
        std::cout << "\n=== Benchmark Results (" << planning_times.size() 
                  << " successful trials) ===" << std::endl;
        
        std::cout << "\nPlanning Time (microseconds):" << std::endl;
        std::cout << "  Mean:   " << time_mean << std::endl;
        std::cout << "  Std:    " << time_std << std::endl;
        std::cout << "  Min:    " << time_min << std::endl;
        std::cout << "  Median: " << time_med << std::endl;
        std::cout << "  Max:    " << time_max << std::endl;
        
        std::cout << "\nSimplification Time (microseconds):" << std::endl;
        std::cout << "  Mean:   " << simp_mean << std::endl;
        std::cout << "  Std:    " << simp_std << std::endl;
        std::cout << "  Min:    " << simp_min << std::endl;
        std::cout << "  Median: " << simp_med << std::endl;
        std::cout << "  Max:    " << simp_max << std::endl;
        
        std::cout << "\nInitial Path Cost:" << std::endl;
        std::cout << "  Mean:   " << init_mean << std::endl;
        std::cout << "  Std:    " << init_std << std::endl;
        std::cout << "  Min:    " << init_min << std::endl;
        std::cout << "  Median: " << init_med << std::endl;
        std::cout << "  Max:    " << init_max << std::endl;
        
        std::cout << "\nSimplified Path Cost:" << std::endl;
        std::cout << "  Mean:   " << final_mean << std::endl;
        std::cout << "  Std:    " << final_std << std::endl;
        std::cout << "  Min:    " << final_min << std::endl;
        std::cout << "  Median: " << final_med << std::endl;
        std::cout << "  Max:    " << final_max << std::endl;
    }
    else
    {
        std::cout << "No successful trials!" << std::endl;
    }
    
    return 0;
}
