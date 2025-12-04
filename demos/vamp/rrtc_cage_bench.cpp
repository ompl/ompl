// demos/vamp/rrtc_cage_bench.cpp
#include <iostream>
#include <vector>
#include <array>
#include <chrono>
#include <random>
#include <algorithm>
#include <numeric>
#include <cmath>

// OMPL headers
#include <ompl/base/PlannerTerminationCondition.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>

// VAMP integration headers
#include <ompl/vamp/Utils.h>
#include <ompl/vamp/VampStateValidityChecker.h>
#include <ompl/vamp/VampMotionValidator.h>
#include <vamp/collision/factory.hh>

// Robot definition (local to demos)
#include <vamp/robots/panda.hh>

namespace ob = ompl::base;
namespace og = ompl::geometric;

using Robot = vamp::robots::Panda;
using Environment = vamp::collision::Environment<vamp::FloatVector<vamp::FloatVectorWidth>>;

int main(int argc, char** argv)
{
    // Benchmark parameters (similar to Python version)
    int n_trials = 100;
    float variation = 0.01f;
    float radius = 0.2f;
    
    // Parse command line arguments if provided
    if (argc > 1) n_trials = std::atoi(argv[1]);
    if (argc > 2) variation = std::atof(argv[2]);
    if (argc > 3) radius = std::atof(argv[3]);
    
    std::cout << "Running benchmark with " << n_trials << " trials, variation=" 
              << variation << ", radius=" << radius << std::endl;
    
    // Base obstacle positions
    std::vector<std::array<float, 3>> base_obstacles = {
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
    
    // Random number generator
    std::mt19937 rng(0);
    std::uniform_real_distribution<float> dist(-variation, variation);

    // Create state space (reused across trials)
    auto space = std::make_shared<ob::RealVectorStateSpace>(Robot::dimension);
    static constexpr std::array<float, Robot::dimension> zeros = {0., 0., 0., 0., 0., 0., 0.};
    static constexpr std::array<float, Robot::dimension> ones = {1., 1., 1., 1., 1., 1., 1.};
    auto zero_v = Robot::Configuration(zeros);
    auto one_v = Robot::Configuration(ones);
    Robot::scale_configuration(zero_v);
    Robot::scale_configuration(one_v);
    ob::RealVectorBounds bounds(Robot::dimension);
    for (auto i = 0U; i < Robot::dimension; ++i)
    {
        bounds.setLow(i, zero_v[{0, i}]);
        bounds.setHigh(i, one_v[{0, i}]);
    }
    space->setBounds(bounds);

    // Define start and goal configurations
    std::array<double, 7> start_config = {0., -0.785, 0., -2.356, 0., 1.571, 0.785};
    std::array<double, 7> goal_config = {2.35, 1., 0., -0.8, 0., 2.5, 0.785};
    
    // Statistics collection
    std::vector<double> planning_times;
    std::vector<double> path_costs;
    std::vector<size_t> path_lengths;
    
    // Run benchmark trials
    for (int trial = 0; trial < n_trials; ++trial)
    {
        // Create varied environment for this trial
        auto obstacles = base_obstacles;
        std::shuffle(obstacles.begin(), obstacles.end(), rng);
        
        vamp::collision::Environment<float> env_float;
        for (auto& obs : obstacles)
        {
            // Add random variation
            obs[0] += dist(rng);
            obs[1] += dist(rng);
            obs[2] += dist(rng);
            
            env_float.spheres.emplace_back(
                vamp::collision::factory::sphere::array(obs, radius));
        }
        env_float.sort();
        Environment env(env_float);

        // Create space information with VAMP validators
        auto si = std::make_shared<ob::SpaceInformation>(space);
        si->setStateValidityChecker(
            std::make_shared<ompl::vamp::VampStateValidityChecker<Robot>>(si, env));
        si->setMotionValidator(
            std::make_shared<ompl::vamp::VampMotionValidator<Robot>>(si, env));
        si->setup();

        // Setup start and goal
        ob::ScopedState<> start(space);
        ob::ScopedState<> goal(space);
        for (std::size_t i = 0; i < Robot::dimension; ++i)
        {
            start[i] = start_config[i];
            goal[i] = goal_config[i];
        }
        
        // Validate start and goal
        if (!si->isValid(start.get()) || !si->isValid(goal.get()))
        {
            continue;
        }

        // Setup problem definition
        auto pdef = std::make_shared<ob::ProblemDefinition>(si);
        pdef->setStartAndGoalStates(start, goal);

        // Create RRTConnect planner
        auto planner = std::make_shared<og::RRTConnect>(si);
        planner->setProblemDefinition(pdef);
        planner->setup();

        // Solve with 5 second timeout
        auto start_time = std::chrono::steady_clock::now();
        ob::PlannerStatus status = planner->solve(ob::timedPlannerTerminationCondition(5.0));
        auto end_time = std::chrono::steady_clock::now();
        
        if (status == ob::PlannerStatus::EXACT_SOLUTION)
        {
            auto duration_us = std::chrono::duration_cast<std::chrono::microseconds>(
                end_time - start_time).count();
            planning_times.push_back(static_cast<double>(duration_us));
            
            auto path = pdef->getSolutionPath()->as<og::PathGeometric>();
            path_costs.push_back(path->length());
            path_lengths.push_back(path->getStateCount());
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
        auto [cost_mean, cost_std, cost_min, cost_med, cost_max] = compute_stats(path_costs);
        
        std::cout << "\n=== Benchmark Results (" << planning_times.size() 
                  << " successful trials) ===" << std::endl;
        std::cout << "\nPlanning Time (microseconds):" << std::endl;
        std::cout << "  Mean:   " << time_mean << std::endl;
        std::cout << "  Std:    " << time_std << std::endl;
        std::cout << "  Min:    " << time_min << std::endl;
        std::cout << "  Median: " << time_med << std::endl;
        std::cout << "  Max:    " << time_max << std::endl;
        
        std::cout << "\nPath Cost:" << std::endl;
        std::cout << "  Mean:   " << cost_mean << std::endl;
        std::cout << "  Std:    " << cost_std << std::endl;
        std::cout << "  Min:    " << cost_min << std::endl;
        std::cout << "  Median: " << cost_med << std::endl;
        std::cout << "  Max:    " << cost_max << std::endl;
    }
    else
    {
        std::cout << "No successful trials!" << std::endl;
    }

    return 0;
}