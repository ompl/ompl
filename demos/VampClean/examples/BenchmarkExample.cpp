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

/**
 * @file BenchmarkExample.cpp
 * @brief Example showing VAMP-OMPL integration with benchmarking
 * 
 * This example demonstrates how to use VAMPSetup with OMPL's benchmarking
 * infrastructure to compare different planners using VAMP's collision detection.
 */

#include "../core/VAMPSetup.h"

#include <ompl/tools/benchmark/Benchmark.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/informedtrees/BITstar.h>
#include <ompl/geometric/planners/prm/PRM.h>

#include <vamp/robots/panda.hh>
#include <vamp/collision/factory.hh>

#include <iostream>

using namespace ompl::vamp;

/**
 * @brief Create a challenging environment for benchmarking
 */
vamp::collision::Environment<float> createBenchmarkEnvironment() {
    vamp::collision::Environment<float> environment;
    
    // Create a more complex environment with various obstacles
    environment.spheres.emplace_back(
        vamp::collision::factory::sphere::array({0.0f, 0.0f, 0.5f}, 0.08f));
    environment.spheres.emplace_back(
        vamp::collision::factory::sphere::array({0.3f, -0.4f, 0.6f}, 0.06f));
    environment.spheres.emplace_back(
        vamp::collision::factory::sphere::array({-0.3f, 0.4f, 0.6f}, 0.06f));
    
    // Add some cuboids for variety
    environment.cuboids.emplace_back(
        vamp::collision::factory::cuboid::array(
            {0.2f, 0.2f, 0.4f}, {0.0f, 0.0f, 0.0f}, {0.05f, 0.05f, 0.2f}));
    environment.cuboids.emplace_back(
        vamp::collision::factory::cuboid::array(
            {-0.2f, -0.2f, 0.4f}, {0.0f, 0.0f, 0.0f}, {0.05f, 0.05f, 0.2f}));
    
    environment.sort();
    return environment;
}

int main(int argc, char** argv) {
    std::cout << "VAMP-OMPL Benchmarking Example" << std::endl;
    std::cout << "==============================" << std::endl;
    
    try {
        // Create environment and setup
        auto environment = createBenchmarkEnvironment();
        VAMPSetup<vamp::robots::Panda> vamp_setup(environment);
        
        // Set challenging start and goal states using OMPL SimpleSetup
        auto& simple_setup = vamp_setup.getSimpleSetup();
        auto space = simple_setup.getStateSpace();
        
        // Create start state
        ompl::base::ScopedState<> start_state(space);
        start_state[0] = -1.0; start_state[1] = -0.5; start_state[2] = 0.0;
        start_state[3] = -2.0; start_state[4] = 0.0; start_state[5] = 1.5; start_state[6] = 0.0;
        
        // Create goal state
        ompl::base::ScopedState<> goal_state(space);
        goal_state[0] = 1.0; goal_state[1] = 0.5; goal_state[2] = 0.0;
        goal_state[3] = -1.5; goal_state[4] = 0.0; goal_state[5] = 2.0; goal_state[6] = 0.785;
        
        simple_setup.setStartAndGoalStates(start_state, goal_state);
        
        std::cout << "Environment created with obstacles" << std::endl;
        std::cout << "Start and goal states configured" << std::endl;
        
        // Create benchmark using OMPL's benchmarking infrastructure
        ompl::tools::Benchmark benchmark(simple_setup, "VAMP-OMPL Benchmark");
        
        // Add planners to benchmark (using OMPL directly)
        std::cout << "Adding planners to benchmark..." << std::endl;
        
        benchmark.addPlanner(std::make_shared<ompl::geometric::RRTConnect>(
            simple_setup.getSpaceInformation()));
        
        benchmark.addPlanner(std::make_shared<ompl::geometric::RRTstar>(
            simple_setup.getSpaceInformation()));
        
        benchmark.addPlanner(std::make_shared<ompl::geometric::BITstar>(
            simple_setup.getSpaceInformation()));
        
        benchmark.addPlanner(std::make_shared<ompl::geometric::PRM>(
            simple_setup.getSpaceInformation()));
        
        // Configure benchmark parameters
        ompl::tools::Benchmark::Request request;
        request.maxTime = 5.0;         // 5 seconds per run
        request.maxMem = 1000.0;       // 1GB memory limit
        request.runCount = 10;         // 10 runs per planner
        request.displayProgress = true;
        
        std::cout << "Running benchmark..." << std::endl;
        std::cout << "- Time limit: " << request.maxTime << " seconds per run" << std::endl;
        std::cout << "- Runs per planner: " << request.runCount << std::endl;
        std::cout << "- Total planners: 4" << std::endl;
        
        // Run the benchmark
        benchmark.benchmark(request);
        
        // Save results
        std::string results_file = "vamp_ompl_benchmark.log";
        benchmark.saveResultsToFile(results_file.c_str());
        
        std::cout << "Benchmark completed!" << std::endl;
        std::cout << "Results saved to: " << results_file << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
} 