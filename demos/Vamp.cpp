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
 * @file Vamp.cpp
 * @brief VAMP-OMPL integration with simple planning and benchmarking modes
 */

#include <ompl/base/spaces/VAMPStateSpace.h>

#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/informedtrees/BITstar.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/tools/benchmark/Benchmark.h>

#include <iostream>
#include <chrono>
#include <string>

using namespace ompl;

/**
 * @brief Create sphere cage environment
 */
vamp::collision::Environment<float> createSphereCageEnvironment() {
    vamp::collision::Environment<float> environment;
    
    const std::vector<std::array<float, 3>> sphere_positions = {
        {0.55f, 0.0f, 0.25f},   {0.35f, 0.35f, 0.25f},  {0.0f, 0.55f, 0.25f},
        {-0.55f, 0.0f, 0.25f},  {-0.35f, -0.35f, 0.25f},{0.0f, -0.55f, 0.25f},
        {0.35f, -0.35f, 0.25f}, {0.35f, 0.35f, 0.8f},   {0.0f, 0.55f, 0.8f},
        {-0.35f, 0.35f, 0.8f},  {-0.55f, 0.0f, 0.8f},   {-0.35f, -0.35f, 0.8f},
        {0.0f, -0.55f, 0.8f},   {0.35f, -0.35f, 0.8f}
    };
    
    for (const auto& pos : sphere_positions) {
        environment.spheres.emplace_back(
            vamp::collision::factory::sphere::array(pos, 0.15f));
    }
    
    environment.sort();
    return environment;
}

/**
 * @brief Run simple planning demo
 */
void runSimpleDemo() {
    std::cout << "VAMP-OMPL Simple Planning Demo" << std::endl;
    std::cout << "===============================" << std::endl;
    
    // 1. Create VAMP collision environment
    std::cout << "\n1. Creating collision environment..." << std::endl;
    auto environment = createSphereCageEnvironment();
    std::cout << "   Created environment with " << environment.spheres.size() << " spheres" << std::endl;
    
    // 2. Create VAMP state space
    std::cout << "\n2. Creating VAMP state space..." << std::endl;
    auto space = std::make_shared<geometric::VAMPStateSpace<vamp::robots::Panda>>(environment);
    std::cout << "   State space: " << space->getName() << " (dimension: " << space->getDimension() << ")" << std::endl;
    
    // 3. Create SimpleSetup and configure validators
    std::cout << "\n3. Creating SimpleSetup..." << std::endl;
    geometric::SimpleSetup ss(space);
    
    auto si = ss.getSpaceInformation();
    ss.setStateValidityChecker(space->allocDefaultStateValidityChecker(si));
    si->setMotionValidator(space->allocDefaultMotionValidator(si));
    std::cout << "   SimpleSetup created with VAMP validators" << std::endl;
    
    // 4. Set start and goal states
    std::cout << "\n4. Setting start and goal states..." << std::endl;
    base::ScopedState<> start(space);
    start[0] = 0.0; start[1] = -0.785; start[2] = 0.0;
    start[3] = -2.356; start[4] = 0.0; start[5] = 1.571; start[6] = 0.785;
    
    base::ScopedState<> goal(space);
    goal[0] = 2.35; goal[1] = 1.0; goal[2] = 0.0;
    goal[3] = -0.8; goal[4] = 0.0; goal[5] = 2.5; goal[6] = 0.785;
    
    ss.setStartAndGoalStates(start, goal);
    
    // 5. Plan with RRTConnect
    std::cout << "\n5. Planning with RRTConnect..." << std::endl;
    ss.setPlanner(std::make_shared<geometric::RRTConnect>(ss.getSpaceInformation()));
    
    auto start_time = std::chrono::high_resolution_clock::now();
    base::PlannerStatus solved = ss.solve(5.0);
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    
    if (solved) {
        std::cout << "    Solution found in " << duration.count() << " ms!" << std::endl;
        auto& path = ss.getSolutionPath();
        std::cout << "   Path: " << path.getStateCount() << " waypoints, length = " << path.length() << std::endl;
        
        ss.simplifySolution();
        std::cout << "   Simplified: " << path.getStateCount() << " waypoints, length = " << path.length() << std::endl;
    } else {
        std::cout << "    No solution found" << std::endl;
    }
}

/**
 * @brief Run benchmarking demo
 */
void runBenchmark() {
    std::cout << "VAMP-OMPL Benchmarking" << std::endl;
    std::cout << "======================" << std::endl;
    
    // 1. Create environment and state space
    auto environment = createSphereCageEnvironment();
    auto space = std::make_shared<geometric::VAMPStateSpace<vamp::robots::Panda>>(environment);
    
    // 2. Create SimpleSetup and configure validators
    geometric::SimpleSetup ss(space);
    auto si = ss.getSpaceInformation();
    ss.setStateValidityChecker(space->allocDefaultStateValidityChecker(si));
    si->setMotionValidator(space->allocDefaultMotionValidator(si));
    std::cout << "SimpleSetup created with " << space->getName() << " and VAMP validators" << std::endl;
    
    // 3. Set start and goal
    base::ScopedState<> start(space);
    start[0] = -1.0; start[1] = -0.5; start[2] = 0.0;
    start[3] = -2.0; start[4] = 0.0; start[5] = 1.5; start[6] = 0.0;
    
    base::ScopedState<> goal(space);
    goal[0] = 1.0; goal[1] = 0.5; goal[2] = 0.0;
    goal[3] = -1.5; goal[4] = 0.0; goal[5] = 2.0; goal[6] = 0.785;
    
    ss.setStartAndGoalStates(start, goal);
    
    // 4. Create benchmark
    tools::Benchmark benchmark(ss, "VAMP-OMPL Benchmark");
    
    // 5. Add planners
    std::cout << "Adding planners..." << std::endl;
    benchmark.addPlanner(std::make_shared<geometric::RRTConnect>(ss.getSpaceInformation()));
    benchmark.addPlanner(std::make_shared<geometric::RRTstar>(ss.getSpaceInformation()));
    benchmark.addPlanner(std::make_shared<geometric::BITstar>(ss.getSpaceInformation()));
    benchmark.addPlanner(std::make_shared<geometric::PRM>(ss.getSpaceInformation()));
    
    // 6. Run benchmark
    tools::Benchmark::Request request;
    request.maxTime = 5.0;
    request.maxMem = 1000.0;
    request.runCount = 10;
    request.displayProgress = true;
    
    std::cout << "Running benchmark (4 planners, 10 runs each, 5s per run)..." << std::endl;
    benchmark.benchmark(request);
    
    // 7. Save results
    std::string results_file = "vamp_ompl_benchmark.log";
    benchmark.saveResultsToFile(results_file.c_str());
    
    std::cout << "\n Benchmark completed!" << std::endl;
    std::cout << "Results saved to: " << results_file << std::endl;
    
    // Instructions for processing results
    std::cout << "\nNext steps to visualize results:" << std::endl;
    std::cout << "=================================" << std::endl;
    std::cout << "\n1. Process the log file into a database:" << std::endl;
    std::cout << "   ompl_benchmark_statistics.py " << results_file << " -d vamp_benchmark.db" << std::endl;
    
    std::cout << "\n2. Visualize results (choose one):" << std::endl;
    std::cout << "   a) Upload to Planner Arena (recommended):" << std::endl;
    std::cout << "      - Visit: http://plannerarena.org" << std::endl;
    std::cout << "      - Upload vamp_benchmark.db via the web interface" << std::endl;
    
    std::cout << "\n   b) Run Planner Arena locally:" << std::endl;
    std::cout << "      plannerarena" << std::endl;
    std::cout << "      (Requires R and dependencies. See: ompl.kavrakilab.org/plannerarena.html)" << std::endl;
    
    std::cout << "\n   c) Generate PDF plots:" << std::endl;
    std::cout << "      ompl_benchmark_statistics.py -d vamp_benchmark.db -p vamp_plots.pdf" << std::endl;
    
    std::cout << "\nFor more information: ompl_benchmark_statistics.py --help" << std::endl;
}

/**
 * @brief Print usage information
 */
void printUsage(const char* program_name) {
    std::cout << "VAMP-OMPL Integration Demo" << std::endl;
    std::cout << "==========================" << std::endl;
    std::cout << "\nUsage: " << program_name << " [OPTIONS]" << std::endl;
    std::cout << "\nOptions:" << std::endl;
    std::cout << "  --simple       Run simple planning demo (default)" << std::endl;
    std::cout << "  --benchmark    Run benchmarking comparison" << std::endl;
    std::cout << "  --help         Show this help message" << std::endl;
    std::cout << "\nDescription:" << std::endl;
    std::cout << "  Demonstrates VAMP-OMPL integration using VAMPStateSpace." << std::endl;
    std::cout << "  VAMP provides SIMD-accelerated collision detection for OMPL planners." << std::endl;
    std::cout << "\nExamples:" << std::endl;
    std::cout << "  " << program_name << "              # Run simple demo" << std::endl;
    std::cout << "  " << program_name << " --benchmark  # Run benchmark" << std::endl;
}

int main(int argc, char** argv) {
    try {
        // Parse command line arguments
        std::string mode = "simple";
        
        if (argc > 1) {
            std::string arg = argv[1];
            if (arg == "--help" || arg == "-h") {
                printUsage(argv[0]);
                return 0;
            } else if (arg == "--benchmark" || arg == "-b") {
                mode = "benchmark";
            } else if (arg == "--simple" || arg == "-s") {
                mode = "simple";
            } else {
                std::cerr << "Unknown option: " << arg << std::endl;
                printUsage(argv[0]);
                return 1;
            }
        }
        
        // Run selected mode
        if (mode == "benchmark") {
            runBenchmark();
        } else {
            runSimpleDemo();
            
            // Show usage pattern
            std::cout << "\nUsage pattern:" << std::endl;
            std::cout << "  // 1. Create VAMP state space with environment" << std::endl;
            std::cout << "  auto space = std::make_shared<ompl::geometric::VAMPStateSpace<Robot>>(environment);" << std::endl;
            std::cout << "\n  // 2. Create SimpleSetup and configure validators" << std::endl;
            std::cout << "  ompl::geometric::SimpleSetup ss(space);" << std::endl;
            std::cout << "  auto si = ss.getSpaceInformation();" << std::endl;
            std::cout << "  ss.setStateValidityChecker(space->allocDefaultStateValidityChecker(si));" << std::endl;
            std::cout << "  si->setMotionValidator(space->allocDefaultMotionValidator(si));" << std::endl;
            std::cout << "\n  // 3. Everything else is standard OMPL!" << std::endl;
            std::cout << "  ss.setStartAndGoalStates(start, goal);" << std::endl;
            std::cout << "  ss.setPlanner(...);" << std::endl;
            std::cout << "  ss.solve(...);" << std::endl;
        }
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
} 