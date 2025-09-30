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
 * @brief VAMP-OMPL benchmarking example
 * 
 * VAMP state space with OMPL's benchmarking.
 */

#include "../core/VAMPStateSpace.h"

#include <ompl/tools/benchmark/Benchmark.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/informedtrees/BITstar.h>
#include <ompl/geometric/planners/prm/PRM.h>

#include <iostream>

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

int main(int argc, char** argv) {
    std::cout << "VAMP-OMPL Benchmarking Example" << std::endl;
    std::cout << "==============================" << std::endl;
    
    try {
        // 1. Create environment and state space
        auto environment = createSphereCageEnvironment();
        auto space = std::make_shared<geometric::VAMPStateSpace<vamp::robots::Panda>>(environment);
        
        // 2. Create SimpleSetup and configure VAMP validators
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
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
} 