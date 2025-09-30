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
 * @file SimpleVampDemo.cpp
 * @brief Simple demonstration of VAMP-OMPL integration
 */

#include "../core/VAMPSetup.h"

#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/informedtrees/BITstar.h>

#include <vamp/robots/panda.hh>
#include <vamp/collision/factory.hh>

#include <iostream>
#include <chrono>

using namespace ompl;

/**
 * @brief Create a simple sphere cage environment
 */
::vamp::collision::Environment<float> createSphereCageEnvironment() {
    ::vamp::collision::Environment<float> environment;
    
    const std::vector<std::array<float, 3>> sphere_positions = {
        {0.55f, 0.0f, 0.25f},   {0.35f, 0.35f, 0.25f},  {0.0f, 0.55f, 0.25f},
        {-0.55f, 0.0f, 0.25f},  {-0.35f, -0.35f, 0.25f},{0.0f, -0.55f, 0.25f},
        {0.35f, -0.35f, 0.25f}, {0.35f, 0.35f, 0.8f},   {0.0f, 0.55f, 0.8f},
        {-0.35f, 0.35f, 0.8f},  {-0.55f, 0.0f, 0.8f},   {-0.35f, -0.35f, 0.8f},
        {0.0f, -0.55f, 0.8f},   {0.35f, -0.35f, 0.8f}
    };
    
    for (const auto& pos : sphere_positions) {
        environment.spheres.emplace_back(
            ::vamp::collision::factory::sphere::array(pos, 0.15f));
    }
    
    environment.sort();
    return environment;
}

int main() {
    std::cout << "VAMP-OMPL Integration Demo" << std::endl;
    std::cout << "===========================" << std::endl;
    
    try {
        // 1. Create VAMP collision environment
        std::cout << "\n1. Creating collision environment..." << std::endl;
        auto environment = createSphereCageEnvironment();
        std::cout << "   Created environment with " << environment.spheres.size() << " spheres" << std::endl;
        
        // 2. Create VAMP state space (automatically configures joint limits and validators)
        std::cout << "\n2. Creating VAMP state space..." << std::endl;
        auto space = std::make_shared<vamp::VAMPStateSpace<::vamp::robots::Panda>>(environment);
        std::cout << "   State space: " << space->getName() << " (dimension: " << space->getDimension() << ")" << std::endl;
        
        // 3. Create SimpleSetup with VAMP state space
        std::cout << "\n3. Creating SimpleSetup..." << std::endl;
        geometric::SimpleSetup ss(space);
        std::cout << "   SimpleSetup created (VAMP validators auto-configured)" << std::endl;
        
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
            std::cout << "   ✓ Solution found in " << duration.count() << " ms!" << std::endl;
            auto& path = ss.getSolutionPath();
            std::cout << "   Path: " << path.getStateCount() << " waypoints, length = " << path.length() << std::endl;
            
            ss.simplifySolution();
            std::cout << "   Simplified: " << path.getStateCount() << " waypoints, length = " << path.length() << std::endl;
        } else {
            std::cout << "   ✗ No solution found" << std::endl;
        }
        
        // 6. Try with BIT*
        std::cout << "\n6. Planning with BIT*..." << std::endl;
        ss.clear();
        ss.setStartAndGoalStates(start, goal);
        ss.setPlanner(std::make_shared<geometric::BITstar>(ss.getSpaceInformation()));
        
        start_time = std::chrono::high_resolution_clock::now();
        solved = ss.solve(5.0);
        end_time = std::chrono::high_resolution_clock::now();
        duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        
        if (solved) {
            std::cout << "   ✓ Solution found in " << duration.count() << " ms!" << std::endl;
            auto& path = ss.getSolutionPath();
            std::cout << "   Path: " << path.getStateCount() << " waypoints, length = " << path.length() << std::endl;
        } else {
            std::cout << "   ✗ No solution found" << std::endl;
        }
        
        std::cout << "\n✓ Demo completed successfully!" << std::endl;
        std::cout << "\nUsage pattern:" << std::endl;
        std::cout << "  auto space = std::make_shared<vamp::VAMPStateSpace<Robot>>(environment);" << std::endl;
        std::cout << "  geometric::SimpleSetup ss(space);" << std::endl;
        std::cout << "  // Everything else is standard OMPL!" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
} 