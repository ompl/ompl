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
 * @brief Simple demonstration of the VAMP-OMPL integration
 * 
 */

#include "../core/VAMPSetup.h"

#include <vamp/robots/panda.hh>
#include <vamp/collision/factory.hh>

#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/informedtrees/BITstar.h>

#include <iostream>
#include <chrono>

using namespace ompl::vamp;

/**
 * @brief Create a simple sphere cage environment using VAMP's factory
 */
vamp::collision::Environment<float> createSphereCageEnvironment() {
    vamp::collision::Environment<float> environment;
    
    // Create sphere cage
    const std::vector<std::array<float, 3>> sphere_positions = {
        {0.55f, 0.0f, 0.25f},
        {0.35f, 0.35f, 0.25f},
        {0.0f, 0.55f, 0.25f},
        {-0.55f, 0.0f, 0.25f},
        {-0.35f, -0.35f, 0.25f},
        {0.0f, -0.55f, 0.25f},
        {0.35f, -0.35f, 0.25f},
        {0.35f, 0.35f, 0.8f},
        {0.0f, 0.55f, 0.8f},
        {-0.35f, 0.35f, 0.8f},
        {-0.55f, 0.0f, 0.8f},
        {-0.35f, -0.35f, 0.8f},
        {0.0f, -0.55f, 0.8f},
        {0.35f, -0.35f, 0.8f}
    };
    
    const float radius = 0.15f;
    
    for (const auto& position : sphere_positions) {
        environment.spheres.emplace_back(
            vamp::collision::factory::sphere::array(position, radius)
        );
    }
    
    environment.sort();
    return environment;
}

int main() {
    std::cout << "Simple VAMP-OMPL Integration Demo" << std::endl;
    std::cout << "=================================" << std::endl;
    
    try {
        // 1. Create VAMP collision environment
        std::cout << "Creating collision environment..." << std::endl;
        auto environment = createSphereCageEnvironment();
        std::cout << "Environment created with " << environment.spheres.size() << " spheres" << std::endl;
        
        // 2. Create VAMPSetup (this sets up OMPL SimpleSetup with VAMP validators)
        std::cout << "Initializing VAMP-OMPL integration..." << std::endl;
        VAMPSetup<vamp::robots::Panda> vamp_setup(environment);
        std::cout << "VAMPSetup initialized for Panda robot" << std::endl;
        
        // 3. Set start and goal configurations using OMPL SimpleSetup
        std::cout << "Setting start and goal states..." << std::endl;
        auto& simple_setup = vamp_setup.getSimpleSetup();
        auto space = simple_setup.getStateSpace();
        
        // Create start state
        ompl::base::ScopedState<> start_state(space);
        start_state[0] = 0.0; start_state[1] = -0.785; start_state[2] = 0.0;
        start_state[3] = -2.356; start_state[4] = 0.0; start_state[5] = 1.571; start_state[6] = 0.785;
        
        // Create goal state  
        ompl::base::ScopedState<> goal_state(space);
        goal_state[0] = 2.35; goal_state[1] = 1.0; goal_state[2] = 0.0;
        goal_state[3] = -0.8; goal_state[4] = 0.0; goal_state[5] = 2.5; goal_state[6] = 0.785;
        
        simple_setup.setStartAndGoalStates(start_state, goal_state);
        std::cout << "Start and goal states configured" << std::endl;
        
        // 4. Use OMPL SimpleSetup directly for planning
        
        // Set planner
        simple_setup.setPlanner(std::make_shared<ompl::geometric::RRTConnect>(
            simple_setup.getSpaceInformation()));
        
        std::cout << "Planning with RRTConnect..." << std::endl;
        auto start_time = std::chrono::high_resolution_clock::now();
        
        ompl::base::PlannerStatus solved = simple_setup.solve(5.0);
        
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        
        if (solved) {
            std::cout << "Solution found in " << duration.count() << " ms!" << std::endl;
            
            // Get and print path info
            auto& path = simple_setup.getSolutionPath();
            std::cout << "Path has " << path.getStateCount() << " waypoints" << std::endl;
            std::cout << "Path length: " << path.length() << std::endl;
            
            // Simplify path
            std::cout << "Simplifying path..." << std::endl;
            simple_setup.simplifySolution();
            std::cout << "Simplified path has " << path.getStateCount() << " waypoints" << std::endl;
            std::cout << "Simplified path length: " << path.length() << std::endl;
            
        } else {
            std::cout << "No solution found" << std::endl;
        }
        
        // 5. Try with a different planner
        std::cout << "\nTrying with BIT* planner..." << std::endl;
        simple_setup.clear();
        simple_setup.setStartAndGoalStates(start_state, goal_state);
        
        simple_setup.setPlanner(std::make_shared<ompl::geometric::BITstar>(
            simple_setup.getSpaceInformation()));
        
        start_time = std::chrono::high_resolution_clock::now();
        solved = simple_setup.solve(5.0);
        end_time = std::chrono::high_resolution_clock::now();
        duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        
        if (solved) {
            std::cout << "BIT* solution found in " << duration.count() << " ms!" << std::endl;
            auto& path = simple_setup.getSolutionPath();
            std::cout << "BIT* path has " << path.getStateCount() << " waypoints" << std::endl;
            std::cout << "BIT* path length: " << path.length() << std::endl;
        } else {
            std::cout << "BIT* found no solution" << std::endl;
        }
        
        std::cout << "\nDemo completed successfully!" << std::endl;
        std::cout << "VAMPSetup provides VAMP collision detection to OMPL SimpleSetup." << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
} 