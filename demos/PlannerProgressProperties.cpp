/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2013, Rice University
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
*   * Neither the name of the Rice University nor the names of its
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

/* Author: Luis G. Torres */

#include <ompl/geometric/SimpleSetup.h>
#include <ompl/tools/benchmark/Benchmark.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

/// @cond IGNORE
//
// In this demo we define a simple 2D planning problem to get from
// (0,0) to (1,1) without obstacles. We're interested in collecting
// data over the execution of a planner using the Benchmark class.
//
// This program will output two files: a .console file, and a .log
// file. You can generate plots in a file called "plot.pdf" from this
// experiment using the ompl/scripts/ompl_benchmark_statistics.py
// script and the .log file like so:
//
// python path/to/ompl/scripts/ompl_benchmark_statistics.py <your_log_filename>.log -p plot.pdf
//
// Toward the bottom of the generated plot.pdf file, you'll see plots
// for how various properties change, on average, during planning
// runs.
int main(int, char**)
{
    auto space(std::make_shared<ob::RealVectorStateSpace>(2));
    space->setBounds(0, 1);
    og::SimpleSetup ss(space);

    // Set our robot's starting state to be the bottom-left corner of
    // the environment, or (0,0).
    ob::ScopedState<> start(space);
    start->as<ob::RealVectorStateSpace::StateType>()->values[0] = 0.0;
    start->as<ob::RealVectorStateSpace::StateType>()->values[1] = 0.0;

    // Set our robot's goal state to be the top-right corner of the
    // environment, or (1,1).
    ob::ScopedState<> goal(space);
    goal->as<ob::RealVectorStateSpace::StateType>()->values[0] = 1.0;
    goal->as<ob::RealVectorStateSpace::StateType>()->values[1] = 1.0;

    // Goal tolerance is 0.05
    ss.setStartAndGoalStates(start, goal, 0.05);

    // Create a new Benchmark object and set the RRTstar algorithm as
    // the planner. We're using RRTstar because currently, only the
    // RRTstar algorithm reports interesting planner progress
    // properties.
    ompl::tools::Benchmark b(ss, "my experiment");
    auto rrt(std::make_shared<og::RRTstar>(ss.getSpaceInformation()));
    rrt->setName("rrtstar");

    // We disable goal biasing so that the straight-line path doesn't
    // get found immediately. We want to demonstrate the steady
    // downward trend in path cost that characterizes RRTstar.
    rrt->setGoalBias(0.0);

    b.addPlanner(rrt);

    // Here we specify options for this benchmark. Maximum time spent
    // per planner execution is 2.0 seconds, maximum memory is 100MB,
    // number of planning runs is 50, planner progress properties will
    // be queried every 0.01 seconds, and a progress bar will be
    // displayed to show how the benchmarking is going.
    ompl::tools::Benchmark::Request req;
    req.maxTime = 2.0;
    req.maxMem = 100;
    req.runCount = 50;
    req.timeBetweenUpdates = 0.01;
    req.displayProgress = true;
    b.benchmark(req);

    b.saveResultsToFile();

    return 0;

}
