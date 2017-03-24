/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, Rice University
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

/* Author: Javier V. Gomez, Mark Moll */

#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/cforest/CForest.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/tools/benchmark/Benchmark.h>
#include <boost/program_options.hpp>

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace ot = ompl::tools;
namespace po = boost::program_options;

bool isStateValid(double radiusSquared, const ob::State *state)
{
    const auto *s = state->as<ob::SE2StateSpace::StateType>();
    double x=s->getX(), y=s->getY();
    x = std::abs(x - std::floor(x));
    y = std::abs(y - std::floor(y));
    x = std::min(x, 1. - x);
    y = std::min(y, 1. - y);
    return x*x + y*y > radiusSquared;
}

int main(int argc, char **argv)
{
    int distance, gridLimit, runCount;
    double obstacleRadius, turningRadius, runtimeLimit;

    auto space(std::make_shared<ob::SE2StateSpace>());

    po::options_description desc("Options");

    desc.add_options()
        ("help", "show help message")
        ("dubins", "use Dubins state space")
        ("dubinssym", "use symmetrized Dubins state space")
        ("reedsshepp", "use Reeds-Shepp state space")
        ("distance", po::value<int>(&distance)->default_value(3), "integer grid distance between start and goal")
        ("obstacle-radius", po::value<double>(&obstacleRadius)->default_value(.25), "radius of obstacles")
        ("turning-radius", po::value<double>(&turningRadius)->default_value(.5), "turning radius of robot (ignored for default point robot)")
        ("grid-limit", po::value<int>(&gridLimit)->default_value(10), "size of the grid")
        ("runtime-limit", po::value<double>(&runtimeLimit)->default_value(2), "time limit for every test")
        ("run-count", po::value<int>(&runCount)->default_value(100), "number of times to run each planner")
    ;

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help") != 0u)
    {
        std::cout << desc << "\n";
        return 1;
    }

    if (vm.count("dubins") != 0u)
        space = std::make_shared<ob::DubinsStateSpace>(turningRadius);
    if (vm.count("dubinssym") != 0u)
        space = std::make_shared<ob::DubinsStateSpace>(turningRadius, true);
    if (vm.count("reedsshepp") != 0u)
        space = std::make_shared<ob::ReedsSheppStateSpace>(turningRadius);

    // set the bounds for the R^2 part of SE(2)
    ob::RealVectorBounds bounds(2);
    bounds.setLow(-.5 * gridLimit);
    bounds.setHigh(.5 * gridLimit);
    space->setBounds(bounds);

    // define a simple setup class
    og::SimpleSetup ss(space);

    // set state validity checking for this space
    double radiusSquared = obstacleRadius * obstacleRadius;
    ss.setStateValidityChecker(
        [radiusSquared](const ob::State *state)
        {
            return isStateValid(radiusSquared, state);
        });

    // define start & goal states
    ob::ScopedState<ob::SE2StateSpace> start(space), goal(space);
    start->setXY(0., 0.5);
    start->setYaw(0.);
    goal->setXY(0., (double)distance + .5);
    goal->setYaw(0);
    ss.setStartAndGoalStates(start, goal);

    // setting collision checking resolution to 0.05 (absolute)
    ss.getSpaceInformation()->setStateValidityCheckingResolution(0.05 / gridLimit);
    ss.getProblemDefinition()->setOptimizationObjective(
        std::make_shared<ompl::base::PathLengthOptimizationObjective>(ss.getSpaceInformation()));

    // by default, use the Benchmark class
    double memoryLimit = 4096;
    ot::Benchmark::Request request(runtimeLimit, memoryLimit, runCount);
    ot::Benchmark b(ss, "CircleGrid");

    b.addPlanner(std::make_shared<og::RRTstar>(ss.getSpaceInformation()));
    b.addPlanner(std::make_shared<og::CForest>(ss.getSpaceInformation()));
    b.benchmark(request);
    b.saveResultsToFile("circleGrid.log");

    exit(0);
}


