/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2024, Metron, Inc.
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
 *   * Neither the name of the Metron, Inc. nor the names of its
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

/* Author: Mark Moll */

#include <ompl/base/spaces/OwenStateSpace.h>
#include <ompl/base/spaces/VanaStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/est/EST.h>
#include <ompl/geometric/planners/sst/SST.h>
#include <ompl/geometric/planners/AnytimePathShortening.h>
#include <boost/program_options.hpp>
#include <cmath>
#include <fstream>

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace po = boost::program_options;

std::string toString(ob::State const *state, unsigned int numDims)
{
    const auto &st = *state->as<ob::VanaStateSpace::StateType>();
    std::stringstream s;
    for (unsigned int i = 0; i < numDims; ++i)
        s << st[i] << ' ';
    s << st.yaw() << '\n';
    return s.str();
}
std::string toString(ob::ScopedState<> const &state)
{
    return toString(state.get(), state.getSpace()->getDimension() - 1);
}

// This function models a world tiled by 3D spheres. The speres are 2*radius apart along the X-, Y-, and Z-axis.
// The radius of each sphere is .75*radius. The spheres are positioned at ((2*i+1)*radius, (2*j+1)*radius,
// (2*k+1)*radius), for i,j,k = ... , -2, -1, 0, 1, 2, ...
bool isValid(const ob::State *state, double radius)
{
    auto s = state->as<ob::VanaStateSpace::StateType>();
    double dist = 0., d;
    for (unsigned int i = 0; i < 3; ++i)
    {
        d = std::fmod(std::abs((*s)[i]), 2. * radius) - radius;
        dist += d * d;
    }
    return std::sqrt(dist) > .75 * radius;
}

bool checkPath(og::PathGeometric &path, double radius)
{
    bool result = true;
    unsigned int numDims = path.getSpaceInformation()->getStateDimension() - 1;
    for (auto const &state : path.getStates())
        if (!isValid(state, radius))
        {
            result = false;
            std::cout << toString(state, numDims);
        }
    if (!result)
        std::cout << "Path is not valid!" << std::endl;
    return result;
}

ob::PlannerPtr allocPlanner(ob::SpaceInformationPtr const &si, std::string const &plannerName)
{
    if (plannerName == "rrtstar")
        return std::make_shared<og::RRTstar>(si);
    if (plannerName == "est")
        return std::make_shared<og::EST>(si);
    if (plannerName == "kpiece")
        return std::make_shared<og::KPIECE1>(si);
    if (plannerName == "sst")
    {
        auto planner(std::make_shared<og::SST>(si));
        planner->setSelectionRadius(0.05);
        planner->setPruningRadius(0.01);
        return planner;
    }
    if (plannerName == "aps")
    {
        auto planner = og::AnytimePathShortening::createPlanner<og::EST, og::EST, og::EST, og::EST, og::KPIECE1, og::KPIECE1, og::KPIECE1, og::KPIECE1>(si);
        planner->setHybridize(false);
        return planner;
    }

    if (plannerName != "rrt")
        OMPL_ERROR("Unknown planner specified; defaulting to RRT");

    return std::make_shared<og::RRT>(si);
}

void computeOwenPath(ob::ScopedState<> const& start, ob::ScopedState<>& goal, std::string const& pathName)
{
    auto const& space = start.getSpace()->as<ob::OwenStateSpace>();
    ob::ScopedState<> state(start);
    auto path = space->owenPath(start.get(), goal.get());

    std::cout << "Owen path:\n" << path << '\n';
    double dist = path.length(), d1, d2;
    for (unsigned i=1; i<100; ++i)
    {
        space->interpolate(start.get(), goal.get(), (double)i/100., path, state.get());
        d1 = space->distance(start.get(), state.get());
        d2 = space->distance(state.get(), goal.get());
        if (std::abs(d1 + d2 - dist) > .01)
        {
            std::cout << i << "," << dist << "," << d1 << "," << d2 << "," << d1+d2-dist << "," << toString(state) << '\n';
        }
    }

    if (!pathName.empty())
    {
        std::ofstream outfile(pathName);
        for (double t = 0.; t <= 1.01; t += .01)
        {
            space->interpolate(start.get(), goal.get(), t, path, state.get());
            outfile << toString(state);
        }
    }
}

void computeVanaPath(ob::ScopedState<> const& start, ob::ScopedState<>& goal, std::string const& pathName)
{
    auto const& space = start.getSpace()->as<ob::VanaStateSpace>();
    ob::ScopedState<> state(start);
    auto path = space->vanaPath(start.get(), goal.get());
    if (!path)
    {
        std::cout << "start: " << toString(start);
        std::cout << "goal:  " << toString(goal);
        throw std::runtime_error("Could not find a valid Vana path");
    }
    std::cout << "Vana path:\n" << *path << '\n';

    // double dist = path->length(), d1, d2;
    // for (unsigned i=1; i<100; ++i)
    // {
    //     space->interpolate(start.get(), *path, (double)i/100., state.get());
    //     d1 = space->distance(start.get(), state.get());
    //     d2 = space->distance(state.get(), goal.get());
    //     if (std::abs(d1 + d2 - dist) > .01)
    //     {
    //         std::cout << i << "," << dist << "," << d1 << "," << d2 << "," << d1+d2-dist << "," << toString(state) << '\n';
    //     }
    // }

    if (!pathName.empty())
    {
        std::ofstream outfile(pathName);
        for (double t = 0.; t <= 1.01; t += .01)
        {
            space->interpolate(start.get(), *path, t, state.get());
            outfile << toString(state);
        }
    }
}

void computePlan(ob::ScopedState<> const& start, ob::ScopedState<>& goal, double radius,
    double timeLimit, std::string const& plannerName, std::string const& pathName)
{
    auto const& space = start.getSpace();
    og::SimpleSetup setup(space);
    setup.setStartAndGoalStates(start, goal);
    setup.setStateValidityChecker([radius](const ob::State *state) { return isValid(state, radius); });
    setup.setPlanner(allocPlanner(setup.getSpaceInformation(), plannerName));
    setup.setup();
    setup.print();
    auto result = setup.solve(timeLimit);
    if (result)
    {
        setup.simplifySolution();
        if (!pathName.empty())
        {
            auto path = setup.getSolutionPath();
            if (!path.check())
                std::cout << "Path is not valid!" << std::endl;
            path.interpolate();
            if (checkPath(path, radius))
            {
                std::ofstream outfile(pathName);
                path.printAsMatrix(outfile);
            }
        }
    }
    if (result == ob::PlannerStatus::APPROXIMATE_SOLUTION)
        std::cout << "Approximate solution. Distance to goal is "
                  << setup.getProblemDefinition()->getSolutionDifference() << std::endl;
    if (result)
        std::cout << "Path length is " << setup.getSolutionPath().length() << std::endl;

}

int main(int argc, char *argv[])
{
    try
    {
        std::string pathName;
        double radius, maxPitch, timeLimit;
        std::string plannerName{"rrt"};
        po::options_description desc("Options");
        // clang-format off
        desc.add_options()
            ("help", "show help message")
            ("owen", "generate a owen path starting from (x,y,z,yaw)=(0,0,0,0) to a random pose")
            ("vana", "generate a Vana path starting from (x,y,z,pitch,yaw)=(0,0,0,0,0) to a random pose")
            ("plan", "use a planner to plan a path to a random pose in space with obstacles")
            ("savepath", po::value<std::string>(&pathName), "save an (approximate) solution path to file") 
            ("start", po::value<std::vector<double>>()->multitoken(),
                "use (x,y,z,[pitch,]yaw) as the start")
            ("goal", po::value<std::vector<double>>()->multitoken(),
                "use (x,y,z,[pitch,]yaw) as the goal instead of a random state")
            ("radius", po::value<double>(&radius)->default_value(1.), "turn radius")
            ("maxpitch", po::value<double>(&maxPitch)->default_value(.5), "maximum pitch angle")
            ("planner",  po::value<std::string>(&plannerName)->default_value("kpiece"),
                "planning algorithm to use (rrt, rrtstar, est, kpiece, sst, aps)")
            ("time", po::value<double>(&timeLimit)->default_value(10), "time limit for planning");
        // clang-format on

        po::variables_map vm;
        po::store(po::parse_command_line(argc, argv, desc,
                                         po::command_line_style::unix_style ^ po::command_line_style::allow_short),
                  vm);
        po::notify(vm);

        if ((vm.count("help") != 0u) || argc == 1)
        {
            std::cout << desc << "\n";
            return 1;
        }

        ob::StateSpacePtr space = [&]() -> ob::StateSpacePtr {
            // set the bounds for the R^3 part of the space
            ob::RealVectorBounds bounds(3);
            bounds.setLow(-10);
            bounds.setHigh(10);
            if (vm.count("vana") !=0)
            {
                auto space = std::make_shared<ob::VanaStateSpace>(radius, maxPitch);
                space->setTolerance(1e-16);
                space->setBounds(bounds);
                return space;
            }
            else
            {
                auto space = std::make_shared<ob::OwenStateSpace>(radius, maxPitch);
                space->setBounds(bounds);
                return space;
            }
        }();
        ob::ScopedState<> start(space);
        ob::ScopedState<> goal(space);

        if (vm.count("start") == 0u)
        {
            for (unsigned int i = 0; i < space->getDimension(); ++i)
                start[i] = 0.;
        }
        else
        {
            auto startVec = vm["start"].as<std::vector<double>>();
            for (unsigned int i = 0; i < space->getDimension(); ++i)
                start[i] = startVec[i];
        }

        if (vm.count("goal") == 0u)
        {
            unsigned i = 100;
            do
            {
                goal.random();
            } while (--i > 0 && !isValid(goal.get(), radius));
            if (i == 0)
                throw std::runtime_error("Could not sample a valid goal state");
        }
        else
        {
            auto goalVec = vm["goal"].as<std::vector<double>>();
            for (unsigned int i = 0; i < space->getDimension(); ++i)
                goal[i] = goalVec[i];
        }

        if (vm.count("plan") != 0u)
            computePlan(start, goal, radius, timeLimit, plannerName, pathName);
        else
        {
            if (vm.count("vana") != 0u)
                computeVanaPath(start, goal, pathName);
            else
                computeOwenPath(start, goal, pathName);
        }
    }
    catch (std::exception &e)
    {
        std::cerr << "error: " << e.what() << "\n";
        return 1;
    }
    catch (...)
    {
        std::cerr << "Exception of unknown type!\n";
    }

    return 0;
}
