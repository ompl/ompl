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
#include <ompl/base/spaces/VanaOwenStateSpace.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/est/EST.h>
#include <ompl/geometric/planners/sst/SST.h>
#include <ompl/geometric/planners/AnytimePathShortening.h>
#include <ompl/tools/debug/Profiler.h>
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

// This class models a world tiled by 3D spheres. The speres are 2*radius apart along
// the X-, Y-, and Z-axis.
// The radius of each sphere is .75*radius.
// The spheres are positioned at ((2*i+1)*radius, (2*j+1)*radius, (2*k+1)*radius),
// for i,j,k = ... , -2, -1, 0, 1, 2, ...
class Environment
{
public:
    Environment(Environment const &) = default;
    Environment(double radius) : radius_{radius}
    {
    }

    double distance(const ob::State *state) const
    {
        auto s = state->as<ob::VanaStateSpace::StateType>();
        double dist = 0., d;
        for (unsigned int i = 0; i < 3; ++i)
        {
            d = std::fmod(std::abs((*s)[i]), 2. * radius_) - radius_;
            dist += d * d;
        }
        return std::sqrt(dist);
    }
    bool isValid(const ob::State *state) const
    {
        return distance(state) > .75 * radius_;
    }

protected:
    double radius_;
};

class OptObjective : public ob::StateCostIntegralObjective
{
public:
    OptObjective(const ob::SpaceInformationPtr &si, const Environment &env, bool enableMotionCostInterpolation = false)
      : ob::StateCostIntegralObjective(si, enableMotionCostInterpolation), env_(env)
    {
    }
    ob::Cost stateCost(ob::State const *state) const override
    {
        return ob::Cost(1. / (env_.distance(state) + .1));
    }

protected:
    Environment env_;
};

bool checkPath(og::PathGeometric &path, Environment const &env)
{
    bool result = true;
    unsigned int numDims = path.getSpaceInformation()->getStateDimension() - 1;
    for (auto const &state : path.getStates())
        if (!env.isValid(state))
        {
            result = false;
            std::cout << env.distance(state) << ' ' << toString(state, numDims);
        }
    if (!result)
        std::cout << "Path is not valid!" << std::endl;
    return result;
}

ob::PlannerPtr allocPlanner(ob::SpaceInformationPtr const &si, std::string const &plannerName)
{
    if (plannerName == "rrtstar")
    {
        auto planner(std::make_shared<og::RRTstar>(si));
        planner->setRange(1);
        return planner;
    }
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
        auto planner = og::AnytimePathShortening::createPlanner<og::EST, og::EST, og::EST, og::EST, og::KPIECE1,
                                                                og::KPIECE1, og::KPIECE1, og::KPIECE1>(si);
        planner->setHybridize(false);
        return planner;
    }

    if (plannerName != "rrt")
        OMPL_ERROR("Unknown planner specified; defaulting to RRT");

    return std::make_shared<og::RRT>(si);
}

template <class Space>
typename Space::PathType getPath(ob::ScopedState<> const &start, ob::ScopedState<> const &goal)
{
    auto path = start.getSpace()->as<Space>()->getPath(start.get(), goal.get());
    if (!path)
    {
#if ENABLE_PROFILING == 0
        std::cout << "start: " << toString(start);
        std::cout << "goal:  " << toString(goal);
#endif
        throw std::runtime_error("Could not find a valid path");
    }
    return *path;
}

template <class Space>
void saveStatistic(std::ostream &logfile, ob::ScopedState<> const &start, ob::ScopedState<> const &goal)
{
    double length;
    unsigned int success;
    char type = '?';
    const auto& name = start.getSpace()->getName();
    try
    {
        ompl::tools::Profiler::ScopedBlock _(name);
        auto path = getPath<Space>(start, goal);
        success = 1;
        length = path.length();
        if constexpr (!std::is_same_v<Space,ob::VanaStateSpace>)
            type = (char)path.category();
    }
    catch (std::runtime_error &e)
    {
        success = 0;
        length = std::numeric_limits<double>::quiet_NaN();
    }
    logfile << ',' << success << ',' << length << ',' << type;
}

template <class Space>
void savePath(ob::ScopedState<> const &start, ob::ScopedState<> const &goal, std::string const &pathName)
{
    auto path = getPath<Space>(start, goal);
    auto space = start.getSpace()->as<Space>();
    ob::ScopedState<Space> state(start);
    // double dist = path.length(), d1, d2;
    // for (unsigned i=1; i<100; ++i)
    // {
    //     space->interpolate(start.get(), goal.get(), (double)i/50., path, st);
    //     d1 = space->distance(start.get(), state.get());
    //     d2 = space->distance(state.get(), goal.get());
    //     if (std::abs(d1 + d2 - dist) > .01)
    //     {
    //         std::cout << i << "," << dist << "," << d1 << "," << d2 << "," << d1+d2-dist << "," << toString(state) <<
    //         '\n';
    //     }
    // }

    if (!pathName.empty())
    {
        std::ofstream outfile(pathName);
        for (double t = 0.; t <= 1.001; t += .001)
        {
            space->interpolate(start.get(), goal.get(), t, path, state.get());
            outfile << t << ' ' << toString(state);
        }
    }
    std::cout << "start: " << toString(start) << "goal: " << toString(goal) << "path: " << path << '\n';
}

void computePlan(ob::ScopedState<> const &start, ob::ScopedState<> &goal, double radius, double timeLimit,
                 std::string const &plannerName, std::string const &pathName)
{
    auto const &space = start.getSpace();
    og::SimpleSetup setup(space);
    Environment env(radius);

    setup.setStartAndGoalStates(start, goal);
    setup.setStateValidityChecker([env](const ob::State *state) { return env.isValid(state); });
    // setup.setStateValidityChecker([](const ob::State*){ return true; });
    // setup.setOptimizationObjective(std::make_shared<OptObjective>(setup.getSpaceInformation(), env));
    setup.getSpaceInformation()->setStateValidityCheckingResolution(0.001);
    setup.setPlanner(allocPlanner(setup.getSpaceInformation(), plannerName));
    setup.setup();
    setup.print();
    auto result = setup.solve(timeLimit);
    if (result)
    {
        // setup.simplifySolution();
        if (!pathName.empty())
        {
            auto path = setup.getSolutionPath();
            if (!path.checkAndRepair(100).second)
                std::cout << "Path is in collision!" << std::endl;
            path.interpolate();
            if (true)  // checkPath(path, radius))
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

void benchmark(unsigned numSamples, double radius, double maxPitch)
{
    ob::RealVectorBounds bounds(3);
    bounds.setLow(-10);
    bounds.setHigh(10);

    auto owenSpace = std::make_shared<ob::OwenStateSpace>(radius, maxPitch);
    auto vanaSpace = std::make_shared<ob::VanaStateSpace>(radius, maxPitch);
    auto vanaowenSpace = std::make_shared<ob::VanaOwenStateSpace>(radius, maxPitch);
    owenSpace->setBounds(bounds);
    vanaSpace->setBounds(bounds);
    vanaowenSpace->setBounds(bounds);

    std::vector<double> start(5u, 0.);

    ob::ScopedState<> owenStart(owenSpace);
    ob::ScopedState<> owenGoal(owenSpace);
    ob::ScopedState<> vanaStart(vanaSpace);
    ob::ScopedState<> vanaGoal(vanaSpace);
    ob::ScopedState<> vanaowenStart(vanaowenSpace);
    ob::ScopedState<> vanaowenGoal(vanaowenSpace);

    owenStart = start;
    vanaStart = start;
    vanaowenStart = start;

    ompl::tools::Profiler::Start();
    std::ofstream logfile("benchmark.csv");
    logfile << "X,Y,Z,pitch,yaw,owen_success,owen_length,owen_category,vana_success,vana_length,vana_category,vanaowen_success,vanaowen_length,vanaowen_category\n";

    for (unsigned int i = 0; i < numSamples; ++i)
    {
        vanaowenGoal.random();
        vanaGoal = vanaowenGoal.reals();
        owenGoal = vanaowenGoal.reals();
        owenGoal->as<ob::OwenStateSpace::StateType>()->yaw() =
            vanaowenGoal->as<ob::VanaOwenStateSpace::StateType>()->yaw();

        logfile << vanaowenGoal[0] << ',' << vanaowenGoal[1] << ',' << vanaowenGoal[2] << ',' << vanaowenGoal[3] << ','
                << vanaowenGoal[4];

        saveStatistic<ob::OwenStateSpace>(logfile, owenStart, owenGoal);
        saveStatistic<ob::VanaStateSpace>(logfile, vanaStart, vanaGoal);
        saveStatistic<ob::VanaOwenStateSpace>(logfile, vanaowenStart, vanaowenGoal);
        logfile << '\n';
    }
}

int main(int argc, char *argv[])
{
    try
    {
        std::string pathName;
        double radius, maxPitch, timeLimit;
        unsigned numSamples;
        std::string plannerName{"rrt"};
        po::options_description desc("Options");
        // clang-format off
        desc.add_options()
            ("help", "show help message")
            ("owen", "generate a owen path starting from (x,y,z,yaw)=(0,0,0,0) to a random pose")
            ("vana", "generate a Vana path starting from (x,y,z,pitch,yaw)=(0,0,0,0,0) to a random pose")
            ("vanaowen", "generate a Vana-Owen path starting from (x,y,z,pitch,yaw)=(0,0,0,0,0) to a random pose")
            ("plan", "use a planner to plan a path to a random pose in space with obstacles")
            ("savepath", po::value<std::string>(&pathName), "save an (approximate) solution path to file") 
            ("start", po::value<std::vector<double>>()->multitoken(),
                "use (x,y,z,[pitch,]yaw) as the start")
            ("goal", po::value<std::vector<double>>()->multitoken(),
                "use (x,y,z,[pitch,]yaw) as the goal instead of a random state")
            ("radius", po::value<double>(&radius)->default_value(1.), "turn radius")
            ("maxpitch", po::value<double>(&maxPitch)->default_value(.5), "maximum pitch angle")
            ("planner", po::value<std::string>(&plannerName)->default_value("kpiece"),
                "planning algorithm to use (rrt, rrtstar, est, kpiece, sst, aps)")
            ("time", po::value<double>(&timeLimit)->default_value(10), "time limit for planning")
            ("benchmark", po::value<unsigned int>(&numSamples)->default_value(0), "benchmark performance with given number of random goal positions");
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

        ob::StateSpacePtr space = [&]() -> ob::StateSpacePtr
        {
            // set the bounds for the R^3 part of the space
            ob::RealVectorBounds bounds(3);
            bounds.setLow(-10);
            bounds.setHigh(10);
            if (vm.count("vana") != 0)
            {
                auto space = std::make_shared<ob::VanaStateSpace>(radius, maxPitch);
                // space->setTolerance(1e-16);
                space->setBounds(bounds);
                return space;
            }
            else if (vm.count("vanaowen") != 0)
            {
                auto space = std::make_shared<ob::VanaOwenStateSpace>(radius, maxPitch);
                // space->setTolerance(1e-16);
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
            Environment env(radius);
            do
            {
                goal.random();
            } while (--i > 0 && !env.isValid(goal.get()));
            if (i == 0)
                throw std::runtime_error("Could not sample a valid goal state");
        }
        else
        {
            auto goalVec = vm["goal"].as<std::vector<double>>();
            for (unsigned int i = 0; i < space->getDimension(); ++i)
                goal[i] = goalVec[i];
        }

        if (numSamples > 0)
            benchmark(numSamples, radius, maxPitch);
        else if (vm.count("plan") != 0u)
            computePlan(start, goal, radius, timeLimit, plannerName, pathName);
        else
        {
            if (vm.count("vana") != 0u)
                savePath<ob::VanaStateSpace>(start, goal, pathName);
            else if (vm.count("vanaowen") != 0u)
                savePath<ob::VanaOwenStateSpace>(start, goal, pathName);
            else
                savePath<ob::OwenStateSpace>(start, goal, pathName);
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
