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

/* Author: Beck Chen, Mark Moll */

/**
\file Koules.cpp
\brief This file contains an elaborate demo to solve the game of
[Koules](http://www.ucw.cz/~hubicka/koules/English/).

This problem was used to illustrate the capabilities of the PDST planner to
find trajectories for underactuated systems with drift. The details can be
found in the references below [1,2]. The physics have been made significantly
harder compared to the original game. We have tried to recreate the problem as
closely as possible to the one described in [2]. The demo can solve just one
level of Koules, all levels, or run a number of planners on one level as a
benchmarking run.

This demo illustrates also many advanced OMPL concepts, such as classes for
a custom state space, a control sampler, a projection, a state propagator,
and a goal claks-> It also demonstrates how one could put a simple bang-bang
controller inside the StatePropagator. In this demo the
(Directed)ControlSampler simply samples a target velocity vector and inside
the StatePropagator the control is chosen to drive the ship to attain this
velocity.

[1] A. M. Ladd and L. E. Kavraki, “Motion planning in the presence of drift,
underactuation and discrete system changes,” in Robotics: Science and Systems
I, (Boston, MA), pp. 233–241, MIT Press, June 2005.

[2] A. M. Ladd, Motion Planning for Physical Simulation. PhD thesis, Dept. of
Computer Science, Rice University, Houston, TX, Dec. 2006.
*/

#include "KoulesConfig.h"
#include "KoulesSetup.h"
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/tools/benchmark/Benchmark.h>
#include <ompl/config.h>
#include <boost/program_options.hpp>
#include <boost/format.hpp>
#include <fstream>

namespace ob = ompl::base;
namespace oc = ompl::control;
namespace ot = ompl::tools;
namespace po = boost::program_options;


void planOneLevel(KoulesSetup& ks, double maxTime, const std::string& plannerName,
    const std::string& outputFile)
{
    if (ks.solve(maxTime))
    {
        std::ofstream out(outputFile.c_str());
        oc::PathControl path(ks.getSolutionPath());
        path.interpolate();
        if (!path.check())
            OMPL_ERROR("Path is invalid");
        path.printAsMatrix(out);
        if (!ks.haveExactSolutionPath())
            OMPL_INFORM("Solution is approximate. Distance to actual goal is %g",
                ks.getProblemDefinition()->getSolutionDifference());
        OMPL_INFORM("Output saved in %s", outputFile.c_str());
    }

#if 0
    // Get the planner data, save the ship's (x,y) coordinates to one file and
    // the edge information to another file. This can be used for debugging
    // purposes; plotting the tree of states might give you some idea of
    // a planner's strategy.
    ob::PlannerData pd(ks.getSpaceInformation());
    ks.getPlannerData(pd);
    std::ofstream vertexFile((outputFile + "-vertices").c_str()), edgeFile((outputFile + "-edges").c_str());
    double* coords;
    unsigned numVerts = pd.numVertices(), offset = ks.getStateSpace()->getDimension() - 5;
    std::vector<unsigned int> edgeList;

    for (unsigned int i = 0; i < numVerts; ++i)
    {
        coords = pd.getVertex(i).getState()->as<ob::CompoundStateSpace::StateType>()
            ->as<ob::RealVectorStateSpace::StateType>(0)->values;
        vertexFile << coords[offset] << ' ' << coords[offset + 1] << '\n';

        pd.getEdges(i, edgeList);
        for (unsigned int j = 0; j < edgeList.size(); ++j)
            edgeFile << i << ' ' << edgeList[j] << '\n';
    }
#endif
}

void planAllLevelsRecursive(KoulesSetup& ks, double maxTime, const std::string& plannerName,
    std::vector<ob::PathPtr>& solution)
{
    double timeAttempt = maxTime / numAttempts;
    ob::PlannerStatus status;
    for (unsigned int i = 0; i < numAttempts; ++i)
    {
        ompl::time::point startTime = ompl::time::now();
        solution.clear();
        ks.clear();
        OMPL_INFORM("Attempt %d of %d to solve for %d koules",
            i + 1, numAttempts, (ks.getStateSpace()->getDimension() - 5)/4);
        status = ks.solve(timeAttempt);
        if (status != ob::PlannerStatus::EXACT_SOLUTION && numAttempts > 1)
            continue;

        ob::PathPtr path(ks.getProblemDefinition()->getSolutionPath());
        oc::PathControl* cpath = static_cast<oc::PathControl*>(path.get());
        const ob::State* goalState = cpath->getStates().back();
        std::vector<double> s, nextStart;

        if (status == ob::PlannerStatus::APPROXIMATE_SOLUTION)
        {
            cpath->interpolate();
            solution.push_back(path);
            OMPL_INFORM("Approximate solution found for %d koules",
                (ks.getStateSpace()->getDimension() - 5)/4);
            return;
        }
        ks.getStateSpace()->copyToReals(s, goalState);
        nextStart.reserve(s.size() - 4);
        for (unsigned int j = 0; j < s.size() - 5; j += 4)
            // include koule in next state if it is within workspace
            if (std::min(s[j], s[j+1]) > kouleRadius && std::max(s[j], s[j+1]) < sideLength - kouleRadius)
                for (unsigned k = 0; k < 4; ++k)
                    nextStart.push_back(s[j + k]);
        // add ship's state
        for (unsigned int j = s.size() - 5; j < s.size(); ++j)
            nextStart.push_back(s[j]);
        // make sure the problem size decreases as we recurse
        assert(nextStart.size() < s.size());

        unsigned int numKoules = (nextStart.size() - 5) / 4;
        if (numKoules > 0)
        {
            double timeElapsed = (ompl::time::now() - startTime).total_microseconds() * 1e-6;
            KoulesSetup ssNext(numKoules, plannerName, nextStart);
            planAllLevelsRecursive(ssNext, timeAttempt - timeElapsed, plannerName, solution);
        }
        if (numKoules == 0 || solution.size())
        {
            cpath->interpolate();
            solution.push_back(path);
            OMPL_INFORM("Solution found for %d koules", (s.size() - 5) / 4);
            return;
        }
    }
}

void planAllLevels(KoulesSetup& ks, double maxTime,
    const std::string& plannerName, const std::string& outputFile)
{
    std::vector<ob::PathPtr> solution;
    planAllLevelsRecursive(ks, maxTime, plannerName, solution);
    if (solution.size())
    {
        std::ofstream out(outputFile.c_str());
        for (std::vector<ob::PathPtr>::reverse_iterator p = solution.rbegin(); p != solution.rend(); p++)
            static_cast<oc::PathControl*>(p->get())->printAsMatrix(out);
        OMPL_INFORM("Output saved in %s", outputFile.c_str());
    }
}

void benchmarkOneLevel(KoulesSetup& ks, ot::Benchmark::Request request,
    const std::string& plannerName, const std::string& outputFile)
{
    // Create a benchmark class
    ompl::tools::Benchmark b(ks, "Koules experiment");
    // Add the planner to evaluate
    b.addPlanner(ks.getConfiguredPlannerInstance(plannerName));
    // Start benchmark
    b.benchmark(request);
    // Save the results
    b.saveResultsToFile(outputFile.c_str());
    OMPL_INFORM("Output saved in %s", outputFile.c_str());
}

int main(int argc, char **argv)
{
    try
    {
        unsigned int numKoules, numRuns;
        double maxTime, kouleVel;
        std::string plannerName, outputFile;
        po::options_description desc("Options");
        desc.add_options()
            ("help", "show help message")
            ("plan", "plan one level of koules")
            ("planall", "plan all levels of koules")
            ("benchmark", "benchmark one level")
            ("numkoules", po::value<unsigned int>(&numKoules)->default_value(3),
                "start from <numkoules> koules")
            ("maxtime", po::value<double>(&maxTime)->default_value(10.),
                "time limit in seconds")
            ("output", po::value<std::string>(&outputFile), "output file name")
            ("numruns", po::value<unsigned int>(&numRuns)->default_value(10),
                "number of runs for each planner in benchmarking mode")
            ("planner", po::value<std::string>(&plannerName)->default_value("kpiece"),
                "planning algorithm to use (pdst, kpiece, rrt, or est)")
            ("velocity", po::value<double>(&kouleVel)->default_value(0.),
                "initial velocity of each koule")
        ;

        po::variables_map vm;
        po::store(po::parse_command_line(argc, argv, desc,
            po::command_line_style::unix_style ^ po::command_line_style::allow_short), vm);
        po::notify(vm);

        KoulesSetup ks(numKoules, plannerName, kouleVel);
        if (vm.count("help") || argc == 1)
        {
            std::cout << "Solve the games of Koules.\nSelect one of these three options:\n"
                      << "\"--plan\", \"--planall\", or \"--benchmark\"\n\n" << desc << "\n";
            return 1;
        }

        if (outputFile.size() == 0)
        {
            std::string prefix(vm.count("plan") ? "koules_"
                : (vm.count("planall") ? "koules_1-" : "koulesBenchmark_"));
            outputFile = boost::str(boost::format("%1%%2%_%3%_%4%.dat")
                % prefix % numKoules % plannerName % maxTime);
        }
        if (vm.count("plan"))
            planOneLevel(ks, maxTime, plannerName, outputFile);
        else if (vm.count("planall"))
            planAllLevels(ks, maxTime, plannerName, outputFile);
        else if (vm.count("benchmark"))
            benchmarkOneLevel(ks, ot::Benchmark::Request(maxTime, 10000.0, numRuns),
                plannerName, outputFile);
    }
    catch(std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }
    catch(...) {
        std::cerr << "Exception of unknown type!\n";
    }

    return 0;
}
