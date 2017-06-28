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
and a goal class. It also demonstrates how one could put a simple bang-bang
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
#include "KoulesStateSpace.h"
#include <ompl/tools/benchmark/Benchmark.h>
#include <ompl/config.h>
#include <boost/program_options.hpp>
#include <boost/format.hpp>
#include <fstream>

namespace ob = ompl::base;
namespace oc = ompl::control;
namespace ot = ompl::tools;
namespace po = boost::program_options;

void writeParams(std::ostream& out)
{
    out << sideLength << ' ' << shipRadius << ' ' << kouleRadius << ' ' << ' '
        << propagationStepSize << ' ' << shipAcceleration << ' ' << shipRotVel << ' '
        << shipDelta << ' ' << shipEps << std::endl;
}

void plan(KoulesSetup& ks, double maxTime, const std::string& outputFile)
{
    if (ks.solve(maxTime))
    {
        std::ofstream out(outputFile.c_str());
        oc::PathControl path(ks.getSolutionPath());
        path.interpolate();
        if (!path.check())
            OMPL_ERROR("Path is invalid");
        writeParams(out);
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
    unsigned numVerts = pd.numVertices();
    std::vector<unsigned int> edgeList;

    for (unsigned int i = 0; i < numVerts; ++i)
    {
        coords = pd.getVertex(i).getState()->as<KoulesStateSpace::StateType>()->values;
        vertexFile << coords[0] << ' ' << coords[1] << '\n';

        pd.getEdges(i, edgeList);
        for (unsigned int j = 0; j < edgeList.size(); ++j)
            edgeFile << i << ' ' << edgeList[j] << '\n';
    }
#endif
}


void benchmark(KoulesSetup& ks, ot::Benchmark::Request request,
    const std::string& plannerName, const std::string& outputFile)
{
    // Create a benchmark class
    ompl::tools::Benchmark b(ks, "Koules");
    b.addExperimentParameter("num_koules", "INTEGER", std::to_string(
        (ks.getStateSpace()->getDimension() - 5) / 4));
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
            ("plan", "solve the game of koules")
            ("benchmark", "benchmark the game of koules")
            ("numkoules", po::value<unsigned int>(&numKoules)->default_value(3),
                "start from <numkoules> koules")
            ("maxtime", po::value<double>(&maxTime)->default_value(10.),
                "time limit in seconds")
            ("output", po::value<std::string>(&outputFile), "output file name")
            ("numruns", po::value<unsigned int>(&numRuns)->default_value(10),
                "number of runs for each planner in benchmarking mode")
            ("planner", po::value<std::string>(&plannerName)->default_value("kpiece"),
                "planning algorithm to use (pdst, kpiece, sst, rrt, est, sycloprrt, or syclopest)")
            ("velocity", po::value<double>(&kouleVel)->default_value(0.),
                "initial velocity of each koule")
        ;

        po::variables_map vm;
        po::store(po::parse_command_line(argc, argv, desc,
            po::command_line_style::unix_style ^ po::command_line_style::allow_short), vm);
        po::notify(vm);

        KoulesSetup ks(numKoules, plannerName, kouleVel);
        if ((vm.count("help") != 0u) || argc == 1)
        {
            std::cout << "Solve the games of Koules.\nSelect one of these two options:\n"
                      << "\"--plan\", or \"--benchmark\"\n\n" << desc << "\n";
            return 1;
        }

        if (outputFile.empty())
        {
            std::string prefix(vm.count("plan") != 0u ? "koules_" : "koulesBenchmark_");
            outputFile = boost::str(boost::format("%1%%2%_%3%_%4%.dat")
                % prefix % numKoules % plannerName % maxTime);
        }
        if (vm.count("plan") != 0u)
            plan(ks, maxTime, outputFile);
        else if (vm.count("benchmark") != 0u)
            benchmark(ks, ot::Benchmark::Request(maxTime, 10000.0, numRuns),
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
