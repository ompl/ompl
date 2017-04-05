/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2017, Rice University
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

/* Author: Zachary Kingston */

#include <ompl/tools/benchmark/Benchmark.h>
#include "ConstrainedPlanningCommon.h"

const double runtime_limit = 1;
const double memory_limit = 2048;
const int run_count = 10;
const double update_interval = 0.05;
const bool progress = false;
const bool save_output = false;
const bool use_threads = true;
const bool simplify = false;

void projectedChainBench(int links, double sleep, const char *planner)
{
    std::cout << "Beginning benchmark for ProjectedStateSpace with " + std::string(planner) << std::endl;
    std::cout << "  " + std::to_string(links) + " links & " + std::to_string(sleep) + "s artificial sleep" << std::endl;

    // Initialize the atlas
    Eigen::VectorXd x, y;
    ompl::base::StateValidityCheckerFn isValid;

    ompl::base::Constraint *constraint = initChainProblem(x, y, isValid, sleep, links);

    ompl::base::ProjectedStateSpacePtr projected(
        new ompl::base::ProjectedStateSpace(constraint->getAmbientSpace(), constraint));

    projected->setDelta(0.02);

    ompl::geometric::SimpleSetup ss(projected);
    ss.setStateValidityChecker(isValid);

    ompl::base::SpaceInformationPtr si = ss.getSpaceInformation();
    projected->setSpaceInformation(si);
    si->setValidStateSamplerAllocator(pvssa);

    {
        ompl::base::ScopedState<> start(projected);
        ompl::base::ScopedState<> goal(projected);
        start->as<ompl::base::ProjectedStateSpace::StateType>()->setRealState(x);
        goal->as<ompl::base::ProjectedStateSpace::StateType>()->setRealState(y);

        ss.setStartAndGoalStates(start, goal);
    }

    ompl::base::RealVectorBounds bounds(projected->getAmbientDimension());
    bounds.setLow(-links);
    bounds.setHigh(links);
    projected->as<ompl::base::RealVectorStateSpace>()->setBounds(bounds);

    projected->registerDefaultProjection(ompl::base::ProjectionEvaluatorPtr(new ChainProjection(projected, 3, links)));

    projected->setup();

    ompl::tools::Benchmark bench(ss, "chain");

    bench.addExperimentParameter("number_dofs", "INTEGER", std::to_string(3 * links));
    bench.addExperimentParameter("collision_check_time", "REAL", std::to_string(sleep));
    bench.addExperimentParameter("delta", "REAL", std::to_string(projected->getDelta()));

    const ompl::tools::Benchmark::Request request(runtime_limit, memory_limit, run_count, update_interval, progress,
                                                  save_output, use_threads, simplify);

    ompl::base::PlannerPtr pptr(parsePlanner(planner, si, 0.707));
    pptr->setName(pptr->getName() + "+P");

    bench.addPlanner(pptr);

    bench.setPreRunEvent([](const ompl::base::PlannerPtr &planner) {
        static std::string currentPlanner = "";
        static unsigned int run = 0;
        if (currentPlanner != planner->getName())
        {
            run = 0;
            currentPlanner = planner->getName();
        }
        std::cout << currentPlanner << " run " << run++ << "\n";

        ompl::base::ProjectedStateSpace *ss =
            planner->getSpaceInformation()->getStateSpace()->as<ompl::base::ProjectedStateSpace>();

        planner->clear();
        ss->clear();
    });

    bench.benchmark(request);

    std::string file = std::string(planner) + "+P_" + std::to_string(links) + "_" + std::to_string(sleep) + ".log";
    bench.saveResultsToFile(file.c_str());
}

void atlasChainBench(int links, double sleep, const char *planner)
{
    std::cout << "Beginning benchmark for AtlasStateSpace with " + std::string(planner) << std::endl;
    std::cout << "  " + std::to_string(links) + " links & " + std::to_string(sleep) + "s artificial sleep" << std::endl;

    Eigen::VectorXd x, y;
    ompl::base::StateValidityCheckerFn isValid;

    ompl::base::Constraint *constraint = initChainProblem(x, y, isValid, sleep, links);
    ompl::base::AtlasStateSpacePtr atlas(new ompl::base::AtlasStateSpace(constraint->getAmbientSpace(), constraint));

    atlas->setExploration(0.8);
    atlas->setRho(0.5);  // 0.2
    atlas->setAlpha(M_PI / 8);
    atlas->setEpsilon(0.2);  // 0.1
    atlas->setDelta(0.02);
    atlas->setMaxChartsPerExtension(200);

    ompl::geometric::SimpleSetup ss(atlas);
    ss.setStateValidityChecker(isValid);

    ompl::base::SpaceInformationPtr si = ss.getSpaceInformation();
    atlas->setSpaceInformation(si);
    si->setValidStateSamplerAllocator(avssa);

    {
        ompl::base::AtlasChart *startChart = atlas->anchorChart(x);
        ompl::base::AtlasChart *goalChart = atlas->anchorChart(y);
        ompl::base::ScopedState<> start(atlas);
        ompl::base::ScopedState<> goal(atlas);
        start->as<ompl::base::AtlasStateSpace::StateType>()->setRealState(x, startChart);
        goal->as<ompl::base::AtlasStateSpace::StateType>()->setRealState(y, goalChart);
        ss.setStartAndGoalStates(start, goal);
    }

    ompl::base::RealVectorBounds bounds(atlas->getAmbientDimension());
    bounds.setLow(-links);
    bounds.setHigh(links);
    atlas->as<ompl::base::RealVectorStateSpace>()->setBounds(bounds);

    atlas->registerDefaultProjection(ompl::base::ProjectionEvaluatorPtr(new ChainProjection(atlas, 3, links)));

    atlas->setup();

    ompl::tools::Benchmark bench(ss, "chain");

    bench.addExperimentParameter("number_dofs", "INTEGER", std::to_string(3 * links));
    bench.addExperimentParameter("collision_check_time", "REAL", std::to_string(sleep));
    bench.addExperimentParameter("delta", "REAL", std::to_string(atlas->getDelta()));
    const ompl::tools::Benchmark::Request request(runtime_limit, memory_limit, run_count, update_interval, progress,
                                                  save_output, use_threads, simplify);

    ompl::base::PlannerPtr pptr(parsePlanner(planner, si, 0.707));
    pptr->setName(pptr->getName() + "+A");

    bench.addPlanner(pptr);

    bench.setPreRunEvent([](const ompl::base::PlannerPtr &planner) {
        static std::string currentPlanner = "";
        static unsigned int run = 0;
        if (currentPlanner != planner->getName())
        {
            run = 0;
            currentPlanner = planner->getName();
        }
        std::cout << currentPlanner << " run " << run++ << "\n";

        ompl::base::AtlasStateSpace *ss =
            planner->getSpaceInformation()->getStateSpace()->as<ompl::base::AtlasStateSpace>();

        planner->clear();
        ss->clear();
    });

    bench.benchmark(request);

    std::string file = std::string(planner) + "+A_" + std::to_string(links) + "_" + std::to_string(sleep) + ".log";
    bench.saveResultsToFile(file.c_str());
}

void usage()
{
    std::cout << "Invalid parameters." << std::endl;
    std::cout << "  ConstrainedBenchmark <n links> <sleep> <planner> <projected/atlas>" << std::endl;
    printPlanners();
}

int main(int argc, char **argv)
{
    int links = atoi(argv[1]);
    double sleep = atof(argv[2]);

    if (strcmp(argv[4], "atlas") == 0)
        atlasChainBench(links, sleep, argv[3]);

    else if (strcmp(argv[4], "projected") == 0)
        projectedChainBench(links, sleep, argv[3]);
}
