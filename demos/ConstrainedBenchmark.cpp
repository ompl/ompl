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

#include <thread>
#include <ompl/tools/benchmark/Benchmark.h>

#include "ConstrainedPlanningCommon.h"

const char *planners[] = {"EST", "PRM", "RRT", "RRTConnect", "KPIECE1"};

/** Print usage information. Does not return. */
void usage(const char *const progname)
{
    std::cout << "Usage: " << progname << " <problem> <timelimit> <runcount>\n";
    printProblems();
    exit(0);
}

void atlasBench(int argc, char **argv)
{
    // Initialize the atlas
    Eigen::VectorXd x, y;
    ompl::base::StateValidityCheckerFn isValid;

    ompl::base::Constraint *constraint = parseProblem(argv[1], x, y, isValid);
    ompl::base::AtlasStateSpacePtr atlas(new ompl::base::AtlasStateSpace(constraint->getAmbientSpace(), constraint));
    if (!atlas)
        usage(argv[0]);

    // All the 'Constrained' classes are loose wrappers for the normal
    // classes. No effect except on the two special planners.
    ompl::geometric::SimpleSetup ss(atlas);
    ompl::base::SpaceInformationPtr si = ss.getSpaceInformation();
    atlas->setSpaceInformation(si);
    ss.setStateValidityChecker(isValid);
    si->setValidStateSamplerAllocator(avssa);

    // Atlas parameters
    atlas->setExploration(0.5);
    atlas->setRho(0.5);  // 0.2
    atlas->setAlpha(M_PI / 8);
    atlas->setEpsilon(0.2);  // 0.1
    atlas->setDelta(0.02);
    atlas->setMaxChartsPerExtension(200);

    // The atlas needs some place to start sampling from, so we manually seed
    // charts at the start and goal.
    {
        ompl::base::AtlasChart *startChart = atlas->anchorChart(x);
        ompl::base::AtlasChart *goalChart = atlas->anchorChart(y);
        ompl::base::ScopedState<> start(atlas);
        ompl::base::ScopedState<> goal(atlas);
        start->as<ompl::base::AtlasStateSpace::StateType>()->setRealState(x, startChart);
        goal->as<ompl::base::AtlasStateSpace::StateType>()->setRealState(y, goalChart);
        ss.setStartAndGoalStates(start, goal);
    }

    // Bounds
    ompl::base::RealVectorBounds bounds(atlas->getAmbientDimension());
    bounds.setLow(-10);
    bounds.setHigh(10);
    atlas->as<ompl::base::RealVectorStateSpace>()->setBounds(bounds);
    atlas->setup();

    // Set up the benchmark for all the planners
    ompl::tools::Benchmark bench(ss, argv[1]);
    const double runtime_limit = std::atof(argv[2]);
    if (runtime_limit <= 0)
        usage(argv[0]);
    const double memory_limit = 2048;
    const int run_count = std::atoi(argv[3]);
    if (run_count < 1)
        usage(argv[0]);
    const double update_interval = 0.05;
    const bool progress = true;
    const bool save_output = false;
    const bool use_threads = true;
    const ompl::tools::Benchmark::Request request(runtime_limit, memory_limit, run_count, update_interval, progress, save_output, use_threads, false);
    for (auto & planner : planners)
    {
        ompl::base::PlannerPtr pptr(parsePlanner(planner, si, atlas->getRho_s()));
        pptr->setName(pptr->getName() +"+A");
        bench.addPlanner(pptr);
    }
    bench.setPreRunEvent([](const ompl::base::PlannerPtr &planner)
                         {
                             ompl::base::AtlasStateSpace *ss =
                                 planner->getSpaceInformation()->getStateSpace()->as<ompl::base::AtlasStateSpace>();
                             ss->clear();
                         });
    // Execute
    bench.benchmark(request);
    bench.saveResultsToFile("atlas.log");
}

void projBench(int argc, char **argv)
{
    // Initialize the atlas
    Eigen::VectorXd x, y;
    ompl::base::StateValidityCheckerFn isValid;

    ompl::base::Constraint *constraint = parseProblem(argv[1], x, y, isValid);
    ompl::base::ProjectedStateSpacePtr proj(new ompl::base::ProjectedStateSpace(constraint->getAmbientSpace(), constraint));
    if (!proj)
        usage(argv[0]);

    // All the 'Constrained' classes are loose wrappers for the normal
    // classes. No effect except on the two special planners.
    ompl::geometric::SimpleSetup ss(proj);
    ompl::base::SpaceInformationPtr si = ss.getSpaceInformation();
    proj->setSpaceInformation(si);
    ss.setStateValidityChecker(isValid);
    si->setValidStateSamplerAllocator(pvssa);

    // The proj needs some place to start sampling from, so we manually seed
    // charts at the start and goal.
    {
        ompl::base::ScopedState<> start(proj);
        ompl::base::ScopedState<> goal(proj);
        start->as<ompl::base::ProjectedStateSpace::StateType>()->setRealState(x);
        goal->as<ompl::base::ProjectedStateSpace::StateType>()->setRealState(y);
        ss.setStartAndGoalStates(start, goal);
    }

    // Bounds
    ompl::base::RealVectorBounds bounds(proj->getAmbientDimension());
    bounds.setLow(-10);
    bounds.setHigh(10);
    proj->as<ompl::base::RealVectorStateSpace>()->setBounds(bounds);
    proj->setup();

    // Set up the benchmark for all the planners
    ompl::tools::Benchmark bench(ss, argv[1]);
    const double runtime_limit = std::atof(argv[2]);
    if (runtime_limit <= 0)
        usage(argv[0]);
    const double memory_limit = 2048;
    const int run_count = std::atoi(argv[3]);
    if (run_count < 1)
        usage(argv[0]);
    const double update_interval = 0.05;
    const bool progress = true;
    const bool save_output = false;
    const bool use_threads = true;
    const ompl::tools::Benchmark::Request request(runtime_limit, memory_limit, run_count, update_interval, progress, save_output, use_threads, false);
    for (auto & planner : planners)
    {
        ompl::base::PlannerPtr pptr(parsePlanner(planner, si, 0.707));
        pptr->setName(pptr->getName() +"+P");
        bench.addPlanner(pptr);
    }
    bench.setPreRunEvent([](const ompl::base::PlannerPtr &planner)
                         {
                             ompl::base::ProjectedStateSpace *ss =
                                 planner->getSpaceInformation()->getStateSpace()->as<ompl::base::ProjectedStateSpace>();
                             ss->clear();
                         });

    // Execute
    bench.benchmark(request);
    bench.saveResultsToFile("proj.log");
}

void projectedChainBench(int links, double sleep)
{
    // Initialize the atlas
    Eigen::VectorXd x, y;
    ompl::base::StateValidityCheckerFn isValid;

    ompl::base::Constraint *constraint = initChainProblem(x, y, isValid, sleep, links);

    ompl::base::ProjectedStateSpacePtr projected(new ompl::base::ProjectedStateSpace(constraint->getAmbientSpace(), constraint));

    // All the 'Constrained' classes are loose wrappers for the normal
    // classes. No effect except on the two special planners.
    ompl::geometric::SimpleSetup ss(projected);
    ompl::base::SpaceInformationPtr si = ss.getSpaceInformation();

    projected->setSpaceInformation(si);
    ss.setStateValidityChecker(isValid);
    si->setValidStateSamplerAllocator(pvssa);

    // Projected parameters
    projected->setDelta(0.02);

    // The atlas needs some place to start sampling from, so we manually seed
    // charts at the start and goal.
    {
        ompl::base::ScopedState<> start(projected);
        ompl::base::ScopedState<> goal(projected);
        start->as<ompl::base::ProjectedStateSpace::StateType>()->setRealState(x);
        goal->as<ompl::base::ProjectedStateSpace::StateType>()->setRealState(y);
        ss.setStartAndGoalStates(start, goal);
    }

    // Bounds
    ompl::base::RealVectorBounds bounds(projected->getAmbientDimension());

    int bound = links - 2;
    bounds.setLow(-bound);
    bounds.setHigh(bound);

    projected->as<ompl::base::RealVectorStateSpace>()->setBounds(bounds);
    projected->setup();

    // Set up the benchmark for all the planners
    ompl::tools::Benchmark bench(ss, "chain");

    bench.addExperimentParameter("number_dofs", "INTEGER", std::to_string(links));
    bench.addExperimentParameter("collision_check_time", "REAL", std::to_string(sleep));

    const double runtime_limit = 60;
    const double memory_limit = 2048;
    const int run_count = 100;
    const double update_interval = 0.05;
    const bool progress = true;
    const bool save_output = false;
    const bool use_threads = true;
    const ompl::tools::Benchmark::Request request(runtime_limit, memory_limit, run_count, update_interval, progress, save_output, use_threads, false);

    for (auto & planner : planners)
    {
        ompl::base::PlannerPtr pptr(parsePlanner(planner, si, 0.707));
        pptr->setName(pptr->getName() + "+P");
        bench.addPlanner(pptr);
    }

    bench.setPreRunEvent([](const ompl::base::PlannerPtr &planner) {
            ompl::base::ProjectedStateSpace *ss =
                planner->getSpaceInformation()->getStateSpace()->as<ompl::base::ProjectedStateSpace>();
            ss->clear();
        });

    // Execute
    bench.benchmark(request);
    std::string file = "P" + std::to_string(links) + "_" + std::to_string(sleep) + ".log";
    bench.saveResultsToFile(file.c_str());
}

void atlasChainBench(int links, double sleep)
{
    // Initialize the atlas
    Eigen::VectorXd x, y;
    ompl::base::StateValidityCheckerFn isValid;

    ompl::base::Constraint *constraint = initChainProblem(x, y, isValid, sleep, links);

    ompl::base::AtlasStateSpacePtr atlas(new ompl::base::AtlasStateSpace(constraint->getAmbientSpace(), constraint));

    // All the 'Constrained' classes are loose wrappers for the normal
    // classes. No effect except on the two special planners.
    ompl::geometric::SimpleSetup ss(atlas);
    ompl::base::SpaceInformationPtr si = ss.getSpaceInformation();

    atlas->setSpaceInformation(si);
    ss.setStateValidityChecker(isValid);
    si->setValidStateSamplerAllocator(avssa);

    // Atlas parameters
    atlas->setExploration(0.5);
    atlas->setRho(0.5);  // 0.2
    atlas->setAlpha(M_PI / 8);
    atlas->setEpsilon(0.2);  // 0.1
    atlas->setDelta(0.02);
    atlas->setMaxChartsPerExtension(200);

    // The atlas needs some place to start sampling from, so we manually seed
    // charts at the start and goal.
    {
        ompl::base::AtlasChart *startChart = atlas->anchorChart(x);
        ompl::base::AtlasChart *goalChart = atlas->anchorChart(y);
        ompl::base::ScopedState<> start(atlas);
        ompl::base::ScopedState<> goal(atlas);
        start->as<ompl::base::AtlasStateSpace::StateType>()->setRealState(x, startChart);
        goal->as<ompl::base::AtlasStateSpace::StateType>()->setRealState(y, goalChart);
        ss.setStartAndGoalStates(start, goal);
    }

    // Bounds
    ompl::base::RealVectorBounds bounds(atlas->getAmbientDimension());

    int bound = links - 2;
    bounds.setLow(-bound);
    bounds.setHigh(bound);

    atlas->as<ompl::base::RealVectorStateSpace>()->setBounds(bounds);
    atlas->setup();

    // Set up the benchmark for all the planners
    ompl::tools::Benchmark bench(ss, "chain");

    bench.addExperimentParameter("number_dofs", "INTEGER", std::to_string(links));
    bench.addExperimentParameter("collision_check_time", "REAL", std::to_string(sleep));

    const double runtime_limit = 60;
    const double memory_limit = 2048;
    const int run_count = 100;
    const double update_interval = 0.05;
    const bool progress = true;
    const bool save_output = false;
    const bool use_threads = true;
    const ompl::tools::Benchmark::Request request(runtime_limit, memory_limit, run_count, update_interval, progress, save_output, use_threads, false);

    for (auto & planner : planners)
    {
        ompl::base::PlannerPtr pptr(parsePlanner(planner, si, atlas->getRho_s()));
        pptr->setName(pptr->getName() + "+A");
        bench.addPlanner(pptr);
    }

    bench.setPreRunEvent([](const ompl::base::PlannerPtr &planner) {
            ompl::base::AtlasStateSpace *ss =
                planner->getSpaceInformation()->getStateSpace()->as<ompl::base::AtlasStateSpace>();
            ss->clear();
        });

    // Execute
    bench.benchmark(request);
    std::string file = "A" + std::to_string(links) + "_" + std::to_string(sleep) + ".log";
    bench.saveResultsToFile(file.c_str());
}

int main(int argc, char **argv)
{
    std::thread t[4];

    for (int i = 1; i < 5; ++i)
    {
        t[i - 1] = std::thread([&](){
                for (int links = 4 * i; links < 4 * (i + 1); ++links)
                {
                    for (double sleep = 0.0; sleep < 0.6; sleep += 0.1)
                    {
                        projectedChainBench(links, sleep);
                        atlasChainBench(links, sleep);
                    }
                }
            });
    }

    for (int i = 0; i < 4; ++i)
    {
        t[i].join();
    }
}
