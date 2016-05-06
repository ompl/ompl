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

/* Author: Caleb Voss */

#include <ompl/tools/benchmark/Benchmark.h>

#include "AtlasCommon.h"

/** Print usage information. Does not return. */
void usage (const char *const progname)
{
    std::cout << "Usage: " << progname << " <problem> <timelimit> <runcount>\n";
    printProblems();
    exit(0);
}

/** To be called between planner runs to clear all charts out of the atlas.
 * Also makes the atlas behave just like RealVectorStateSpace for two of the planners. */
void resetStateSpace (const ompl::base::PlannerPtr &planner)
{
    static std::string cur_planner = "";
    static unsigned int run = 0;
    if (cur_planner != planner->getName())
    {
        run = 0;
        cur_planner = planner->getName();
    }
    std::cout << cur_planner << " run " << run++ << "\n";
    ompl::base::AtlasStateSpace *atlas = planner->getSpaceInformation()->getStateSpace()->as<ompl::base::AtlasStateSpace>();
    atlas->clear();
    if (std::strcmp(cur_planner.c_str(), "ConstrainedRRT") == 0 || std::strcmp(cur_planner.c_str(), "CBiRRT2") == 0)
        atlas->stopBeingAnAtlas(true);
    else
        atlas->stopBeingAnAtlas(false);
}

int main (int argc, char **argv)
{
    if (argc != 4)
        usage(argv[0]);
    
    // Initialize the atlas
    Eigen::VectorXd x, y;
    ompl::base::StateValidityCheckerFn isValid;
    ompl::base::AtlasStateSpacePtr atlas(parseProblem(argv[1], x, y, isValid));
    if (!atlas)
        usage(argv[0]);
    
    // All the 'Constrained' classes are loose wrappers for the normal classes. No effect except on the two special planners.
    ompl::geometric::ConstrainedSimpleSetup ss(atlas);
    ompl::base::ConstrainedSpaceInformationPtr si = ss.getConstrainedSpaceInformation();
    atlas->setSpaceInformation(si);
    ss.setStateValidityChecker(isValid);
    si->setValidStateSamplerAllocator(std::bind(vssa, atlas, std::placeholders::_1));
    ompl::base::ConstraintInformationPtr ci(new ompl::base::ConstraintInformation);
    ompl::base::ConstraintPtr c(new ompl::base::AtlasConstraint(atlas));
    ci->addConstraint(c);
    si->setConstraintInformation(ci);
    
    // Atlas parameters
    atlas->setExploration(0.8);
    atlas->setRho(0.5); // 0.2
    atlas->setAlpha(M_PI/8);
    atlas->setEpsilon(0.2); // 0.1
    atlas->setDelta(0.02);
    atlas->setMaxChartsPerExtension(200);
    
    // The atlas needs some place to start sampling from. We will make start and goal charts.
    ompl::base::AtlasChart &startChart = atlas->anchorChart(x);
    ompl::base::AtlasChart &goalChart = atlas->anchorChart(y);
    ompl::base::ScopedState<> start(atlas);
    ompl::base::ScopedState<> goal(atlas);
    start->as<ompl::base::AtlasStateSpace::StateType>()->setRealState(x, &startChart);
    goal->as<ompl::base::AtlasStateSpace::StateType>()->setRealState(y, &goalChart);
    ss.setStartAndGoalStates(start, goal);
    
    // Bounds
    ompl::base::RealVectorBounds bounds(atlas->getAmbientDimension());
    bounds.setLow(-10);
    bounds.setHigh(10);
    atlas->as<ompl::base::RealVectorStateSpace>()->setBounds(bounds);
    atlas->setup();
    
    // Set up the benchmark for all the planners
    ompl::tools::Benchmark bench(ss, "Atlas");
    const double runtime_limit = std::atof(argv[2]);
    if (runtime_limit <= 0)
        usage(argv[0]);
    const double memory_limit = 2048;
    const int run_count = std::atoi(argv[3]);
    if (run_count < 1)
        usage(argv[0]);
    const double update_interval = 0.1;
    const bool progress = false;
    const bool save_output = false;
    const bool use_threads = true;
    const ompl::tools::Benchmark::Request request(runtime_limit, memory_limit, run_count, update_interval, progress, save_output, use_threads);
    const char *planners[] = {"CBiRRT2", "EST", "PRM", "RRT", "RRTintermediate", "RRTConnect", "KPIECE1"};
    for (std::size_t i = 0; i < sizeof(planners)/sizeof(char *); i++)
        bench.addPlanner(ompl::base::PlannerPtr(parsePlanner(planners[i], si, atlas->getRho_s())));
    bench.setPreRunEvent(&resetStateSpace);
    
    // Execute
    bench.benchmark(request);
    bench.saveResultsToFile("atlas.log");
    
    return 0;
}
