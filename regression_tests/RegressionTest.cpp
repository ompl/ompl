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

/* Author: Ioan Sucan */

// This is copied from the latest version.
#include "../tests/geometric/2d/2DcirclesSetup.h"

#include "ompl/config.h"

#include "ompl/tools/benchmark/Benchmark.h"

#include "ompl/base/spaces/RealVectorStateProjections.h"

#include "ompl/geometric/planners/kpiece/LBKPIECE1.h"
#include "ompl/geometric/planners/kpiece/BKPIECE1.h"
#include "ompl/geometric/planners/kpiece/KPIECE1.h"
#include "ompl/geometric/planners/sbl/SBL.h"
#include "ompl/geometric/planners/sbl/pSBL.h"
#include "ompl/geometric/planners/rrt/RRT.h"
#include "ompl/geometric/planners/rrt/RRTConnect.h"
#include "ompl/geometric/planners/rrt/pRRT.h"
#include "ompl/geometric/planners/rrt/LazyRRT.h"
#include "ompl/geometric/planners/est/EST.h"
#include "ompl/geometric/planners/prm/PRM.h"

#if OMPL_MAJOR_VERSION >= 0 && OMPL_MINOR_VERSION >= 12
#include "ompl/geometric/planners/rrt/TRRT.h"
#endif

#if OMPL_MAJOR_VERSION >= 0 && OMPL_MINOR_VERSION >= 13
#include "ompl/geometric/planners/prm/LazyPRM.h"
#include "ompl/geometric/planners/pdst/PDST.h"
#include "ompl/geometric/planners/prm/SPARS.h"
#include "ompl/geometric/planners/prm/SPARStwo.h"
#endif

#if OMPL_MAJOR_VERSION >= 0 && OMPL_MINOR_VERSION >= 14
#include "ompl/geometric/planners/stride/STRIDE.h"
#endif

using namespace ompl;

#if OMPL_MAJOR_VERSION >= 0 && OMPL_MINOR_VERSION > 9
using tools::Benchmark;
#endif

static const double SOLUTION_TIME = 1.0;
static const bool VERBOSE = true;

void addPlanner(Benchmark &benchmark, const base::PlannerPtr &planner)
{
    planner->setName(planner->getName() + "-" OMPL_VERSION);
    benchmark.addPlanner(planner);
}

template<typename T>
void addPlanner(Benchmark &benchmark, const base::SpaceInformationPtr &si)
{
    addPlanner(benchmark, base::PlannerPtr(new T(si)));
}

int main(int argc, char **argv)
{
    boost::filesystem::path path(TEST_RESOURCES_DIR);

    Circles2D circles;
    circles.loadCircles((path / "circle_obstacles.txt").string());
    circles.loadQueries((path / "circle_queries.txt").string());
    base::SpaceInformationPtr si = geometric::spaceInformation2DCircles(circles);

#if OMPL_MAJOR_VERSION <= 0 && OMPL_MINOR_VERSION < 15
    // For older versions of OMPL, we are missing the constructor we need, so we hack things.
    geometric::SimpleSetup ss(si->getStateSpace());
    const_cast<base::SpaceInformationPtr&>(ss.getSpaceInformation()) = si;
    const_cast<base::ProblemDefinitionPtr&>(ss.getProblemDefinition()).reset(new base::ProblemDefinition(si));
    const_cast<geometric::PathSimplifierPtr&>(ss.getPathSimplifier()).reset(new geometric::PathSimplifier(si));
#else
    geometric::SimpleSetup ss(si);
#endif

    base::ScopedState<> start(ss.getSpaceInformation());
    base::ScopedState<> goal(ss.getSpaceInformation());
    const Circles2D::Query &q = circles.getQuery(0);
    start[0] = q.startX_;
    start[1] = q.startY_;
    goal[0] = q.goalX_;
    goal[1] = q.goalY_;
    ss.setStartAndGoalStates(start, goal, 1e-3);

    // by default, use the Benchmark class
    double runtime_limit = 1, memory_limit = 4096;
    int run_count = 100;
    Benchmark b(ss, OMPL_VERSION);

    addPlanner<geometric::EST>(b, ss.getSpaceInformation());
    addPlanner<geometric::RRT>(b, ss.getSpaceInformation());
#if OMPL_MAJOR_VERSION >= 0 && OMPL_MINOR_VERSION > 9
    Benchmark::Request request(runtime_limit, memory_limit, run_count);
    b.benchmark(request);
#else
    b.benchmark(runtime_limit, memory_limit, run_count);
#endif

    b.saveResultsToFile(OMPL_VERSION ".log");

    return 0;
}
