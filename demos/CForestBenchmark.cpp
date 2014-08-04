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

/* Author: Javier V. Gomez - adapted from HypercubeBenchmark.cpp */

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

#include <ompl/geometric/planners/cforest/CForest.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/tools/benchmark/Benchmark.h>

#include <boost/math/constants/constants.hpp>
#include <boost/format.hpp>
#include <fstream>
#include <libgen.h>

static unsigned NDIM = 3;
static const double EDGEWIDTH = 0.1;

ompl::base::OptimizationObjectivePtr getPathLengthObjective(const ompl::base::SpaceInformationPtr& si)
{
    return ompl::base::OptimizationObjectivePtr(new ompl::base::PathLengthOptimizationObjective(si));
}

// Only states near some edges of a hypercube are valid. The valid edges form a
// narrow passage from (0,...,0) to (1,...,1). A state s is valid if there exists
// a k s.t. (a) 0<=s[k]<=1, (b) for all i<k s[i]<=EDGEWIDTH, and (c) for all i>k
// s[i]>=1-EDGEWIDTH.
bool isStateValid(const ompl::base::State *state)
{
    const ompl::base::RealVectorStateSpace::StateType *s
        = static_cast<const ompl::base::RealVectorStateSpace::StateType*>(state);
    bool foundMaxDim = false;

    for (int i = NDIM - 1; i >= 0; i--)
        if (!foundMaxDim)
        {
            if ((*s)[i] > EDGEWIDTH)
                foundMaxDim = true;
        }
        else if ((*s)[i] < (1. - EDGEWIDTH))
            return false;
        return true;
}

void addPlanner(ompl::tools::Benchmark& benchmark, ompl::base::PlannerPtr planner, double range)
{
    ompl::base::ParamSet& params = planner->params();
    if (params.hasParam(std::string("range")))
        params.setParam(std::string("range"), boost::lexical_cast<std::string>(range));
    benchmark.addPlanner(planner);
}

int main(int argc, char **argv)
{
    if(argc > 1)
        NDIM = boost::lexical_cast<size_t>(argv[1]);

    double range = EDGEWIDTH * 0.5;
    ompl::base::StateSpacePtr space(new ompl::base::RealVectorStateSpace(NDIM));
    ompl::base::RealVectorBounds bounds(NDIM);
    ompl::geometric::SimpleSetup ss(space);
    ompl::base::ScopedState<> start(space), goal(space);

    bounds.setLow(0.);
    bounds.setHigh(1.);
    space->as<ompl::base::RealVectorStateSpace>()->setBounds(bounds);
    ss.setStateValidityChecker(&isStateValid);
    ss.getSpaceInformation()->setStateValidityCheckingResolution(0.001);
    ss.getProblemDefinition()->setOptimizationObjective(getPathLengthObjective(ss.getSpaceInformation()));

    for(unsigned int i = 0; i < NDIM; ++i)
    {
        start[i] = 0.;
        goal[i] = 1.;
    }
    ss.setStartAndGoalStates(start, goal);
    

    // by default, use the Benchmark class
    double runtime_limit = 5, memory_limit = 4096;
    int run_count = 10;
    ompl::tools::Benchmark::Request request(runtime_limit, memory_limit, run_count);
    ompl::tools::Benchmark b(ss, "HyperCube");

    addPlanner(b, ompl::base::PlannerPtr(new ompl::geometric::CForest(ss.getSpaceInformation())), range);
    addPlanner(b, ompl::base::PlannerPtr(new ompl::geometric::RRTstar(ss.getSpaceInformation())), range);
    b.benchmark(request);
    b.saveResultsToFile(boost::str(boost::format("hypercube_%i_2t.log") % NDIM).c_str());

    exit(0);
}
