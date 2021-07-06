/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, University of Stuttgart
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
 *   * Neither the name of the University of Stuttgart nor the names
 *     of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written
 *     permission.
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

/* Author: Andreas Orthey */

#include "MultiLevelPlanningCommon.h"
#include "MultiLevelPlanningHyperCubeCommon.h"
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/multilevel/datastructures/PlannerMultiLevel.h>

#include <ompl/tools/benchmark/Benchmark.h>
#include <ompl/util/String.h>

#include <ompl/base/Path.h>
#include <ompl/geometric/PathGeometric.h>

const double runtime_limit = 60;
const double memory_limit = 1024 * 20;  // in MB, but does not consider free operations from prev runs
const int run_count = 10;
unsigned int curDim = 100;

using namespace ompl::geometric;
using namespace ompl::multilevel;

int main(int argc, char **argv)
{
    if (argc > 1)
    {
        curDim = std::atoi(argv[1]);
    }

    double range = edgeWidth * 0.5;
    auto space(std::make_shared<ompl::base::RealVectorStateSpace>(curDim));
    ompl::base::RealVectorBounds bounds(curDim);
    ompl::geometric::SimpleSetup ss(space);
    ompl::base::SpaceInformationPtr si = ss.getSpaceInformation();
    ompl::base::ProblemDefinitionPtr pdef = ss.getProblemDefinition();
    ompl::base::ScopedState<> start(space), goal(space);

    bounds.setLow(0.);
    bounds.setHigh(1.);
    space->setBounds(bounds);
    ss.setStateValidityChecker(std::make_shared<HyperCubeValidityChecker>(si, curDim));
    for (unsigned int i = 0; i < curDim; ++i)
    {
        start[i] = 0.;
        goal[i] = 1.;
    }
    ss.setStartAndGoalStates(start, goal);

    ompl::tools::Benchmark benchmark(ss, "HyperCube");
    benchmark.addExperimentParameter("num_dims", "INTEGER", std::to_string(curDim));

    //############################################################################
    // Load All Planner
    //############################################################################

    // MultiLevel Planner
    std::vector<int> proj = getHypercubeAdmissibleProjection(curDim);
    addPlanner(benchmark, GetMultiLevelPlanner<QRRT>(proj, si, "QRRT"), range);
    addPlanner(benchmark, GetMultiLevelPlanner<QRRTStar>(proj, si, "QRRTStar"), range);
    addPlanner(benchmark, GetMultiLevelPlanner<QMP>(proj, si, "QMP"), range);
    addPlanner(benchmark, GetMultiLevelPlanner<QMPStar>(proj, si, "QMPStar"), range);

    // Classical Planner
    addPlanner(benchmark, std::make_shared<RRT>(si), range);
    addPlanner(benchmark, std::make_shared<RRTConnect>(si), range);
    addPlanner(benchmark, std::make_shared<RRTstar>(si), range);
    addPlanner(benchmark, std::make_shared<RRTXstatic>(si), range);
    addPlanner(benchmark, std::make_shared<LazyRRT>(si), range);

    addPlanner(benchmark, std::make_shared<TRRT>(si), range);
    addPlanner(benchmark, std::make_shared<BiTRRT>(si), range);
    addPlanner(benchmark, std::make_shared<LBTRRT>(si), range);
    addPlanner(benchmark, std::make_shared<RRTsharp>(si), range);
    addPlanner(benchmark, std::make_shared<InformedRRTstar>(si), range);

    addPlanner(benchmark, std::make_shared<SORRTstar>(si), range);
    addPlanner(benchmark, std::make_shared<SBL>(si), range);
    // addPlanner(benchmark, std::make_shared<SST>(si), range); //out of memory
    addPlanner(benchmark, std::make_shared<STRIDE>(si), range);
    addPlanner(benchmark, std::make_shared<FMT>(si), range);

    addPlanner(benchmark, std::make_shared<BFMT>(si), range);
    addPlanner(benchmark, std::make_shared<BITstar>(si), range);
    addPlanner(benchmark, std::make_shared<ABITstar>(si), range);
    addPlanner(benchmark, std::make_shared<EST>(si), range);
    addPlanner(benchmark, std::make_shared<BiEST>(si), range);

    addPlanner(benchmark, std::make_shared<ProjEST>(si), range);
    addPlanner(benchmark, std::make_shared<KPIECE1>(si), range);
    addPlanner(benchmark, std::make_shared<BKPIECE1>(si), range);
    addPlanner(benchmark, std::make_shared<LBKPIECE1>(si), range);
    // addPlanner(benchmark, std::make_shared<PDST>(si), range); //OOM

    addPlanner(benchmark, std::make_shared<PRM>(si), range);
    addPlanner(benchmark, std::make_shared<PRMstar>(si), range);
    addPlanner(benchmark, std::make_shared<LazyPRMstar>(si), range);
    addPlanner(benchmark, std::make_shared<SPARS>(si), range);
    addPlanner(benchmark, std::make_shared<SPARStwo>(si), range);

    //############################################################################

    printEstimatedTimeToCompletion(numberPlanners, run_count, runtime_limit);

    ompl::tools::Benchmark::Request request;
    request.maxTime = runtime_limit;
    request.maxMem = memory_limit;
    request.runCount = run_count;
    request.simplify = false;
    request.displayProgress = false;
    numberRuns = numberPlanners * run_count;

    benchmark.setPostRunEvent(std::bind(&PostRunEvent, std::placeholders::_1, std::placeholders::_2));
    std::cout << "Run benchmark" << std::endl;
    benchmark.benchmark(request);
    benchmark.saveResultsToFile(boost::str(boost::format("hypercube_%i.log") % curDim).c_str());

    printBenchmarkResults(benchmark);

    return 0;
}
