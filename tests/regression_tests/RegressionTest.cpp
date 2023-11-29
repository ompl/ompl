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

#include "ompl/config.h"

#ifndef OMPL_VERSION_VALUE
#define OMPL_VERSION_VALUE ( OMPL_MAJOR_VERSION * 1000000       \
                             + OMPL_MINOR_VERSION * 1000        \
                             + OMPL_PATCH_VERSION)
#endif

// This is copied from the latest version.
#include "ompl/util/DisableCompilerWarning.h"
OMPL_PUSH_DISABLE_CLANG_WARNING(-Wunused-function)
OMPL_PUSH_DISABLE_GCC_WARNING(-Wunused-function)
#include "../geometric/2d/2DcirclesSetup.h"
OMPL_POP_CLANG

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
#include "ompl/geometric/planners/prm/PRM.h"

#if OMPL_VERSION_VALUE >= 12000
#include "ompl/geometric/planners/rrt/TRRT.h"
#endif

#if OMPL_VERSION_VALUE >= 13000
#include "ompl/geometric/planners/prm/LazyPRM.h"
#include "ompl/geometric/planners/pdst/PDST.h"
#include "ompl/geometric/planners/prm/SPARS.h"
#include "ompl/geometric/planners/prm/SPARStwo.h"
#endif

#if OMPL_VERSION_VALUE >= 14000
#include "ompl/geometric/planners/stride/STRIDE.h"
#endif

#if OMPL_VERSION_VALUE >= 1002000
#include "ompl/geometric/planners/est/EST.h"
#include "ompl/geometric/planners/est/BiEST.h"
#include "ompl/geometric/planners/est/ProjEST.h"
#endif

using namespace ompl;

#if OMPL_VERSION_VALUE > 9000
using tools::Benchmark;
#endif

template<unsigned int PROBLEM>
std::string problemName() { return ""; }

template<typename T, unsigned int PROBLEM>
void addPlanner(Benchmark &benchmark, const base::SpaceInformationPtr &si)
{
    benchmark.addPlanner(std::make_shared<T>(si));
}

#include "RegressionTestCirclesProblem.inl.h"

template<unsigned int PROBLEM>
void addAllPlanners(Benchmark &b, geometric::SimpleSetup &ss)
{
    // EST
    addPlanner<geometric::EST, PROBLEM>(b, ss.getSpaceInformation());
    // SBL
    addPlanner<geometric::SBL, PROBLEM>(b, ss.getSpaceInformation());
    // RRT
    addPlanner<geometric::RRT, PROBLEM>(b, ss.getSpaceInformation());
    // RRTConnect
    addPlanner<geometric::RRTConnect, PROBLEM>(b, ss.getSpaceInformation());

    // KPIECE
    addPlanner<geometric::KPIECE1, PROBLEM>(b, ss.getSpaceInformation());
    addPlanner<geometric::BKPIECE1, PROBLEM>(b, ss.getSpaceInformation());
    addPlanner<geometric::LBKPIECE1, PROBLEM>(b, ss.getSpaceInformation());

    // PRM
    addPlanner<geometric::PRM, PROBLEM>(b, ss.getSpaceInformation());

    // PDST
#if OMPL_VERSION_VALUE >= 13000
    addPlanner<geometric::PDST, PROBLEM>(b, ss.getSpaceInformation());
#endif
}

// Setup a problem from the known set of problems included with the regression tests.
template<unsigned int PROBLEM>
std::shared_ptr<geometric::SimpleSetup> setupProblem()
{
    if (PROBLEM == CIRCLES_ID)
        return setupCirclesProblem(0);
    fprintf(stderr, "Unknown problem '%d'", PROBLEM);
    return std::shared_ptr<geometric::SimpleSetup>();
}

template<unsigned int PROBLEM>
void runProblem(double runtime_limit, double memory_limit, int run_count)
{
    std::shared_ptr<geometric::SimpleSetup> ss = setupProblem<PROBLEM>();
    if (ss)
    {
        const std::string exp_name = problemName<PROBLEM>();
        Benchmark b(*ss, exp_name);
        addAllPlanners<PROBLEM>(b, *ss);

#if OMPL_VERSION_VALUE > 9000
        Benchmark::Request request(runtime_limit, memory_limit, run_count);
        b.benchmark(request);
#else
        b.benchmark(runtime_limit, memory_limit, run_count);
#endif

        b.saveResultsToFile((exp_name + OMPL_VERSION + ".log").c_str());
    }
    else
    {
        fprintf(stderr, "Unable to run problem '%d'", PROBLEM);
    }
}

int main(int, char **)
{
    double runtime_limit = 1, memory_limit = 4096;
    int run_count = 1000;
    runProblem<CIRCLES_ID>(runtime_limit, memory_limit, run_count);

    return 0;
}
