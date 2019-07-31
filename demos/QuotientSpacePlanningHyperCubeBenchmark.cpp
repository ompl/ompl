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

#include "QuotientSpacePlanningCommon.h"
#include <ompl/base/spaces/RealVectorStateSpace.h>

#include <ompl/geometric/planners/quotientspace/MultiQuotient.h>
#include <ompl/geometric/planners/quotientspace/QRRT.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTsharp.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/RRTXstatic.h>
#include <ompl/geometric/planners/rrt/LazyRRT.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/rrt/TRRT.h>
#include <ompl/geometric/planners/rrt/BiTRRT.h>
#include <ompl/geometric/planners/rrt/LBTRRT.h>
#include <ompl/geometric/planners/rrt/SORRTstar.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/prm/LazyPRM.h>
#include <ompl/geometric/planners/prm/LazyPRMstar.h>
#include <ompl/geometric/planners/prm/SPARS.h>
#include <ompl/geometric/planners/prm/SPARStwo.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/kpiece/BKPIECE1.h>
#include <ompl/geometric/planners/kpiece/LBKPIECE1.h>
#include <ompl/geometric/planners/fmt/FMT.h>
#include <ompl/geometric/planners/fmt/BFMT.h>
#include <ompl/geometric/planners/est/EST.h>
#include <ompl/geometric/planners/est/BiEST.h>
#include <ompl/geometric/planners/est/ProjEST.h>
#include <ompl/geometric/planners/sbl/SBL.h>
#include <ompl/geometric/planners/sbl/pSBL.h>
#include <ompl/geometric/planners/stride/STRIDE.h>
#include <ompl/geometric/planners/sst/SST.h>
#include <ompl/geometric/planners/pdst/PDST.h>
#include <ompl/geometric/planners/bitstar/BITstar.h>

#include <ompl/tools/benchmark/Benchmark.h>
#include <ompl/util/String.h>

#include <boost/math/constants/constants.hpp>
#include <boost/format.hpp>
#include <fstream>

const double edgeWidth = 0.1;
const unsigned ndim = 10;
const double runtime_limit = 10;
const double memory_limit = 4096*4096;
const int run_count = 10;

// Only states near some edges of a hypercube are valid. The valid edges form a
// narrow passage from (0,...,0) to (1,...,1). A state s is valid if there exists
// a k s.t. (a) 0<=s[k]<=1, (b) for all i<k s[i]<=edgeWidth, and (c) for all i>k
// s[i]>=1-edgewidth.
class HyperCubeValidityChecker : public ompl::base::StateValidityChecker
{
public:
    HyperCubeValidityChecker(const ompl::base::SpaceInformationPtr &si, int nDim) : ompl::base::StateValidityChecker(si), nDim_(nDim)
    {
    }

    bool isValid(const ompl::base::State *state) const override
    {
        const auto *s
            = static_cast<const ompl::base::RealVectorStateSpace::StateType*>(state);
        bool foundMaxDim = false;

        for (int i = nDim_ - 1; i >= 0; i--)
            if (!foundMaxDim)
            {
                if ((*s)[i] > edgeWidth)
                    foundMaxDim = true;
            }
            else if ((*s)[i] < (1. - edgeWidth))
                return false;
        return true;
    }
protected:
    int nDim_;
};


void addPlanner(ompl::tools::Benchmark& benchmark, const ompl::base::PlannerPtr& planner, double range)
{
    ompl::base::ParamSet& params = planner->params();
    if (params.hasParam(std::string("range")))
        params.setParam(std::string("range"), ompl::toString(range));
    benchmark.addPlanner(planner);
}

ob::PlannerPtr GetQRRT(
    ob::SpaceInformationPtr si, 
    ob::ProblemDefinitionPtr pdef, 
    unsigned numLinks)
{
    // ompl::msg::setLogLevel(ompl::msg::LOG_DEV2);
    std::vector<ob::SpaceInformationPtr> si_vec;

    for(unsigned k = 2; k < numLinks; k+=2)
    {
        OMPL_INFORM("Create QuotientSpace Chain with %d links.", k);

        auto spaceK(std::make_shared<ompl::base::RealVectorStateSpace>(k));
        ompl::base::RealVectorBounds bounds(k);
        bounds.setLow(0.);
        bounds.setHigh(1.);
        spaceK->setBounds(bounds);

        auto siK = std::make_shared<ob::SpaceInformation>(spaceK);
        siK->setStateValidityChecker(std::make_shared<HyperCubeValidityChecker>(siK, k));
        siK->setStateValidityCheckingResolution(0.001);

        spaceK->setup();
        si_vec.push_back(siK);
    }
    OMPL_INFORM("Add Original Chain with %d links.", numLinks);
    si_vec.push_back(si);

    typedef og::MultiQuotient<og::QRRT> MultiQuotient;
    auto planner = std::make_shared<MultiQuotient>(si_vec);
    planner->setProblemDefinition(pdef);
    std::string qName = "QuotientSpaceRRT["+std::to_string(si_vec.size())+"lvl]";
    planner->setName(qName);
    return planner;
}

int main()
{
    for(uint k = 2; k < ndim; k++){
        double range = edgeWidth * 0.5;
        auto space(std::make_shared<ompl::base::RealVectorStateSpace>(ndim));
        ompl::base::RealVectorBounds bounds(ndim);
        ompl::geometric::SimpleSetup ss(space);
        ompl::base::ScopedState<> start(space), goal(space);

        bounds.setLow(0.);
        bounds.setHigh(1.);
        space->setBounds(bounds);
        ss.setStateValidityChecker(std::make_shared<HyperCubeValidityChecker>(ss.getSpaceInformation(), ndim));
        ss.getSpaceInformation()->setStateValidityCheckingResolution(0.001);
        for(unsigned int i = 0; i < ndim; ++i)
        {
            start[i] = 0.;
            goal[i] = 1.;
        }
        ss.setStartAndGoalStates(start, goal);

        ompl::tools::Benchmark::Request request(runtime_limit, memory_limit, run_count);
        ompl::tools::Benchmark b(ss, "HyperCube");
        b.addExperimentParameter("num_dims", "INTEGER", std::to_string(ndim));

        ob::SpaceInformationPtr si = ss.getSpaceInformation();

        //Note: 30 Planner + QRRT
        addPlanner(b, std::make_shared<og::RRT>(si), range);
        addPlanner(b, std::make_shared<og::RRTConnect>(si), range);
        addPlanner(b, std::make_shared<og::RRTsharp>(si), range);
        addPlanner(b, std::make_shared<og::RRTstar>(si), range);
        addPlanner(b, std::make_shared<og::RRTXstatic>(si), range);
        addPlanner(b, std::make_shared<og::LazyRRT>(si), range);
        addPlanner(b, std::make_shared<og::InformedRRTstar>(si), range);
        addPlanner(b, std::make_shared<og::TRRT>(si), range);
        addPlanner(b, std::make_shared<og::BiTRRT>(si), range);
        addPlanner(b, std::make_shared<og::LBTRRT>(si), range);
        addPlanner(b, std::make_shared<og::SORRTstar>(si), range);
        addPlanner(b, std::make_shared<og::PRM>(si), range);
        addPlanner(b, std::make_shared<og::PRMstar>(si), range);
        addPlanner(b, std::make_shared<og::LazyPRM>(si), range);
        addPlanner(b, std::make_shared<og::LazyPRMstar>(si), range);
        addPlanner(b, std::make_shared<og::SPARS>(si), range);
        addPlanner(b, std::make_shared<og::SPARStwo>(si), range);
        addPlanner(b, std::make_shared<og::KPIECE1>(si), range);
        addPlanner(b, std::make_shared<og::BKPIECE1>(si), range);
        addPlanner(b, std::make_shared<og::LBKPIECE1>(si), range);
        addPlanner(b, std::make_shared<og::FMT>(si), range);
        addPlanner(b, std::make_shared<og::BFMT>(si), range);
        addPlanner(b, std::make_shared<og::EST>(si), range);
        addPlanner(b, std::make_shared<og::BiEST>(si), range);
        addPlanner(b, std::make_shared<og::ProjEST>(si), range);
        addPlanner(b, std::make_shared<og::SBL>(si), range);
        addPlanner(b, std::make_shared<og::STRIDE>(si), range);
        addPlanner(b, std::make_shared<og::SST>(si), range);
        addPlanner(b, std::make_shared<og::PDST>(si), range);
        addPlanner(b, std::make_shared<og::BITstar>(si), range);

        ob::PlannerPtr quotientSpacePlanner = 
          GetQRRT(ss.getSpaceInformation(), ss.getProblemDefinition(), ndim);
        addPlanner(b, quotientSpacePlanner, range);

        b.benchmark(request);
        b.saveResultsToFile(boost::str(boost::format("hypercube_%i.log") % ndim).c_str());

        PrintBenchmarkResults(b);

    }
    return 0;
}

