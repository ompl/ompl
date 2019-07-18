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
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/est/EST.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/stride/STRIDE.h>
#include <ompl/tools/benchmark/Benchmark.h>
#include <ompl/util/String.h>

#include <ompl/geometric/planners/quotientspace/MultiQuotient.h>
#include <ompl/geometric/planners/quotientspace/QRRT.h>

#include <boost/math/constants/constants.hpp>
#include <boost/format.hpp>
#include <fstream>

const double edgeWidth = 0.1;

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
    std::vector<ob::ProblemDefinitionPtr> pdef_vec;

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

        ob::ProblemDefinitionPtr pdefk = std::make_shared<ob::ProblemDefinition>(siK);
        std::vector<double> startVecK(k, 0);
        std::vector<double> goalVecK(k, 1);
        ompl::base::ScopedState<> startk(spaceK), goalk(spaceK);
        spaceK->setup();
        spaceK->copyFromReals(startk.get(), startVecK);
        spaceK->copyFromReals(goalk.get(), goalVecK);
        pdefk->setStartAndGoalStates(startk, goalk);

        si_vec.push_back(siK);
        pdef_vec.push_back(pdefk);
    }
    OMPL_INFORM("Add Original Chain with %d links.", numLinks);
    si_vec.push_back(si);
    pdef_vec.push_back(pdef);

    typedef og::MultiQuotient<og::QRRT> MultiQuotient;
    auto planner = std::make_shared<MultiQuotient>(si_vec);
    planner->setProblemDefinition(pdef_vec);
    std::string qName = "QuotientSpaceRRT["+std::to_string(si_vec.size())+"lvl]";
    planner->setName(qName);
    return planner;
}

int main(int argc, char **argv)
{
    const unsigned ndim = 8;

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

    double runtime_limit = 10, memory_limit = 4096;
    int run_count = 5;
    ompl::tools::Benchmark::Request request(runtime_limit, memory_limit, run_count);
    ompl::tools::Benchmark b(ss, "HyperCube");
    b.addExperimentParameter("num_dims", "INTEGER", std::to_string(ndim));

    addPlanner(b, std::make_shared<ompl::geometric::STRIDE>(ss.getSpaceInformation()), range);
    addPlanner(b, std::make_shared<ompl::geometric::EST>(ss.getSpaceInformation()), range);
    addPlanner(b, std::make_shared<ompl::geometric::KPIECE1>(ss.getSpaceInformation()), range);
    addPlanner(b, std::make_shared<ompl::geometric::RRT>(ss.getSpaceInformation()), range);
    addPlanner(b, std::make_shared<ompl::geometric::PRM>(ss.getSpaceInformation()), range);

    ob::PlannerPtr quotientSpacePlanner = 
      GetQRRT(ss.getSpaceInformation(), ss.getProblemDefinition(), ndim);
    addPlanner(b, quotientSpacePlanner, range);

    b.benchmark(request);
    b.saveResultsToFile(boost::str(boost::format("hypercube_%i.log") % ndim).c_str());

    PrintBenchmarkResults(b);

    return 0;
}

