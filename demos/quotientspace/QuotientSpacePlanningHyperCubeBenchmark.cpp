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

const double edgeWidth = 0.1;
const double runtime_limit = 10;
const double memory_limit = 4096 * 4096;
const int run_count = 10;
unsigned int curDim = 8;
int numberPlanners = 0;

#include "QuotientSpacePlanningCommon.h"
#include <ompl/base/spaces/RealVectorStateSpace.h>

#include <ompl/geometric/planners/informedtrees/BITstar.h>
#include <ompl/geometric/planners/est/BiEST.h>
#include <ompl/geometric/planners/est/EST.h>
#include <ompl/geometric/planners/est/ProjEST.h>
#include <ompl/geometric/planners/fmt/BFMT.h>
#include <ompl/geometric/planners/fmt/FMT.h>
#include <ompl/geometric/planners/kpiece/BKPIECE1.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/kpiece/LBKPIECE1.h>
#include <ompl/geometric/planners/pdst/PDST.h>
// #include <ompl/geometric/planners/prm/LazyPRM.h> //TODO: segfault?
#include <ompl/geometric/planners/prm/LazyPRMstar.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/prm/SPARS.h>
#include <ompl/geometric/planners/prm/SPARStwo.h>
#include <ompl/geometric/planners/quotientspace/QRRT.h>
#include <ompl/geometric/planners/rrt/BiTRRT.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/rrt/LazyRRT.h>
#include <ompl/geometric/planners/rrt/LBTRRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTsharp.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/RRTXstatic.h>
#include <ompl/geometric/planners/rrt/SORRTstar.h>
#include <ompl/geometric/planners/rrt/TRRT.h>
#include <ompl/geometric/planners/sbl/pSBL.h>
#include <ompl/geometric/planners/sbl/SBL.h>
#include <ompl/geometric/planners/sst/SST.h>
#include <ompl/geometric/planners/stride/STRIDE.h>

#include <ompl/tools/benchmark/Benchmark.h>
#include <ompl/util/String.h>

#include <boost/math/constants/constants.hpp>
#include <boost/range/irange.hpp>
#include <boost/range/algorithm_ext/push_back.hpp>
#include <boost/format.hpp>
#include <fstream>

// Only states near some edges of a hypercube are valid. The valid edges form a
// narrow passage from (0,...,0) to (1,...,1). A state s is valid if there exists
// a k s.t. (a) 0<=s[k]<=1, (b) for all i<k s[i]<=edgeWidth, and (c) for all i>k
// s[i]>=1-edgewidth.
class HyperCubeValidityChecker : public ob::StateValidityChecker
{
public:
    HyperCubeValidityChecker(const ob::SpaceInformationPtr &si, int dimension)
      : ob::StateValidityChecker(si), dimension_(dimension)
    {
        si->setStateValidityCheckingResolution(0.001);
    }

    bool isValid(const ob::State *state) const override
    {
        const auto *s = static_cast<const ob::RealVectorStateSpace::StateType *>(state);
        bool foundMaxDim = false;

        for (int i = dimension_ - 1; i >= 0; i--)
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
    int dimension_;
};

static unsigned int numberRuns{0};

void PostRunEvent(const ob::PlannerPtr &planner, ot::Benchmark::RunProperties &run)
{
    static unsigned int pid = 0;

    ob::SpaceInformationPtr si = planner->getSpaceInformation();
    ob::ProblemDefinitionPtr pdef = planner->getProblemDefinition();

    unsigned int states = boost::lexical_cast<int>(run["graph states INTEGER"]);
    double time = boost::lexical_cast<double>(run["time REAL"]);
    double memory = boost::lexical_cast<double>(run["memory REAL"]);

    bool solved = (time < runtime_limit);

    std::cout << "Run " << pid << "/" << numberRuns << " [" << planner->getName() << "] "
              << (solved ? "solved" : "FAILED") << "(time: " << time << ", states: " << states << ", memory: " << memory
              << ")" << std::endl;
    std::cout << std::string(80, '-') << std::endl;
    pid++;
}

// Note: Number of all simplifications is
// unsigned int numberSimplifications = std::pow(2, curDim - 1);
// But here we will only create three simplifications, the trivial one, the
// discrete one and a two-step simplifications, which we found worked well in
// this experiment. You can experiment with finding better simplifications.
// std::cout << "dimension: " << curDim << " simplifications:" << numberSimplifications << std::endl;

std::vector<std::vector<int>> getHypercubeAdmissibleProjections(int dim)
{
    std::vector<std::vector<int>> projections;

    // trivial: just configuration space
    // discrete: use all admissible projections
    std::vector<int> trivial{dim};
    std::vector<int> discrete;
    boost::push_back(discrete, boost::irange(2, dim + 1));

    std::vector<int> twoStep;
    boost::push_back(twoStep, boost::irange(2, dim + 1, 2));
    if (twoStep.back() != dim)
        twoStep.push_back(dim);

    projections.push_back(trivial);
    projections.push_back(discrete);
    projections.push_back(twoStep);
    auto last = std::unique(projections.begin(), projections.end());
    projections.erase(last, projections.end());

    // std::cout << "Projections for dim " << dim << std::endl;
    // for(unsigned int k = 0; k < projections.size(); k++){
    //     std::vector<int> pk = projections.at(k);
    //     std::cout << k << ": ";
    //     for(unsigned int j = 0; j < pk.size(); j++){
    //       std::cout << pk.at(j) << (j<pk.size()-1?",":"");
    //     }
    //     std::cout << std::endl;
    // }

    return projections;
}

void addPlanner(ompl::tools::Benchmark &benchmark, const ompl::base::PlannerPtr &planner, double range)
{
    ompl::base::ParamSet &params = planner->params();
    if (params.hasParam(std::string("range")))
        params.setParam(std::string("range"), ompl::toString(range));
    benchmark.addPlanner(planner);
    numberPlanners++;
}

ob::PlannerPtr GetQRRT(std::vector<int> sequenceLinks, ob::SpaceInformationPtr si)
{
    // ompl::msg::setLogLevel(ompl::msg::LOG_DEV2);
    std::vector<ob::SpaceInformationPtr> si_vec;

    for (unsigned int k = 0; k < sequenceLinks.size() - 1; k++)
    {
        int links = sequenceLinks.at(k);

        auto spaceK(std::make_shared<ompl::base::RealVectorStateSpace>(links));
        ompl::base::RealVectorBounds bounds(links);
        bounds.setLow(0.);
        bounds.setHigh(1.);
        spaceK->setBounds(bounds);

        auto siK = std::make_shared<ob::SpaceInformation>(spaceK);
        siK->setStateValidityChecker(std::make_shared<HyperCubeValidityChecker>(siK, links));

        spaceK->setup();
        si_vec.push_back(siK);
    }
    si_vec.push_back(si);

    auto planner = std::make_shared<og::QRRT>(si_vec);
    std::string qName = "QuotientSpaceRRT[";
    for (unsigned int k = 0; k < sequenceLinks.size() - 1; k++)
    {
        int links = sequenceLinks.at(k);
        qName += std::to_string(links) + ",";
    }
    qName += std::to_string(si->getStateDimension());
    qName += "]";
    std::cout << qName << std::endl;
    planner->setName(qName);
    return planner;
}

int main(int argc, char **argv)
{
    if (argc > 1)
    {
        curDim = std::atoi(argv[1]);
    }

    numberPlanners = 0;
    double range = edgeWidth * 0.5;
    auto space(std::make_shared<ompl::base::RealVectorStateSpace>(curDim));
    ompl::base::RealVectorBounds bounds(curDim);
    ompl::geometric::SimpleSetup ss(space);
    ob::SpaceInformationPtr si = ss.getSpaceInformation();
    ob::ProblemDefinitionPtr pdef = ss.getProblemDefinition();
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

    ot::Benchmark benchmark(ss, "HyperCube");
    benchmark.addExperimentParameter("num_dims", "INTEGER", std::to_string(curDim));

    //############################################################################
    // Load All Planner
    //############################################################################
    std::vector<std::vector<int>> admissibleProjections = getHypercubeAdmissibleProjections(curDim);
    for (unsigned int k = 0; k < admissibleProjections.size(); k++)
    {
        std::vector<int> proj = admissibleProjections.at(k);
        ob::PlannerPtr quotientSpacePlannerK = GetQRRT(proj, si);
        addPlanner(benchmark, quotientSpacePlannerK, range);
    }
    addPlanner(benchmark, std::make_shared<og::BITstar>(si), range);
    addPlanner(benchmark, std::make_shared<og::EST>(si), range);
    addPlanner(benchmark, std::make_shared<og::BiEST>(si), range);
    addPlanner(benchmark, std::make_shared<og::ProjEST>(si), range);
    addPlanner(benchmark, std::make_shared<og::FMT>(si), range);
    addPlanner(benchmark, std::make_shared<og::BFMT>(si), range);
    addPlanner(benchmark, std::make_shared<og::KPIECE1>(si), range);
    addPlanner(benchmark, std::make_shared<og::BKPIECE1>(si), range);
    addPlanner(benchmark, std::make_shared<og::LBKPIECE1>(si), range);
    addPlanner(benchmark, std::make_shared<og::PDST>(si), range);
    addPlanner(benchmark, std::make_shared<og::PRM>(si), range);
    addPlanner(benchmark, std::make_shared<og::PRMstar>(si), range);
    addPlanner(benchmark, std::make_shared<og::LazyPRMstar>(si), range);
    addPlanner(benchmark, std::make_shared<og::SPARS>(si), range);
    addPlanner(benchmark, std::make_shared<og::SPARStwo>(si), range);
    addPlanner(benchmark, std::make_shared<og::RRT>(si), range);
    addPlanner(benchmark, std::make_shared<og::RRTConnect>(si), range);
    addPlanner(benchmark, std::make_shared<og::RRTsharp>(si), range);
    addPlanner(benchmark, std::make_shared<og::RRTstar>(si), range);
    addPlanner(benchmark, std::make_shared<og::RRTXstatic>(si), range);
    addPlanner(benchmark, std::make_shared<og::LazyRRT>(si), range);
    addPlanner(benchmark, std::make_shared<og::InformedRRTstar>(si), range);
    addPlanner(benchmark, std::make_shared<og::TRRT>(si), range);
    addPlanner(benchmark, std::make_shared<og::BiTRRT>(si), range);
    addPlanner(benchmark, std::make_shared<og::LBTRRT>(si), range);
    addPlanner(benchmark, std::make_shared<og::SORRTstar>(si), range);
    addPlanner(benchmark, std::make_shared<og::SBL>(si), range);
    addPlanner(benchmark, std::make_shared<og::SST>(si), range);
    addPlanner(benchmark, std::make_shared<og::STRIDE>(si), range);
    //############################################################################

    printEstimatedTimeToCompletion(numberPlanners, run_count, runtime_limit);

    ot::Benchmark::Request request;
    request.maxTime = runtime_limit;
    request.maxMem = memory_limit;
    request.runCount = run_count;
    request.simplify = false;
    request.displayProgress = false;
    numberRuns = numberPlanners * run_count;

    benchmark.setPostRunEvent(std::bind(&PostRunEvent, std::placeholders::_1, std::placeholders::_2));
    benchmark.benchmark(request);
    benchmark.saveResultsToFile(boost::str(boost::format("hypercube_%i.log") % curDim).c_str());

    printBenchmarkResults(benchmark);

    return 0;
}
