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
#include "../KinematicChain.h"
#include <ompl/base/SpaceInformation.h>
#include <ompl/tools/benchmark/Benchmark.h>
#include <boost/range/irange.hpp>
#include <boost/range/algorithm_ext/push_back.hpp>

const unsigned int numLinks = 15;
const double runtime_limit = 5;
const double memory_limit = 4096;
const int run_count = 2;

const double linkLength = 1.0 / numLinks;

using namespace ompl::base;
std::vector<Environment> envs;

Environment createCustomHornEnvironment(unsigned int d)
{
    double narrowPassageWidth = log((double)d) / (double)d;
    return createHornEnvironment(d, narrowPassageWidth);
}

PlannerPtr GetQRRT(std::vector<int> sequenceLinks, SpaceInformationPtr si)
{
    std::vector<SpaceInformationPtr> si_vec;

    for (unsigned int k = 0; k < sequenceLinks.size(); k++)
    {
        auto links = sequenceLinks.at(k);
        assert(links < numLinks);

        OMPL_INFORM("Create MultiLevel Chain with %d links.", links);
        auto spaceK(std::make_shared<KinematicChainSpace>(links, linkLength, &envs.at(links)));

        auto siK = std::make_shared<SpaceInformation>(spaceK);
        siK->setStateValidityChecker(std::make_shared<KinematicChainValidityChecker>(siK));
        spaceK->setup();
        si_vec.push_back(siK);
    }

    OMPL_INFORM("Add Original Chain with %d links.", numLinks);
    si_vec.push_back(si);

    auto planner = std::make_shared<ompl::multilevel::QRRT>(si_vec);

    std::string qName = "QRRT[";
    for (unsigned int k = 0; k < sequenceLinks.size(); k++)
    {
        int links = sequenceLinks.at(k);
        qName += std::to_string(links) + ",";
    }
    qName += std::to_string(numLinks);
    qName += "]";
    planner->setName(qName);
    return planner;
}

std::vector<std::vector<int>> getAdmissibleProjections(int dim)
{
    std::vector<std::vector<int>> projections;
    std::vector<int> discrete;
    boost::push_back(discrete, boost::irange(2, dim + 1));

    std::vector<int> twoStep;
    boost::push_back(twoStep, boost::irange(2, dim + 1, 2));

    if (twoStep.back() != dim)
        twoStep.push_back(dim);

    projections.push_back(twoStep);
    projections.push_back(discrete);

    auto last = std::unique(projections.begin(), projections.end());
    projections.erase(last, projections.end());

    std::cout << "Projections for dim " << dim << std::endl;
    for (unsigned int k = 0; k < projections.size(); k++)
    {
        std::vector<int> pk = projections.at(k);
        std::cout << k << ": ";
        for (unsigned int j = 0; j < pk.size(); j++)
        {
            std::cout << pk.at(j) << (j < pk.size() - 1 ? "," : "");
        }
        std::cout << std::endl;
    }

    return projections;
}

int main()
{
    Environment env = createCustomHornEnvironment(numLinks);
    OMPL_INFORM("Original Chain has %d links", numLinks);
    for (unsigned int k = 0; k < numLinks; k++)
    {
        envs.push_back(createCustomHornEnvironment((k > 0 ? k : 1)));
    }

    auto chain(std::make_shared<KinematicChainSpace>(numLinks, linkLength, &env));
    ompl::geometric::SimpleSetup ss(chain);

    ss.setStateValidityChecker(std::make_shared<KinematicChainValidityChecker>(ss.getSpaceInformation()));

    ompl::base::ScopedState<> start(chain), goal(chain);
    std::vector<double> startVec(numLinks, boost::math::constants::pi<double>() / (double)numLinks);
    std::vector<double> goalVec(numLinks, 0);
    startVec[0] = 0.;
    goalVec[0] = boost::math::constants::pi<double>() - .001;
    chain->setup();
    chain->copyFromReals(start.get(), startVec);
    chain->copyFromReals(goal.get(), goalVec);
    ss.setStartAndGoalStates(start, goal);

    ompl::tools::Benchmark benchmark(ss, "KinematicChain");
    benchmark.addExperimentParameter("num_links", "INTEGER", std::to_string(numLinks));

    //############################################################################
    // Compare QRRT with different QuotientSpace sequences to other OMPL planner
    //############################################################################
    SpaceInformationPtr si = ss.getSpaceInformation();
    ProblemDefinitionPtr pdef = ss.getProblemDefinition();

    addPlanner(benchmark, std::make_shared<ompl::geometric::STRIDE>(si));
    addPlanner(benchmark, std::make_shared<ompl::geometric::KPIECE1>(si));
    addPlanner(benchmark, std::make_shared<ompl::geometric::EST>(si));
    addPlanner(benchmark, std::make_shared<ompl::geometric::PRM>(si));

    std::vector<std::vector<int>> admissibleProjections = getAdmissibleProjections(numLinks - 1);
    for (unsigned int k = 0; k < admissibleProjections.size(); k++)
    {
        std::vector<int> proj = admissibleProjections.at(k);
        PlannerPtr plannerK = GetQRRT(proj, si);
        addPlanner(benchmark, plannerK);
    }

    printEstimatedTimeToCompletion(numberPlanners, run_count, runtime_limit);

    ompl::tools::Benchmark::Request request;
    request.maxTime = runtime_limit;
    request.maxMem = memory_limit;
    request.runCount = run_count;
    request.simplify = false;
    request.displayProgress = false;
    numberRuns = numberPlanners * run_count;

    benchmark.setPostRunEvent(std::bind(&PostRunEvent, std::placeholders::_1, std::placeholders::_2));

    benchmark.benchmark(request);
    benchmark.saveResultsToFile(boost::str(boost::format("kinematic_%i.log") % numLinks).c_str());

    printBenchmarkResults(benchmark);
    return 0;
}
