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

#include "KinematicChain.h"
#include "QuotientSpacePlanningCommon.h"
#include <ompl/geometric/planners/quotientspace/QRRT.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/tools/benchmark/Benchmark.h>
#include <boost/range/irange.hpp>
#include <boost/range/algorithm_ext/push_back.hpp>

const unsigned int numLinks = 20;
const double runtime_limit = 3;
const double memory_limit = 4096;
const int run_count = 1;

const double linkLength = 1.0 / numLinks;

namespace ot = ompl::tools;
namespace og = ompl::geometric;
std::vector<Environment> envs;

Environment createCustomHornEnvironment(unsigned int d)
{
    double narrowPassageWidth = sqrtf((double)d) / (double)d;
    return createHornEnvironment(d, narrowPassageWidth);
}

ob::PlannerPtr GetQRRT(std::vector<int> sequenceLinks, ob::SpaceInformationPtr si)
{
    // ompl::msg::setLogLevel(ompl::msg::LOG_DEV2);
    std::vector<ob::SpaceInformationPtr> si_vec;

    for (unsigned int k = 0; k < sequenceLinks.size(); k++)
    {
        int links = sequenceLinks.at(k);
        assert(links < numLinks);

        OMPL_INFORM("Create QuotientSpace Chain with %d links.", links);
        auto spaceK(std::make_shared<KinematicChainSpace>(links, linkLength, &envs.at(links)));

        auto siK = std::make_shared<ob::SpaceInformation>(spaceK);
        siK->setStateValidityChecker(std::make_shared<KinematicChainValidityChecker>(siK));
        spaceK->setup();
        si_vec.push_back(siK);
    }

    OMPL_INFORM("Add Original Chain with %d links.", numLinks);
    si_vec.push_back(si);

    auto planner = std::make_shared<og::QRRT>(si_vec);

    std::string qName = "QuotientSpaceRRT[";
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
    projections.push_back(std::vector<int>{});
    projections.push_back(std::vector<int>{2});
    projections.push_back(std::vector<int>{3});

    unsigned int K = ceil(dim*0.5);
    for(uint k = 0; k < K; k++){
      std::vector<int> kStep;
      if(k>0) boost::push_back(kStep, boost::irange(2, dim, k));
      projections.push_back(kStep);
    }
    for(uint k = 2; k < dim; k++){
      std::vector<int> kStep{k};
      projections.push_back(kStep);
      for(uint j = k+1; j < dim; j++){
        std::vector<int> kjStep{k,j};
        projections.push_back(kjStep);
      }
    }

    auto last = std::unique(projections.begin(), projections.end());
    projections.erase(last, projections.end());

    std::cout << "Projections for dim " << dim << std::endl;
    for(unsigned int k = 0; k < projections.size(); k++){
        std::vector<int> pk = projections.at(k);
        std::cout << k << ": ";
        for(unsigned int j = 0; j < pk.size(); j++){
          std::cout << pk.at(j) << (j<pk.size()-1?",":"");
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
        envs.push_back(createCustomHornEnvironment((k>0?k:1)));
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

    ompl::tools::Benchmark::Request request(runtime_limit, memory_limit, run_count, 0.5);
    ompl::tools::Benchmark benchmark(ss, "KinematicChain");
    benchmark.addExperimentParameter("num_links", "INTEGER", std::to_string(numLinks));

    //############################################################################
    // Compare QRRT with different QuotientSpace sequences to other OMPL planner
    //############################################################################
    ob::SpaceInformationPtr si = ss.getSpaceInformation();
    ob::ProblemDefinitionPtr pdef = ss.getProblemDefinition();
    benchmark.addPlanner(std::make_shared<og::STRIDE>(si));
    benchmark.addPlanner(std::make_shared<og::EST>(si));
    benchmark.addPlanner(std::make_shared<og::KPIECE1>(si));
    // benchmark.addPlanner(std::make_shared<og::RRT>(si));
    // benchmark.addPlanner(std::make_shared<og::PRM>(si));

    std::vector<std::vector<int>> admissibleProjections = getAdmissibleProjections(numLinks-1);
    for (unsigned int k = 0; k < admissibleProjections.size(); k++)
    {
        std::vector<int> proj = admissibleProjections.at(k);
        ob::PlannerPtr quotientSpacePlannerK = GetQRRT(proj, si);
        benchmark.addPlanner(quotientSpacePlannerK);
    }

    benchmark.benchmark(request);
    benchmark.saveResultsToFile(boost::str(boost::format("kinematic_%i.log") % numLinks).c_str());

    printBenchmarkResults(benchmark);
    return 0;
}
