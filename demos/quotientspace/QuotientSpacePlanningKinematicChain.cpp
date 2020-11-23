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

#include "../KinematicChain.h"
#include "QuotientSpacePlanningCommon.h"
#include <ompl/geometric/planners/quotientspace/QRRT.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/tools/benchmark/Benchmark.h>

const unsigned int numLinks = 15;
const double linkLength = 1.0 / numLinks;
const double narrowPassageWidth = log((double)numLinks) / (double)numLinks;

namespace ot = ompl::tools;
std::vector<Environment> envs;

ob::PlannerPtr GetQRRT(std::vector<unsigned int> sequenceLinks, ob::SpaceInformationPtr si,
                       ob::ProblemDefinitionPtr pdef)
{
    // ompl::msg::setLogLevel(ompl::msg::LOG_DEV2);
    std::vector<ob::SpaceInformationPtr> si_vec;

    for (unsigned int k = 0; k < sequenceLinks.size(); k++)
    {
        auto links = sequenceLinks.at(k);
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
    planner->setProblemDefinition(pdef);

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

int main()
{
    Environment env = createHornEnvironment(numLinks, narrowPassageWidth);
    OMPL_INFORM("Original Chain has %d links", numLinks);
    OMPL_INFORM("Creating Horn Environment with width %f.", narrowPassageWidth);
    envs.push_back(createHornEnvironment(1, narrowPassageWidth));
    for (unsigned int k = 1; k < numLinks; k++)
    {
        envs.push_back(createHornEnvironment(k, narrowPassageWidth));
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

    double runtime_limit = 10, memory_limit = 1024;
    int run_count = 10;
    ompl::tools::Benchmark::Request request(runtime_limit, memory_limit, run_count, 0.5);
    ompl::tools::Benchmark b(ss, "KinematicChain");
    b.addExperimentParameter("num_links", "INTEGER", std::to_string(numLinks));

    //############################################################################
    // Compare QRRT with different QuotientSpace sequences to other OMPL planner
    //############################################################################
    b.addPlanner(std::make_shared<ompl::geometric::STRIDE>(ss.getSpaceInformation()));
    b.addPlanner(std::make_shared<ompl::geometric::EST>(ss.getSpaceInformation()));
    b.addPlanner(std::make_shared<ompl::geometric::KPIECE1>(ss.getSpaceInformation()));
    b.addPlanner(std::make_shared<ompl::geometric::RRT>(ss.getSpaceInformation()));
    b.addPlanner(std::make_shared<ompl::geometric::PRM>(ss.getSpaceInformation()));

    b.addPlanner(GetQRRT(std::vector<unsigned int>{3}, ss.getSpaceInformation(), ss.getProblemDefinition()));
    b.addPlanner(GetQRRT(std::vector<unsigned int>{2}, ss.getSpaceInformation(), ss.getProblemDefinition()));
    b.addPlanner(GetQRRT(std::vector<unsigned int>{3, 5, 9}, ss.getSpaceInformation(), ss.getProblemDefinition()));
    b.addPlanner(GetQRRT(std::vector<unsigned int>{3, 11}, ss.getSpaceInformation(), ss.getProblemDefinition()));
    b.addPlanner(GetQRRT(std::vector<unsigned int>{10}, ss.getSpaceInformation(), ss.getProblemDefinition()));
    b.addPlanner(GetQRRT(std::vector<unsigned int>{12}, ss.getSpaceInformation(), ss.getProblemDefinition()));
    b.addPlanner(GetQRRT(std::vector<unsigned int>{8, 13}, ss.getSpaceInformation(), ss.getProblemDefinition()));
    b.addPlanner(GetQRRT(std::vector<unsigned int>{}, ss.getSpaceInformation(), ss.getProblemDefinition()));
    b.addPlanner(GetQRRT(std::vector<unsigned int>{2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14},
                         ss.getSpaceInformation(), ss.getProblemDefinition()));

    b.benchmark(request);
    b.saveResultsToFile(boost::str(boost::format("kinematic_%i.log") % numLinks).c_str());

    printBenchmarkResults(b);
    return 0;
}
