#include "KinematicChain.h"
#include "QuotientSpacePlanningCommon.h"
#include <ompl/geometric/planners/quotientspace/MultiQuotient.h>
#include <ompl/geometric/planners/quotientspace/QRRT.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/tools/benchmark/Benchmark.h>

const unsigned int numLinks = 15;
const double linkLength = 1.0 / numLinks;
const double narrowPassageWidth = log((double)numLinks) / (double)numLinks;

namespace ot = ompl::tools;
std::vector<Environment> envs;

ob::PlannerPtr GetQRRT(
    std::vector<int> sequenceLinks, 
    ob::SpaceInformationPtr si, 
    ob::ProblemDefinitionPtr pdef, 
    std::vector<double> start, 
    std::vector<double> goal)
{
    // ompl::msg::setLogLevel(ompl::msg::LOG_DEV2);
    std::vector<ob::SpaceInformationPtr> si_vec;
    std::vector<ob::ProblemDefinitionPtr> pdef_vec;

    for(unsigned k = 0; k < sequenceLinks.size(); k++)
    {
        int links = sequenceLinks.at(k);
        assert(links<numLinks);

        OMPL_INFORM("Create QuotientSpace Chain with %d links.", links);
        auto spaceK(std::make_shared<KinematicChainSpace>(links, linkLength, &envs.at(links)));

        auto siK = std::make_shared<ob::SpaceInformation>(spaceK);
        siK->setStateValidityChecker(std::make_shared<KinematicChainValidityChecker>(siK));
        ob::ProblemDefinitionPtr pdefk = std::make_shared<ob::ProblemDefinition>(siK);

        unsigned clippedDofs = numLinks - links;
        std::vector<double> startVecK(start.begin(), start.end()-clippedDofs);
        std::vector<double> goalVecK(goal.begin(), goal.end()-clippedDofs);

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

    std::string qName = "QuotientSpaceRRT[";
    for(unsigned k = 0; k < sequenceLinks.size(); k++)
    {
        int links = sequenceLinks.at(k);
        qName+=std::to_string(links)+",";
    }
    qName+=std::to_string(numLinks);
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
    for(unsigned k = 1; k < numLinks; k++){
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
    //Compare QRRT with different QuotientSpace sequences to other OMPL planner
    //############################################################################
    b.addPlanner(std::make_shared<ompl::geometric::STRIDE>(ss.getSpaceInformation()));
    b.addPlanner(std::make_shared<ompl::geometric::EST>(ss.getSpaceInformation()));
    b.addPlanner(std::make_shared<ompl::geometric::KPIECE1>(ss.getSpaceInformation()));
    b.addPlanner(std::make_shared<ompl::geometric::RRT>(ss.getSpaceInformation()));
    b.addPlanner(std::make_shared<ompl::geometric::PRM>(ss.getSpaceInformation()));

    b.addPlanner( GetQRRT(std::vector<int>{3}, 
          ss.getSpaceInformation(), 
          ss.getProblemDefinition(), 
          startVec, 
          goalVec));
    b.addPlanner( GetQRRT(std::vector<int>{2}, 
          ss.getSpaceInformation(), 
          ss.getProblemDefinition(), 
          startVec, 
          goalVec));
    b.addPlanner( GetQRRT(std::vector<int>{3,5,9}, 
          ss.getSpaceInformation(), 
          ss.getProblemDefinition(), 
          startVec, 
          goalVec));
    b.addPlanner( GetQRRT(std::vector<int>{3,11}, 
          ss.getSpaceInformation(), 
          ss.getProblemDefinition(), 
          startVec, 
          goalVec));
    b.addPlanner( GetQRRT(std::vector<int>{10}, 
          ss.getSpaceInformation(), 
          ss.getProblemDefinition(), 
          startVec, 
          goalVec));
    b.addPlanner( GetQRRT(std::vector<int>{12}, 
          ss.getSpaceInformation(), 
          ss.getProblemDefinition(), 
          startVec, 
          goalVec));
    b.addPlanner( GetQRRT(std::vector<int>{8,13}, 
          ss.getSpaceInformation(), 
          ss.getProblemDefinition(), 
          startVec, 
          goalVec));
    b.addPlanner( GetQRRT(std::vector<int>{}, 
          ss.getSpaceInformation(), 
          ss.getProblemDefinition(), 
          startVec, 
          goalVec));
    b.addPlanner( GetQRRT(std::vector<int>{2,3,4,5,6,7,8,9,10,11,12,13,14}, 
          ss.getSpaceInformation(), 
          ss.getProblemDefinition(), 
          startVec, 
          goalVec));

    b.benchmark(request);
    b.saveResultsToFile(boost::str(boost::format("kinematic_%i.log") % numLinks).c_str());

    PrintBenchmarkResults(b);
    return 0;
}
