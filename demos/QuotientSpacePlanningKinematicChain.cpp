#include "KinematicChain.h"
#include <ompl/geometric/planners/quotientspace/MultiQuotient.h>
#include <ompl/geometric/planners/quotientspace/QRRT.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/tools/benchmark/Benchmark.h>

const unsigned int numLinks = 15;
const double linkLength = 1.0 / numLinks;
const double narrowPassageWidth = 0.2;

namespace ot = ompl::tools;
std::vector<Environment> envs;

ob::PlannerPtr GetQRRT(
    ob::SpaceInformationPtr si, 
    ob::ProblemDefinitionPtr pdef, 
    std::vector<double> start, 
    std::vector<double> goal, 
    uint numLinks, 
    Environment &env)
{
    // ompl::msg::setLogLevel(ompl::msg::LOG_DEV2);
    std::vector<ob::SpaceInformationPtr> si_vec;
    std::vector<ob::ProblemDefinitionPtr> pdef_vec;

    for(uint k = 3; k < numLinks; k+=2)
    {
        OMPL_INFORM("Create QuotientSpace Chain with %d links.", k);
        Environment envk = createHornEnvironment(k, narrowPassageWidth);
        auto chainK(std::make_shared<KinematicChainSpace>(k, linkLength, &envk));
        envs.push_back(envk);

        auto siK = std::make_shared<ob::SpaceInformation>(chainK);
        siK->setStateValidityChecker(std::make_shared<KinematicChainValidityChecker>(siK));
        ob::ProblemDefinitionPtr pdefk = std::make_shared<ob::ProblemDefinition>(siK);

        uint clippedDofs = numLinks - k;
        std::vector<double> startVecK(start.begin(), start.end()-clippedDofs);
        std::vector<double> goalVecK(goal.begin(), goal.end()-clippedDofs);

        ompl::base::ScopedState<> startk(chainK), goalk(chainK);
        chainK->setup();
        chainK->copyFromReals(startk.get(), startVecK);
        chainK->copyFromReals(goalk.get(), goalVecK);
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
    planner->setName("QuotientSpaceRRT");
    return planner;
}

int main()
{
    Environment env = createHornEnvironment(numLinks, narrowPassageWidth);
    OMPL_INFORM("Original Chain has %d links", numLinks);
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

    double runtime_limit = 60, memory_limit = 1024;
    int run_count = 10;
    ompl::tools::Benchmark::Request request(runtime_limit, memory_limit, run_count, 0.5);
    ompl::tools::Benchmark b(ss, "KinematicChain");
    b.addExperimentParameter("num_links", "INTEGER", std::to_string(numLinks));

    // b.addPlanner(std::make_shared<ompl::geometric::STRIDE>(ss.getSpaceInformation()));
    // b.addPlanner(std::make_shared<ompl::geometric::EST>(ss.getSpaceInformation()));
    b.addPlanner(std::make_shared<ompl::geometric::KPIECE1>(ss.getSpaceInformation()));
    b.addPlanner(std::make_shared<ompl::geometric::RRT>(ss.getSpaceInformation()));

    ob::PlannerPtr quotientSpacePlanner = 
      GetQRRT(ss.getSpaceInformation(), 
          ss.getProblemDefinition(), 
          startVec, 
          goalVec, 
          numLinks, 
          env);

    b.addPlanner( quotientSpacePlanner );

    b.benchmark(request);
    b.saveResultsToFile(boost::str(boost::format("kinematic_%i.log") % numLinks).c_str());

    ot::Benchmark::CompleteExperiment experiment = b.getRecordedExperimentData();

    std::vector<double> meanTime;
    std::vector<std::string> plannerName;
    std::map<double, std::string> plannerTimes;

    for(uint k = 0; k < experiment.planners.size(); k++)
    {
        ot::Benchmark::PlannerExperiment pk = experiment.planners.at(k);
        // plannerName.push_back(pk.name);
        std::vector<ot::Benchmark::RunProperties> runs = pk.runs;

        uint N = runs.size();
        double time = 0;
        for(uint j = 0; j < N; j++)
        {
            ot::Benchmark::RunProperties run = runs.at(j);
            time += std::atof(run["time REAL"].c_str());
        }

        time = time / (double)N;
        pk.name.erase(0,10);

        plannerTimes[time] = pk.name;
    }

    std::cout << "Finished Benchmark (Runtime:" << experiment.maxTime 
      << ", RunCount:" << experiment.runCount << ")" << std::endl;
    std::cout << "Placement (in Seconds)" << std::endl;
    uint ctr = 1;
    for (auto const &p : plannerTimes)
    {
        std::cout << "Place <" << ctr++ << "> Time: " << p.first 
          << " (" <<  p.second << ")" 
          << (ctr<2?" <-- Winner":"")<< std::endl;

    }


    exit(0);
}
