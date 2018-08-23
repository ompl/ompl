/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2013, Rice University
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

/* Author: Bryant Gipson, Mark Moll */

#include "KinematicChain.h"

int main(int argc, char **argv)
{
    if (argc < 2)
    {
        std::cout << "Usage:\n" << argv[0] << " <num_links>\n";
        exit(0);
    }

    auto numLinks = std::stoul(argv[1]);
    Environment env = createHornEnvironment(numLinks, log((double)numLinks) / (double)numLinks);
    auto chain(std::make_shared<KinematicChainSpace>(numLinks, 1. / (double)numLinks, &env));
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

    // SEKRIT BONUS FEATURE:
    // if you specify a second command line argument, it will solve the
    // problem just once with STRIDE and print out the solution path.
    if (argc > 2)
    {
        ss.setPlanner(std::make_shared<ompl::geometric::STRIDE>(ss.getSpaceInformation()));
        ss.setup();
        ss.print();
        ss.solve(3600);
        ss.simplifySolution();

        std::ofstream pathfile(boost::str(boost::format("kinematic_path_%i.dat") % numLinks).c_str());
        ss.getSolutionPath().printAsMatrix(pathfile);
        exit(0);
    }

    // by default, use the Benchmark class
    double runtime_limit = 60, memory_limit = 1024;
    int run_count = 20;
    ompl::tools::Benchmark::Request request(runtime_limit, memory_limit, run_count, 0.5);
    ompl::tools::Benchmark b(ss, "KinematicChain");
    b.addExperimentParameter("num_links", "INTEGER", std::to_string(numLinks));

    b.addPlanner(std::make_shared<ompl::geometric::STRIDE>(ss.getSpaceInformation()));
    b.addPlanner(std::make_shared<ompl::geometric::EST>(ss.getSpaceInformation()));
    b.addPlanner(std::make_shared<ompl::geometric::KPIECE1>(ss.getSpaceInformation()));
    b.addPlanner(std::make_shared<ompl::geometric::RRT>(ss.getSpaceInformation()));
    b.addPlanner(std::make_shared<ompl::geometric::PRM>(ss.getSpaceInformation()));
    b.benchmark(request);
    b.saveResultsToFile(boost::str(boost::format("kinematic_%i.log") % numLinks).c_str());

    exit(0);
}
