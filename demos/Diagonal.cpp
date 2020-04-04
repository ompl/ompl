/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2016, Georgia Institute of Technology
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

/* Author: Florian Hauer */

#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/rrt/RRTXstatic.h>
#include <ompl/geometric/planners/rrt/RRTsharp.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/tools/benchmark/Benchmark.h>

#include <boost/format.hpp>
#include <boost/math/constants/constants.hpp>
#include <fstream>

class ValidityChecker : public ompl::base::StateValidityChecker
{
public:
    ValidityChecker(const ompl::base::SpaceInformationPtr &si) : ompl::base::StateValidityChecker(si)
    {
    }

    bool isValid(const ompl::base::State *state) const override
    {
        const auto *state2D = state->as<ompl::base::RealVectorStateSpace::StateType>();

        double sum = 0;
        for (unsigned i = 0; i < si_->getStateSpace()->getDimension(); ++i)
            sum += state2D->values[i] * state2D->values[i];

        return sqrt(sum) > 0.1;
    }
};

int main(int argc, char **argv)
{
    if (argc != 2)
    {
        std::cout << "Usage: " << argv[0] << " dimensionOfTheProblem" << std::endl;
        exit(0);
    }
    int dim = atoi(argv[1]);

    auto space(std::make_shared<ompl::base::RealVectorStateSpace>(dim));
    ompl::geometric::SimpleSetup ss(space);
    const ompl::base::SpaceInformationPtr &si = ss.getSpaceInformation();
    space->setBounds(-1, 1);

    ss.setStateValidityChecker(std::make_shared<ValidityChecker>(si));

    ompl::base::ScopedState<> start(space), goal(space);
    for (int i = 0; i < dim; ++i)
    {
        start[i] = -1;
        goal[i] = 1;
    }

    ss.setStartAndGoalStates(start, goal);

    // by default, use the Benchmark class
    double runtime_limit = 5, memory_limit = 1024;
    int run_count = 100;
    ompl::tools::Benchmark::Request request(runtime_limit, memory_limit, run_count, 0.05, true, true, false);
    ompl::tools::Benchmark b(ss, "Diagonal");

    double range = 0.1 * sqrt(dim);

    auto lengthObj(std::make_shared<ompl::base::PathLengthOptimizationObjective>(si));
    ompl::base::OptimizationObjectivePtr oop((0.5 / sqrt(dim)) * lengthObj);

    ss.setOptimizationObjective(oop);

    bool knn = true;

    auto rrtstar(std::make_shared<ompl::geometric::RRTstar>(si));
    rrtstar->setName("RRT*");
    rrtstar->setDelayCC(true);
    // rrtstar->setFocusSearch(true);
    rrtstar->setRange(range);
    rrtstar->setKNearest(knn);
    b.addPlanner(rrtstar);
    auto rrtsh(std::make_shared<ompl::geometric::RRTsharp>(si));
    rrtsh->setRange(range);
    rrtsh->setKNearest(knn);
    b.addPlanner(rrtsh);
    /*auto rrtsh3(std::make_shared<ompl::geometric::RRTsharp>(si));
    rrtsh3->setName("RRT#v3");
    rrtsh3->setRange(range);
    rrtsh3->setKNearest(knn);
    rrtsh3->setVariant(3);
    b.addPlanner(rrtsh3);
    auto rrtsh2(std::make_shared<ompl::geometric::RRTsharp>(si));
    rrtsh2->setName("RRT#v2");
    rrtsh2->setRange(range);
    rrtsh2->setKNearest(knn);
    rrtsh2->setVariant(2);
    b.addPlanner(rrtsh2);*/
    auto rrtX1(std::make_shared<ompl::geometric::RRTXstatic>(si));
    rrtX1->setName("RRTX0.1");
    rrtX1->setEpsilon(0.1);
    rrtX1->setRange(range);
    // rrtX1->setVariant(3);
    rrtX1->setKNearest(knn);
    b.addPlanner(rrtX1);
    auto rrtX2(std::make_shared<ompl::geometric::RRTXstatic>(si));
    rrtX2->setName("RRTX0.01");
    rrtX2->setEpsilon(0.01);
    rrtX2->setRange(range);
    // rrtX2->setVariant(3);
    rrtX2->setKNearest(knn);
    b.addPlanner(rrtX2);
    auto rrtX3(std::make_shared<ompl::geometric::RRTXstatic>(si));
    rrtX3->setName("RRTX0.001");
    rrtX3->setEpsilon(0.001);
    rrtX3->setRange(range);
    // rrtX3->setVariant(3);
    rrtX3->setKNearest(knn);
    b.addPlanner(rrtX3);
    b.benchmark(request);
    b.saveResultsToFile(boost::str(boost::format("Diagonal.log")).c_str());

    exit(0);
}
