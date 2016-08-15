/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2016, Rice University
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

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/RRTX.h>
#include <ompl/geometric/planners/rrt/RRTsharp.h>
#include <ompl/tools/benchmark/Benchmark.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>

#include <boost/math/constants/constants.hpp>
#include <boost/format.hpp>
#include <fstream>



class ValidityChecker : public ompl::base::StateValidityChecker
{
public:
    ValidityChecker(const ompl::base::SpaceInformationPtr &si): ompl::base::StateValidityChecker(si.get()) {}

    bool isValid(const ompl::base::State* state) const
    {
        const ompl::base::RealVectorStateSpace::StateType* state2D =
            state->as<ompl::base::RealVectorStateSpace::StateType>();

        double sum=0;
        for(unsigned i = 0; i<si_->getStateSpace()->as<ompl::base::RealVectorStateSpace>()->getDimension(); ++i)
            sum += state2D->values[i] * state2D->values[i];

        return sqrt(sum) > 0.1;
    }
};

int main(int argc, char **argv)
{
    if(argc != 2)
    {
        std::cout << "Usage: " << argv[0] << " dimensionOfTheProblem" << std::endl;
        exit(0);
    }
    int dim = atoi(argv[1]);

    ompl::base::StateSpacePtr space(new ompl::base::RealVectorStateSpace(dim));
    ompl::geometric::SimpleSetup ss(space);
    space->as<ompl::base::RealVectorStateSpace>()->setBounds(-1,1);

    ss.setStateValidityChecker(ompl::base::StateValidityCheckerPtr(new ValidityChecker(ss.getSpaceInformation())));

    ompl::base::ScopedState<> start(space), goal(space);
    for(int i = 0; i < dim; ++i)
    {
        start[i] = -1;
        goal[i] = 1;
    }

    ss.setStartAndGoalStates(start, goal);

    // by default, use the Benchmark class
    double runtime_limit = 5, memory_limit = 1024;
    int run_count = 100;
    ompl::tools::Benchmark::Request request(runtime_limit, memory_limit, run_count, 0.05, true, true, false, false);
    ompl::tools::Benchmark b(ss, "Diagonal");

    double range = 0.1*sqrt(dim);

    ompl::base::OptimizationObjectivePtr lengthObj(new ompl::base::PathLengthOptimizationObjective(ss.getSpaceInformation()));
    ompl::base::OptimizationObjectivePtr oop;
    oop = (0.5 / sqrt(dim)) * lengthObj;

    ss.setOptimizationObjective(oop);

    bool knn=true;

    ompl::base::PlannerPtr rrtstar(new ompl::geometric::RRTstar(ss.getSpaceInformation()));
    rrtstar->as<ompl::geometric::RRTstar>()->setName("RRT*");
    rrtstar->as<ompl::geometric::RRTstar>()->setDelayCC(true);
    //rrtstar->as<ompl::geometric::RRTstar>()->setFocusSearch(true);
    rrtstar->as<ompl::geometric::RRTstar>()->setRange(range);
    rrtstar->as<ompl::geometric::RRTstar>()->setKNearest(knn);
    b.addPlanner(rrtstar);
    ompl::base::PlannerPtr rrtsh(new ompl::geometric::RRTsharp(ss.getSpaceInformation()));
    rrtsh->as<ompl::geometric::RRTsharp>()->setRange(range);
    rrtsh->as<ompl::geometric::RRTsharp>()->setKNearest(knn);
    b.addPlanner(rrtsh);
    /*ompl::base::PlannerPtr rrtsh3(new ompl::geometric::RRTsharp(ss.getSpaceInformation()));
    rrtsh3->as<ompl::geometric::RRTsharp>()->setName("RRT#v3");
    rrtsh3->as<ompl::geometric::RRTsharp>()->setRange(range);
    rrtsh3->as<ompl::geometric::RRTsharp>()->setKNearest(knn);
    rrtsh3->as<ompl::geometric::RRTsharp>()->setVariant(3);
    b.addPlanner(rrtsh3);
    ompl::base::PlannerPtr rrtsh2(new ompl::geometric::RRTsharp(ss.getSpaceInformation()));
    rrtsh2->as<ompl::geometric::RRTsharp>()->setName("RRT#v2");
    rrtsh2->as<ompl::geometric::RRTsharp>()->setRange(range);
    rrtsh2->as<ompl::geometric::RRTsharp>()->setKNearest(knn);
    rrtsh2->as<ompl::geometric::RRTsharp>()->setVariant(2);
    b.addPlanner(rrtsh2);*/
    ompl::base::PlannerPtr rrtX1(new ompl::geometric::RRTX(ss.getSpaceInformation()));
    rrtX1->as<ompl::geometric::RRTX>()->setName("RRTX0.1");
    rrtX1->as<ompl::geometric::RRTX>()->setEpsilon(0.1);
    rrtX1->as<ompl::geometric::RRTX>()->setRange(range);
    //rrtX1->as<ompl::geometric::RRTX>()->setVariant(3);
    rrtX1->as<ompl::geometric::RRTX>()->setKNearest(knn);
    b.addPlanner(rrtX1);
    ompl::base::PlannerPtr rrtX2(new ompl::geometric::RRTX(ss.getSpaceInformation()));
    rrtX2->as<ompl::geometric::RRTX>()->setName("RRTX0.01");
    rrtX2->as<ompl::geometric::RRTX>()->setEpsilon(0.01);
    rrtX2->as<ompl::geometric::RRTX>()->setRange(range);
    //rrtX2->as<ompl::geometric::RRTX>()->setVariant(3);
    rrtX2->as<ompl::geometric::RRTX>()->setKNearest(knn);
    b.addPlanner(rrtX2);
    ompl::base::PlannerPtr rrtX3(new ompl::geometric::RRTX(ss.getSpaceInformation()));
    rrtX3->as<ompl::geometric::RRTX>()->setName("RRTX0.001");
    rrtX3->as<ompl::geometric::RRTX>()->setEpsilon(0.001);
    rrtX3->as<ompl::geometric::RRTX>()->setRange(range);
    //rrtX3->as<ompl::geometric::RRTX>()->setVariant(3);
    rrtX3->as<ompl::geometric::RRTX>()->setKNearest(knn);
    b.addPlanner(rrtX3);
    b.benchmark(request);
    b.saveResultsToFile(boost::str(boost::format("Diagonal.log")).c_str());

    exit(0);
}
