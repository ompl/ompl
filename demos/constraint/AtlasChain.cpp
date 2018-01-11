/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, Rice University
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

/* Author: Caleb Voss */

#include "AtlasCommon.h"

#include <ompl/base/spaces/RealVectorStateProjections.h>
#include <ompl/tools/benchmark/Benchmark.h>
#include <boost/format.hpp>
#include <boost/program_options.hpp>
#include <boost/tokenizer.hpp>

namespace po = boost::program_options;

const double RANGE = 0.5;

/** To be called between planner runs to clear all charts out of the atlas.
 * Also makes the atlas behave just like RealVectorStateSpace for two of the planners. */
void resetStateSpace(const ompl::base::PlannerPtr &planner)
{
    std::string name = planner->getName();
    ompl::base::AtlasStateSpace *atlas =
        planner->getSpaceInformation()->getStateSpace()->as<ompl::base::AtlasStateSpace>();
    atlas->clear();
    if (name == "ConstrainedRRT" || name == "CBiRRT2")
        atlas->setMode(ompl::base::AtlasStateSpace::REALVECTOR);
    else
        atlas->setMode(ompl::base::AtlasStateSpace::ATLAS);
}

ompl::geometric::ConstrainedSimpleSetupPtr createChainSetup(std::size_t dimension, std::size_t links, double sleep,
                                                            unsigned int extras)
{
    Eigen::VectorXd x = Eigen::VectorXd::Zero(dimension * links);
    Eigen::VectorXd y = Eigen::VectorXd::Zero(dimension * links);
    const std::size_t kink = links - 3;  // Need kink >= 1 && kink <= links-3.
    for (std::size_t i = 0; i < links; i++)
    {
        x[dimension * i] = i + 1 - (i >= kink) - (i > kink + 1);
        y[dimension * i] = -x[dimension * i];
        x[dimension * i + 1] = (i >= kink && i <= kink + 1);
        y[dimension * i + 2] = (i >= kink && i <= kink + 1);
    }
    ompl::base::AtlasStateSpacePtr atlas(new ChainManifold(dimension, links, extras));
    ompl::base::StateValidityCheckerFn isValid =
        std::bind(&ChainManifold::isValid, (ChainManifold *)atlas.get(), sleep, std::placeholders::_1, false);

    // All the 'Constrained' classes are loose wrappers for the normal classes. No effect except on
    // the two special planners.
    ompl::geometric::ConstrainedSimpleSetupPtr ss(new ompl::geometric::ConstrainedSimpleSetup(atlas));
    ompl::base::ConstrainedSpaceInformationPtr si = ss->getConstrainedSpaceInformation();
    atlas->setSpaceInformation(si);
    ss->setStateValidityChecker(isValid);
    si->setValidStateSamplerAllocator(vssa);
    ompl::base::ConstraintInformationPtr ci(new ompl::base::ConstraintInformation);
    ompl::base::ConstraintPtr c(new ompl::base::AtlasConstraint(atlas));
    ci->addConstraint(c);
    si->setConstraintInformation(ci);

    // Atlas parameters
    atlas->setExploration(0.5);
    atlas->setRho(RANGE);
    atlas->setAlpha(M_PI / 8);
    atlas->setEpsilon(0.2);
    atlas->setDelta(0.02);
    atlas->setMaxChartsPerExtension(200);
    atlas->setProjectionTolerance(1e-8);

    // The atlas needs some place to start sampling from. We will make start and goal charts.
    ompl::base::AtlasChart *startChart = atlas->anchorChart(x);
    ompl::base::AtlasChart *goalChart = atlas->anchorChart(y);
    ompl::base::ScopedState<> start(atlas);
    ompl::base::ScopedState<> goal(atlas);
    start->as<ompl::base::AtlasStateSpace::StateType>()->setRealState(x, startChart);
    goal->as<ompl::base::AtlasStateSpace::StateType>()->setRealState(y, goalChart);
    ss->setStartAndGoalStates(start, goal);

    // Bounds
    ompl::base::RealVectorBounds bounds(atlas->getAmbientDimension());
    bounds.setLow(-10);
    bounds.setHigh(10);
    atlas->as<ompl::base::RealVectorStateSpace>()->setBounds(bounds);

    return ss;
}

void saveSolutionPath(const ompl::geometric::ConstrainedSimpleSetupPtr &ss, bool cons)
{
    const ChainManifold *atlas = ss->getStateSpace()->as<ChainManifold>();
    ompl::geometric::PathGeometric &path = ss->getSolutionPath();

    if (atlas->workspaceDim == 3)
    {
        std::ofstream pathFile("path.ply");
        atlas->dumpPath(path, pathFile, cons);
        pathFile.close();
    }

    // Extract the full solution path by re-interpolating between the saved states (except for the special planners)
    const std::vector<ompl::base::State *> &waypoints = path.getStates();
    if (cons)
    {
        std::ofstream animFile("anim.txt");
        for (auto waypoint : waypoints)
            animFile << waypoint->as<ompl::base::AtlasStateSpace::StateType>()->constVectorView().transpose()
                     << "\n";
        animFile.close();
    }
    else
    {
        std::ofstream animFile("anim.txt");
        for (std::size_t i = 0; i < waypoints.size() - 1; i++)
        {
            // Denote that we are switching to the next saved state
            ompl::base::AtlasStateSpace::StateType *from, *to;
            from = waypoints[i]->as<ompl::base::AtlasStateSpace::StateType>();
            to = waypoints[i + 1]->as<ompl::base::AtlasStateSpace::StateType>();

            // Traverse the manifold
            std::vector<ompl::base::AtlasStateSpace::StateType *> stateList;
            atlas->traverseManifold(from, to, true, &stateList);
            if (atlas->equalStates(stateList.front(), stateList.back()))
                animFile << stateList.front()->constVectorView().transpose() << "\n";
            else
                // Print the intermediate states
                for (std::size_t i = 1; i < stateList.size(); i++)
                    animFile << stateList[i]->constVectorView().transpose() << "\n";

            // Delete the intermediate states
            for (auto & i : stateList)
                atlas->freeState(i);
        }
        animFile.close();
    }
    if (ss->getLastPlannerStatus() == ompl::base::PlannerStatus::APPROXIMATE_SOLUTION)
        std::cout << "Solution is approximate.\n";
    std::cout << "Length: " << path.length() << "\n";

    ompl::base::PlannerData data(ss->getSpaceInformation());
    ss->getPlanner()->getPlannerData(data);
    if (data.properties.find("approx goal distance REAL") != data.properties.end())
        std::cout << "Approx goal distance: " << data.properties["approx goal distance REAL"] << "\n";

    if (!cons)
        std::cout << "Atlas created " << atlas->getChartCount() << " charts.\n";

    if (atlas->workspaceDim == 3)
    {
        if (!cons)
        {
            std::ofstream atlasFile("atlas.ply");
            atlas->dumpMesh(atlasFile);
            atlasFile.close();
        }

        std::ofstream graphFile("graph.ply");
        atlas->dumpGraph(data.toBoostGraph(), graphFile, cons);
        graphFile.close();
    }
}

int main(int argc, char **argv)
{
    unsigned int numDimensions, numConstraints, runCount;
    double maxTime;
    std::string plannerList;
    bool savePath;
    po::options_description desc("Options");
    desc.add_options()("help", "show help message")(
        "numdim,d", po::value<unsigned int>(&numDimensions)->default_value(3),
        "number of dimensions")("numconstraints,c", po::value<unsigned int>(&numConstraints)->default_value(6),
                                "number of constraints (should be between 6 and 10)")(
        "runcount,n", po::value<unsigned int>(&runCount)->default_value(100), "number of runs per planner")(
        "maxtime,t", po::value<double>(&maxTime)->default_value(60.), "maximum time for each run")(
        "plannerlist,p", po::value<std::string>(&plannerList)->default_value("CBiRRT2,EST,PRM,RRT,RRTintermediate,"
                                                                             "RRTConnectIntermediate,RRTConnect,"
                                                                             "KPIECE1,STRIDE,RealEST,BiRealEST"),
        "comma-separated list of planners")("savepath,s", po::bool_switch(&savePath)->default_value(false),
                                            "save path of last run");

    po::variables_map vm;
    // po::store(po::parse_command_line(argc, argv, desc,
    //     po::command_line_style::unix_style ^ po::command_line_style::allow_short), vm);
    po::store(po::command_line_parser(argc, argv).options(desc).run(), vm);
    po::notify(vm);

    if (vm.count("help"))
    {
        std::cout << desc << "\n";
        printPlanners();
        std::cout << "\n";
        return 1;
    }

    ompl::geometric::ConstrainedSimpleSetupPtr ss = createChainSetup(numDimensions, 5, 0., numConstraints - 5);
    const ompl::base::SpaceInformationPtr &si = ss->getSpaceInformation();
    ompl::tools::Benchmark bench(*ss, "AtlasChain");
    ompl::tools::Benchmark::Request request;
    request.maxTime = maxTime;
    request.runCount = runCount;
    request.maxMem = 1e10;
    request.simplify = false;

    boost::tokenizer<> tok(plannerList);
    for (boost::tokenizer<>::iterator planner = tok.begin(); planner != tok.end(); ++planner)
        bench.addPlanner(ompl::base::PlannerPtr(parsePlanner(planner->c_str(), si, RANGE)));

    bench.setPreRunEvent(&resetStateSpace);
    bench.addExperimentParameter("numdimensions", "INTEGER", boost::lexical_cast<std::string>(numDimensions));
    bench.addExperimentParameter("numconstraints", "INTEGER", boost::lexical_cast<std::string>(numConstraints));
    bench.benchmark(request);
    bench.saveResultsToFile((boost::format("atlaschain_%1%_%2%.log") % numDimensions % numConstraints).str().c_str());
}
