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
#include <boost/program_options.hpp>
#include <boost/format.hpp>
#include <boost/tokenizer.hpp>

namespace po = boost::program_options;

const double RANGE = 0.5;

/** Kinematic chain manifold. */
class ChainManifold2 : public ompl::base::AtlasStateSpace
{
public:

    const unsigned int DIM;
    const unsigned int LINKS;
    const double LINKLENGTH;
    const double ENDEFFECTORRADIUS;
    const double JOINTWIDTH;
    const unsigned int EXTRAS;

    ChainManifold2 (unsigned int dim, unsigned int links, double endeffector_radius, unsigned int extras = 0)
        : ompl::base::AtlasStateSpace(dim*links, (dim-1)*links - extras), DIM(dim), LINKS(links), LINKLENGTH(1), ENDEFFECTORRADIUS(endeffector_radius), JOINTWIDTH(0.2), EXTRAS(extras)
    {
        std::cout << "Manifold dimension: " << getManifoldDimension() << "\n";
    }

    void constraintFunction (const Eigen::VectorXd &x, Eigen::Ref<Eigen::VectorXd> out) const
    {
        // Consecutive joints must be a fixed distance apart
        Eigen::VectorXd joint1 = Eigen::VectorXd::Zero(DIM);
        for (unsigned int i = 0; i < LINKS; i++)
        {
            const Eigen::VectorXd joint2 = x.segment(DIM*i, DIM);
            out[i] = (joint1 - joint2).norm() - LINKLENGTH;
            joint1 = joint2;
        }

        if (EXTRAS >= 1) {
            // End effector must lie on a sphere
            out[LINKS] = x.tail(DIM).norm() - ENDEFFECTORRADIUS;
            if (EXTRAS >= 2) {
                // First and second joints must have same z-value.
                out[LINKS+1] = x[2] - x[DIM + 2];
                // First and third joints sqrt(2) apart.
                //out[LINKS+1] = (x.segment(0, DIM) - x.segment(2*DIM, DIM)).norm() - M_SQRT2*LINKLENGTH;
                if (EXTRAS >= 3) {
		    // Second and third joints must have same x-value.
		    out[LINKS+2] = x[DIM] - x[2*DIM];
		    // Third and fifth joints sqrt(2) apart.
		    //out[LINKS+2] = (x.segment(2*DIM, DIM) - x.segment(4*DIM, DIM)).norm() - M_SQRT2*LINKLENGTH;
                    if (EXTRAS >= 4) {
                        // Third and fourth joints must have the same y-value.
                        out[LINKS+3] = x[2*DIM + 1] - x[3*DIM + 1];
                        if (EXTRAS >= 5) {
			    // First and fifth joints have same y-value.
			    out[LINKS+4] = x[1] - x[4*DIM + 1];
			    // Second and fifth joints have same z-value.
			    //out[LINKS+4] = x[DIM + 2] - x[4*DIM + 2];
                        }
                    }
                }
            }
        }
    }

    void jacobianFunction (const Eigen::VectorXd &x, Eigen::Ref<Eigen::MatrixXd> out) const
    {
        out.setZero();
        Eigen::VectorXd plus(DIM*(LINKS+1)); plus.head(DIM*LINKS) = x; plus.tail(DIM) = Eigen::VectorXd::Zero(DIM);
        Eigen::VectorXd minus(DIM*(LINKS+1)); minus.head(DIM) = Eigen::VectorXd::Zero(DIM); minus.tail(DIM*LINKS) = x;
        const Eigen::VectorXd diagonal = plus - minus;
        for (unsigned int i = 0; i < LINKS; i++)
            out.row(i).segment(DIM*i, DIM) = diagonal.segment(DIM*i, DIM).normalized();
        out.block(1, 0, LINKS, DIM*(LINKS-1)) -= out.block(1, DIM, LINKS, DIM*(LINKS-1));

        if (EXTRAS >= 1) {
            out.row(LINKS).tail(DIM) = -diagonal.tail(DIM).normalized().transpose();
            if (EXTRAS >= 2) {
		out(LINKS+1, 2) = 1;
		out(LINKS+1, DIM + 2) = -1;
		//out.row(LINKS+1).segment(0, DIM) = (x.segment(0, DIM) - x.segment(2*DIM, DIM)).normalized();
		//out.row(LINKS+1).segment(2*DIM, DIM) = -out.row(LINKS+1).segment(0, DIM);
                if (EXTRAS >= 3) {
		    out(LINKS+2, DIM) = 1;
		    out(LINKS+2, 2*DIM) = -1;
		    //out.row(LINKS+2).segment(2*DIM, DIM) = (x.segment(2*DIM, DIM) - x.segment(4*DIM, DIM)).normalized();
		    //out.row(LINKS+2).segment(4*DIM, DIM) = -out.row(LINKS+2).segment(2*DIM, DIM);
                    if (EXTRAS >= 4) {
                        out(LINKS+3, 2*DIM + 1) = 1;
                        out(LINKS+3, 3*DIM + 1) = -1;
                        if (EXTRAS >= 5) {
			    out(LINKS+4, 1) = 1;
			    out(LINKS+4, 4*DIM + 1) = -1;
			    //out(LINKS+4, 2) = 1;
			    //out(LINKS+4, 4*DIM + 2) = -1;
                        }
                    }
                }
            }
        }
    }

    /** For the chain example. Joints may not get too close to each other. If \a tough, then the end effector
    * may not occupy states similar to the sphereValid() obstacles (but rotated and scaled). */
    bool isValid (double sleep, const ompl::base::State *state, const bool tough)
    {
        std::this_thread::sleep_for(ompl::time::seconds(sleep));
        Eigen::Ref<const Eigen::VectorXd> x = state->as<ompl::base::AtlasStateSpace::StateType>()->constVectorView();
        for (unsigned int i = 0; i < LINKS-1; i++)
        {
            if (x.segment(DIM*i, DIM).cwiseAbs().maxCoeff() < JOINTWIDTH)
                return false;
            for (unsigned int j = i+1; j < LINKS; j++)
            {
                if ((x.segment(DIM*i, DIM) - x.segment(DIM*j, DIM)).cwiseAbs().maxCoeff() < JOINTWIDTH)
                    return false;
            }
        }

        if (!tough)
            return true;

        Eigen::VectorXd end = x.tail(DIM)/ENDEFFECTORRADIUS;
        const double tmp = end[0];
        end[0] = end[2];
        end[2] = tmp;
        return sphereValid_helper(end);
    }

};

/** To be called between planner runs to clear all charts out of the atlas.
 * Also makes the atlas behave just like RealVectorStateSpace for two of the planners. */
void resetStateSpace(const ompl::base::PlannerPtr &planner)
{
    std::string name = planner->getName();
    ompl::base::AtlasStateSpace *atlas = planner->getSpaceInformation()->getStateSpace()->as<ompl::base::AtlasStateSpace>();
    atlas->clear();
    if (name == "ConstrainedRRT" ||  name == "CBiRRT2")
        atlas->stopBeingAnAtlas(true);
    else
        atlas->stopBeingAnAtlas(false);
}

ompl::geometric::ConstrainedSimpleSetupPtr createChainSetup(std::size_t dimension, std::size_t links, double sleep, unsigned int extras)
{
    Eigen::VectorXd x = Eigen::VectorXd::Zero(dimension * links);
    Eigen::VectorXd y = Eigen::VectorXd::Zero(dimension * links);
    const std::size_t kink = links-3;   // Require kink >= 1 && kink <= links-3
    for (std::size_t i = 0; i < links; i++)
    {
        x[dimension * i] = i+1 - (i >= kink) - (i > kink+1);
        y[dimension * i] = -x[dimension * i];
        x[dimension * i + 1] = (i >= kink && i <= kink+1);
        y[dimension * i + 2] = (i >= kink && i <= kink+1);
    }
    ompl::base::AtlasStateSpacePtr atlas(new ChainManifold2(dimension, links, links-2, extras));
    ompl::base::StateValidityCheckerFn isValid =
        std::bind(&ChainManifold2::isValid, (ChainManifold2 *) atlas.get(), sleep, std::placeholders::_1, false);

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
    atlas->setAlpha(M_PI/8);
    atlas->setEpsilon(0.2);
    atlas->setDelta(0.02);
    atlas->setMaxChartsPerExtension(200);
    atlas->setProjectionTolerance(1e-8);

    // The atlas needs some place to start sampling from. We will make start and goal charts.
    ompl::base::AtlasChart &startChart = atlas->anchorChart(x);
    ompl::base::AtlasChart &goalChart = atlas->anchorChart(y);
    ompl::base::ScopedState<> start(atlas);
    ompl::base::ScopedState<> goal(atlas);
    start->as<ompl::base::AtlasStateSpace::StateType>()->setRealState(x, &startChart);
    goal->as<ompl::base::AtlasStateSpace::StateType>()->setRealState(y, &goalChart);
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
    const ChainManifold2 *atlas = ss->getStateSpace()->as<ChainManifold2>();
    ompl::geometric::PathGeometric &path = ss->getSolutionPath();

    if (atlas->DIM == 3)
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
        for (std::size_t i = 0; i < waypoints.size(); i++)
            animFile << waypoints[i]->as<ompl::base::AtlasStateSpace::StateType>()->constVectorView().transpose() << "\n";
        animFile.close();
    }
    else
    {
        std::ofstream animFile("anim.txt");
        for (std::size_t i = 0; i < waypoints.size()-1; i++)
        {
            // Denote that we are switching to the next saved state
            ompl::base::AtlasStateSpace::StateType *from, *to;
            from = waypoints[i]->as<ompl::base::AtlasStateSpace::StateType>();
            to = waypoints[i+1]->as<ompl::base::AtlasStateSpace::StateType>();

            // Traverse the manifold
            std::vector<ompl::base::AtlasStateSpace::StateType *> stateList;
            atlas->followManifold(from, to, true, &stateList);
            if (atlas->equalStates(stateList.front(), stateList.back()))
                animFile << stateList.front()->constVectorView().transpose() << "\n";
            else
                // Print the intermediate states
                for (std::size_t i = 1; i < stateList.size(); i++)
                    animFile << stateList[i]->constVectorView().transpose() << "\n";

            // Delete the intermediate states
            for (std::size_t i = 0; i < stateList.size(); i++)
                atlas->freeState(stateList[i]);
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

    if (atlas->DIM == 3)
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

int main (int argc, char **argv)
{
    unsigned int numDimensions, numConstraints, runCount;
    double maxTime;
    std::string plannerList;
    bool savePath;
    po::options_description desc("Options");
    desc.add_options()
        ("help", "show help message")
        ("numdim,d", po::value<unsigned int>(&numDimensions)->default_value(3),
            "number of dimensions")
        ("numconstraints,c", po::value<unsigned int>(&numConstraints)->default_value(6),
            "number of constraints (should be between 6 and 10)")
        ("runcount,n", po::value<unsigned int>(&runCount)->default_value(100),
            "number of runs per planner")
        ("maxtime,t", po::value<double>(&maxTime)->default_value(60.),
            "maximum time for each run")
        ("plannerlist,p", po::value<std::string>(&plannerList)->default_value("CBiRRT2,EST,PRM,RRT,RRTintermediate,RRTConnectIntermediate,RRTConnect,KPIECE1,STRIDE,RealEST,BiRealEST"),
            "comma-separated list of planners")
        ("savepath,s", po::bool_switch(&savePath)->default_value(false), "save path of last run")
    ;

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
    for(boost::tokenizer<>::iterator planner=tok.begin(); planner!=tok.end(); ++planner)
        bench.addPlanner(ompl::base::PlannerPtr(parsePlanner(planner->c_str(), si, RANGE)));

    bench.setPreRunEvent(&resetStateSpace);
    bench.addExperimentParameter("numdimensions", "INTEGER", boost::lexical_cast<std::string>(numDimensions));
    bench.addExperimentParameter("numconstraints", "INTEGER", boost::lexical_cast<std::string>(numConstraints));
    bench.benchmark(request);
    bench.saveResultsToFile((boost::format("atlaschain_%1%_%2%.log") % numDimensions % numConstraints).str().c_str());
}
