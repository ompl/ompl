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
    
    void bigF (const Eigen::VectorXd &x, Eigen::Ref<Eigen::VectorXd> out) const
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
                if (EXTRAS >= 3) {
                    // Second and third joints must have same x-value.
                    out[LINKS+2] = x[DIM] - x[2*DIM];
                    if (EXTRAS >= 4) {
                        // Third and fourth joints must have the same y-value.
                        out[LINKS+3] = x[2*DIM + 1] - x[3*DIM + 1];
                        if (EXTRAS >= 5) {
                            // First and fifth joints have same y-value.
                            out[LINKS+4] = x[1] - x[4*DIM + 1];
                        }
                    }
                }
            }
        }
    }

    void bigJ (const Eigen::VectorXd &x, Eigen::Ref<Eigen::MatrixXd> out) const
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
                if (EXTRAS >= 3) {
                    out(LINKS+2, DIM) = 1;
                    out(LINKS+2, 2*DIM) = -1;
                    if (EXTRAS >= 4) {
                        out(LINKS+3, 2*DIM + 1) = 1;
                        out(LINKS+3, 3*DIM + 1) = -1;
                        if (EXTRAS >= 5) {
                            out(LINKS+4, 1) = 1;
                            out(LINKS+4, 4*DIM + 1) = -1;
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
        boost::this_thread::sleep(ompl::time::seconds(sleep));
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

/** Print usage information. Does not return. */
void usage (const char *const progname)
{
    std::cout << "Usage: " << progname << " <dimension> <links> <planner> <timelimit>\n";
    printPlanners();
    exit(0);
}

int main (int argc, char **argv)
{
    if (argc != 5 && argc != 7)
        usage(argv[0]);
    
    // Detect artifical validity checking delay.
    double sleep = 0;
    unsigned int extras = 0;
    if (argc == 7)
    {
        if (strcmp(argv[5], "-s") == 0)
            sleep = std::atof(argv[6]);
        else if (strcmp(argv[5], "-e") == 0)
            extras = std::atoi(argv[6]);
        else
            usage(argv[0]);
    }

    // Initialize the atlas for the problem's manifold.
    const std::size_t dimension = std::atoi(argv[1]);
    const std::size_t links = std::atoi(argv[2]);
    if (dimension < 3)
    {
        std::cout << "Dimension must be at least 3.\n";
        std::exit(-1);
    }
    if (links < 4)
    {
        std::cout << "Must have at least 4 links.\n";
        std::exit(-1);
    }

    std::cout << "Planning for " << links << " links in " << dimension << "-D space.\n";
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
    std::cout << "Start: " << x.transpose() << "\nGoal: " << y.transpose() << "\n";
    ompl::base::AtlasStateSpacePtr atlas(new ChainManifold2(dimension, links, links-2, extras));
    ompl::base::StateValidityCheckerFn isValid =
        boost::bind(&ChainManifold2::isValid, (ChainManifold2 *) atlas.get(), sleep, _1, false);

    // These two planners get special treatment. The atlas will pretend to be RealVectorStateSpace
    // for them.
    bool cons = false;
    if (std::strcmp(argv[3], "ConstrainedRRT") == 0 || std::strcmp(argv[3], "CBiRRT2") == 0)
        cons = true;
    if (cons)
        atlas->stopBeingAnAtlas(true);
    
    // All the 'Constrained' classes are loose wrappers for the normal classes. No effect except on
    // the two special planners.
    ompl::geometric::ConstrainedSimpleSetup ss(atlas);
    ompl::base::ConstrainedSpaceInformationPtr si = ss.getConstrainedSpaceInformation();
    atlas->setSpaceInformation(si);
    ss.setStateValidityChecker(isValid);
    si->setValidStateSamplerAllocator(boost::bind(vssa, atlas, _1));
    ompl::base::ConstraintInformationPtr ci(new ompl::base::ConstraintInformation);
    ompl::base::ConstraintPtr c(new ompl::base::AtlasConstraint(atlas));
    ci->addConstraint(c);
    si->setConstraintInformation(ci);
    
    // Atlas parameters
    atlas->setExploration(0.5);
    atlas->setRho(0.5);
    atlas->setAlpha(M_PI/8);
    atlas->setEpsilon(0.2);
    atlas->setDelta(0.02);
    atlas->setMaxChartsPerExtension(200);
    atlas->setMonteCarloSampleCount(0);
    atlas->setProjectionTolerance(1e-8);
    
    // The atlas needs some place to start sampling from. We will make start and goal charts.
    ompl::base::AtlasChart &startChart = atlas->anchorChart(x);
    ompl::base::AtlasChart &goalChart = atlas->anchorChart(y);
    ompl::base::ScopedState<> start(atlas);
    ompl::base::ScopedState<> goal(atlas);
    start->as<ompl::base::AtlasStateSpace::StateType>()->setRealState(x, &startChart);
    goal->as<ompl::base::AtlasStateSpace::StateType>()->setRealState(y, &goalChart);
    ss.setStartAndGoalStates(start, goal);
    
    // Bounds
    ompl::base::RealVectorBounds bounds(atlas->getAmbientDimension());
    bounds.setLow(-10);
    bounds.setHigh(10);
    atlas->as<ompl::base::RealVectorStateSpace>()->setBounds(bounds);
    
    // Choose the planner.
    ompl::base::PlannerPtr planner(parsePlanner(argv[3], si, atlas->getRho_s()));
    if (!planner)
        usage(argv[0]);
    ss.setPlanner(planner);
    ss.setup();
    
    // Set the time limit
    const double runtime_limit = std::atof(argv[4]);
    if (runtime_limit <= 0)
        usage(argv[0]);
    
    // Plan. For 3D problems, we save the chart mesh, planner graph, and solution path in the .ply format.
    // Regardless of dimension, we write the doubles in the path states to a .txt file.
    std::clock_t tstart = std::clock();
    ompl::base::PlannerStatus stat = planner->solve(runtime_limit);
    if (stat)
    {
        const double time = ((double)(std::clock()-tstart))/CLOCKS_PER_SEC;
        
        ompl::geometric::PathGeometric &path = ss.getSolutionPath();
        if (x.size() == 3)
        {
            std::ofstream pathFile("path.ply");
            atlas->dumpPath(path, pathFile, cons);
            pathFile.close();
        }
        
        // Extract the full solution path by re-interpolating between the saved states (except for the special planners)
        const std::vector<ompl::base::State *> &waypoints = path.getStates();
        double length = 0;
        if (cons)
        {
            std::ofstream animFile("anim.txt");
            for (std::size_t i = 0; i < waypoints.size(); i++)
            {
                //std::cout << "[" << waypoints[i]->as<ompl::base::AtlasStateSpace::StateType>()->constVectorView().transpose() << "]\n";
                animFile << waypoints[i]->as<ompl::base::AtlasStateSpace::StateType>()->constVectorView().transpose() << "\n";
            }
            animFile.close();
            length = path.length();
        }
        else
        {
            std::ofstream animFile("anim.txt");
            for (std::size_t i = 0; i < waypoints.size()-1; i++)
            {
                // Denote that we are switching to the next saved state
                //std::cout << "-----\n";
                ompl::base::AtlasStateSpace::StateType *from, *to;
                from = waypoints[i]->as<ompl::base::AtlasStateSpace::StateType>();
                to = waypoints[i+1]->as<ompl::base::AtlasStateSpace::StateType>();
                
                // Traverse the manifold
                std::vector<ompl::base::AtlasStateSpace::StateType *> stateList;
                atlas->followManifold(from, to, true, &stateList);
                if (atlas->equalStates(stateList.front(), stateList.back()))
                {
                    //std::cout << "[" << stateList.front()->constVectorView().transpose() << "]  " << stateList.front()->getChart()->getID() << "\n";
                    animFile << stateList.front()->constVectorView().transpose() << "\n";
                }
                else
                {
                    // Print the intermediate states
                    for (std::size_t i = 1; i < stateList.size(); i++)
                    {
                        //std::cout << "[" << stateList[i]->constVectorView().transpose() << "]  " << stateList[i]->getChart()->getID() << "\n";
                        animFile << stateList[i]->constVectorView().transpose() << "\n";
                        length += atlas->distance(stateList[i-1], stateList[i]);
                    }
                }
                
                // Delete the intermediate states
                for (std::size_t i = 0; i < stateList.size(); i++)
                    atlas->freeState(stateList[i]);
            }
            animFile.close();
            //std::cout << "-----\n";
        }
        if (stat == ompl::base::PlannerStatus::APPROXIMATE_SOLUTION)
            std::cout << "Solution is approximate.\n";
        std::cout << "Length: " << length << "\n";
        std::cout << "Took " << time << " seconds.\n";
    }
    else
    {
        std::cout << "No solution found.\n";
    }
    
    ompl::base::PlannerData data(si);
    planner->getPlannerData(data);
    if (data.properties.find("approx goal distance REAL") != data.properties.end())
        std::cout << "Approx goal distance: " << data.properties["approx goal distance REAL"] << "\n";

    if (!cons)
        std::cout << "Atlas created " << atlas->getChartCount() << " charts.\n";
    
    if (x.size() == 3)
    {
        if (!cons)
        {
            std::ofstream atlasFile("atlas.ply");
            atlas->dumpMesh(atlasFile);
            atlasFile.close();
        }
        
        std::ofstream graphFile("graph.ply");
        ompl::base::PlannerData pd(si);
        planner->getPlannerData(pd);
        atlas->dumpGraph(pd.toBoostGraph(), graphFile, cons);
        graphFile.close();
    }
    
    return 0;
}
