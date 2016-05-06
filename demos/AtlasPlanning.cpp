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

/** Print usage information. Does not return. */
void usage (const char *const progname)
{
    std::cout << "Usage: " << progname << " <problem> <planner> <timelimit>\n";
    printProblems();
    printPlanners();
    exit(0);
}

int main (int argc, char **argv)
{
    if (argc != 4 && argc != 6)
        usage(argv[0]);
    
    // Detect artifical validity checking delay.
    double sleep = 0;
    if (argc == 6)
    {
        if (strcmp(argv[4], "-s") != 0)
            usage(argv[0]);
        sleep = std::atof(argv[5]);
    }

    // Initialize the atlas for the problem's manifold
    Eigen::VectorXd x, y;
    ompl::base::StateValidityCheckerFn isValid;
    ompl::base::AtlasStateSpacePtr atlas(parseProblem(argv[1], x, y, isValid, sleep));
    if (!atlas)
        usage(argv[0]);
    
    // These two planners get special treatment. The atlas will pretend to be RealVectorStateSpace
    // for them.
    bool cons = false;
    if (std::strcmp(argv[2], "ConstrainedRRT") == 0 || std::strcmp(argv[2], "CBiRRT2") == 0)
        cons = true;
    if (cons)
        atlas->stopBeingAnAtlas(true);
    
    // All the 'Constrained' classes are loose wrappers for the normal classes. No effect except on
    // the two special planners.
    ompl::geometric::ConstrainedSimpleSetup ss(atlas);
    ompl::base::ConstrainedSpaceInformationPtr si = ss.getConstrainedSpaceInformation();
    atlas->setSpaceInformation(si);
    ss.setStateValidityChecker(isValid);
    si->setValidStateSamplerAllocator(vssa);
    ompl::base::ConstraintInformationPtr ci(new ompl::base::ConstraintInformation);
    ompl::base::ConstraintPtr c(new ompl::base::AtlasConstraint(atlas));
    ci->addConstraint(c);
    si->setConstraintInformation(ci);
    
    // Atlas parameters
    atlas->setExploration(0.8);
    atlas->setRho(0.5); // 0.2
    atlas->setAlpha(M_PI/8);
    atlas->setEpsilon(0.2); // 0.1
    atlas->setDelta(0.02);
    atlas->setMaxChartsPerExtension(200);
    
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
    ompl::base::PlannerPtr planner(parsePlanner(argv[2], si, atlas->getRho_s()));
    if (!planner)
        usage(argv[0]);
    ss.setPlanner(planner);
    ss.setup();
    
    // Set the time limit
    const double runtime_limit = std::atof(argv[3]);
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
        atlas->dumpGraph(pd.toBoostGraph(), graphFile, /*cons*/ true);
        graphFile.close();
    }

    std::cout << atlas->estimateFrontierPercent() << "% open.\n";
    
    return 0;
}
