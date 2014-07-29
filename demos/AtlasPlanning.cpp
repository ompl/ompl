/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, Rice University
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

#include <fstream>

#include <ompl/base/ScopedState.h>
#include <ompl/base/spaces/AtlasChart.h>
#include <ompl/base/spaces/AtlasStateSpace.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/planners/est/EST.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/prm/SPARStwo.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>

/** Simple manifold example: the unit sphere. */
Eigen::VectorXd Fsphere (const Eigen::VectorXd &x)
{
    Eigen::VectorXd f(1);
    f[0] = x.norm() - 1;
    return f;
}

/** Jacobian of Fsphere(x). */
Eigen::MatrixXd Jsphere (const Eigen::VectorXd &x)
{
    return x.transpose().normalized();
}

/** Sphere has 3 ring-shaped obstacles on latitudinal lines, with a small gap in each. */
bool sphereValid (const ompl::base::State *state)
{
    const Eigen::VectorXd &x = state->as<ompl::base::AtlasStateSpace::StateType>()->toVector();
    
    if (-0.75 < x[2] && x[2] < -0.6)
    {
        if (-0.2 < x[1] && x[1] < 0.2)
            return x[0] > 0;
        return false;
    }
    else if (-0.125 < x[2] && x[2] < 0.125)
    {
        if (-0.2 < x[1] && x[1] < 0.2)
            return x[0] < 0;
        return false;
    }
    else if (0.6 < x[2] && x[2] < 0.75)
    {
        if (-0.2 < x[0] && x[0] < 0.2)
            return x[1] > 0;
        return false;
    }
    return true;
}

/** More complicated manifold example: Consider three points in 3D space: p1, p2, and p3. Put p1 exactly
 * 3 units above p2, and have p3 orbit p1 at a distance of 2 in a plane perpendicular to p1. That's 9
 * dimensions, with 5 constraints, to create a 4D manifold. */
Eigen::VectorXd Fcomplicated (const Eigen::VectorXd &x)
{
    Eigen::VectorXd f(5);
    Eigen::VectorXd p1, p2, p3;
    
    // Separate out the three points
    p1 = x.segment(0, 3);
    p2 = x.segment(3, 3);
    p3 = x.segment(6, 3);
    
    f[0] = p1[0] - p2[0];           // p1, p2 have same x coordinate
    f[1] = p1[1] - p2[1];           // p1, p2 have same y coordinate
    f[2] = p1[2] - p2[2] - 3;       // p1 is 3 units above p2
    f[3] = (p1 - p3).norm() - 2;    // p3 is 2 units away from p1
    f[4] = (p3 - p1).dot(p1);       // p3 lies in the plane perpendicular to p1
    return f;
}

/** Jacobian of Fcomplicated(x).*/
Eigen::MatrixXd Jcomplicated (const Eigen::VectorXd &x)
{
    Eigen::VectorXd p1, p2, p3;
    p1 = x.segment(0, 3);
    p2 = x.segment(3, 3);
    p3 = x.segment(6, 3);
    
    Eigen::MatrixXd j = Eigen::MatrixXd::Zero(5, 9);
    j(0,0) = 1; j(0,3) = -1;
    j(1,1) = 1; j(1,4) = -1;
    j(2,2) = 1; j(2,5) = -1;
    j.row(3).head(3) = (p1 - p3).transpose().normalized(); j.row(3).tail(3) = -j.row(3).head(3);
    j.row(4).head(3) = (p3 - 2*p1).transpose(); j.row(4).tail(3) = p1.transpose();
    return j;
}

/** Klein bottle manifold. */
Eigen::VectorXd FKleinBottle (const Eigen::VectorXd &x)
{
    const double p = x.squaredNorm() + 2*x[1] - 1;
    const double n = x.squaredNorm() - 2*x[1] - 1;
    Eigen::VectorXd f(1);
    f[0] = p*(n*n - 8*x[2]*x[2]) + 16*x[0]*x[1]*n;
    
    return f;
}

/** Torus manifold. */
Eigen::VectorXd Ftorus (const Eigen::VectorXd &x)
{
    Eigen::VectorXd f(1);
    const double r1 = 2;
    const double r2 = 1;
    Eigen::VectorXd c(3); c << x[0], x[1], 0;
    f[0] = (x - r1 * c.normalized()).norm() - r2;
    
    return f;
}

Eigen::MatrixXd Jtorus (const Eigen::VectorXd &x)
{
    Eigen::MatrixXd j(1, 3);
    const double r1 = 2;
    const double r2 = 1;
    const double xySquaredNorm = x[0]*x[0] + x[1]*x[1];
    const double xyNorm = std::sqrt(xySquaredNorm);
    const double denom = std::sqrt(x[2]*x[2] + (xyNorm - r1)*(xyNorm - r1));
    const double c = (xyNorm - r1) * (xyNorm*xySquaredNorm) / (xySquaredNorm * xySquaredNorm * denom);
    j(0,0) = x[0] * c;
    j(0,1) = x[1] * c;
    j(0,2) = x[2] / denom;
    
    return j;
}

/** Every state is valid. */
bool always (const ompl::base::State *)
{
    return true;
}

/** Every state has a small chance to be invalid. On very rare occasions, the start or goal is declared
 * invalid and the planning fails. */
bool almostAlways (const ompl::base::State *)
{
    return ((double) std::rand())/RAND_MAX < 0.95;
}

/** States surrounding the goal are invalid, making it unreachable. */
bool unreachable (const ompl::base::State *state, const Eigen::VectorXd &goal)
{
    return std::abs((state->as<ompl::base::AtlasStateSpace::StateType>()->toVector() - goal).norm() - 0.25) > 0.249;
}

/** Print the state and its chart ID. */
void printState (const ompl::base::AtlasStateSpace::StateType *state)
{
    std::cout << "[" << state->toVector().transpose() << "]  " << state->getChart().getID() << "\n";
}

/** Initialize the atlas for the sphere problem and store the start and goal vectors. */
ompl::base::AtlasStateSpace *initSphereProblem (Eigen::VectorXd &x, Eigen::VectorXd &y, ompl::base::StateValidityCheckerFn &isValid)
{
    const std::size_t dim = 3;
    
    // Start and goal points
    x = Eigen::VectorXd(dim); x << 0, 0, -1;
    y = Eigen::VectorXd(dim); y << 0, 0, 1;
    
    // Validity checker
    isValid = &sphereValid;
    
    // Atlas initialization (can use numerical methods to compute the Jacobian, but giving an explicit function is faster)
    return new ompl::base::AtlasStateSpace(dim, Fsphere, Jsphere);
}

/** Initialize the atlas for the complicated problem and store the start and goal vectors. */
ompl::base::AtlasStateSpace *initComplicatedProblem (Eigen::VectorXd &x, Eigen::VectorXd &y, ompl::base::StateValidityCheckerFn &isValid)
{
    const std::size_t dim = 9;
    
    // Start and goal points
    x = Eigen::VectorXd(dim); x << 0, 0, 3, 0, 0, 0, 2, 0, 3;
    y = Eigen::VectorXd(dim); y << -4, -4, 0, -4, -4, -3, -4, -4, 2;
    
    // Validity checker
    isValid = &almostAlways;
    
    // Atlas initialization (can use numerical methods to compute the Jacobian, but giving an explicit function is faster)
    return new ompl::base::AtlasStateSpace(dim, Fcomplicated, Jcomplicated);
}

/** Initialize the atlas for the sphere problem and store the start and goal vectors. */
ompl::base::AtlasStateSpace *initKleinBottleProblem (Eigen::VectorXd &x, Eigen::VectorXd &y, boost::function<bool (const ompl::base::State *)> &isValid)
{
    const std::size_t dim = 3;
    
    // Start and goal points
    x = Eigen::VectorXd(dim); x << 0, 0, 1;
    y = Eigen::VectorXd(dim); y << 1, 1, 2.94481779371197;
    
    // Validity checker
    isValid = &always;
    
    // Atlas initialization
    return new ompl::base::AtlasStateSpace(dim, FKleinBottle);
}

/** Initialize the atlas for the torus problem and store the start and goal vectors. */
ompl::base::AtlasStateSpace *initTorusProblem (Eigen::VectorXd &x, Eigen::VectorXd &y, ompl::base::StateValidityCheckerFn &isValid)
{
    const std::size_t dim = 3;
    
    // Start and goal points;
    x = Eigen::VectorXd(dim); x << -2, 0, -1;
    y = Eigen::VectorXd(dim); y << 2, 0, 1;
    
    // Validity checker
    isValid = &always;
    
    // Atlas initialization
    return new ompl::base::AtlasStateSpace(dim, Ftorus, Jtorus);
}

/** Allocate a sampler for the atlas that only returns valid points. */
ompl::base::ValidStateSamplerPtr vssa (const ompl::base::AtlasStateSpacePtr &atlas, const ompl::base::SpaceInformation *si)
{
    return ompl::base::ValidStateSamplerPtr(new ompl::base::AtlasValidStateSampler(atlas, si));
}


int main (int, char *[])
{
    // Initialize the atlas for a problem (you can try the other one too)
    Eigen::VectorXd x, y;
    ompl::base::StateValidityCheckerFn isValid;
    ompl::base::AtlasStateSpacePtr atlas(initKleinBottleProblem(x, y, isValid));
    ompl::base::StateSpacePtr space(atlas);
    ompl::base::SpaceInformationPtr si(new ompl::base::SpaceInformation(space));
    atlas->setSpaceInformation(si);
    si->setStateValidityChecker(isValid);
    si->setValidStateSamplerAllocator(boost::bind(vssa, atlas, _1));
    const ompl::base::AtlasChart &startChart = atlas->anchorChart(x);
    const ompl::base::AtlasChart &goalChart = atlas->anchorChart(y);
    ompl::base::ScopedState<> start(space);
    ompl::base::ScopedState<> goal(space);
    start->as<ompl::base::AtlasStateSpace::StateType>()->setRealState(x, startChart);
    goal->as<ompl::base::AtlasStateSpace::StateType>()->setRealState(y, goalChart);
    
    atlas->setExploration(0.5);
    atlas->setRho(0.1);
    atlas->setAlpha(M_PI/32);
    atlas->setEpsilon(0.05);
    atlas->setDelta(0.01);
    
    // More setup for the space and problem definition
    ompl::base::RealVectorBounds bounds(atlas->getAmbientDimension());
    bounds.setLow(-10);
    bounds.setHigh(10);
    atlas->setBounds(bounds);
    ompl::base::ProblemDefinitionPtr pdef(new ompl::base::ProblemDefinition(si));
    pdef->setStartAndGoalStates(start, goal);
    si->setup();
    
    // Choose the planner. Try others, like RRT, RRTstar, EST, PRM, ...
    ompl::base::PlannerPtr planner(new ompl::geometric::RRTstar(si));
    planner->setProblemDefinition(pdef);
    planner->setup();
    
    // Plan
    std::clock_t tstart = std::clock();
    ompl::base::PlannerStatus stat;
    if ((stat = planner->solve(300)) == ompl::base::PlannerStatus::EXACT_SOLUTION)
    {
        double time = ((double)(std::clock()-tstart))/CLOCKS_PER_SEC;
        
        // Extract the solution path by re-interpolating between the saved states
        std::stringstream v;
        std::size_t vcount = 0;
        const std::vector<ompl::base::State *> &waypoints =
            boost::dynamic_pointer_cast<ompl::geometric::PathGeometric>(pdef->getSolutionPath())->getStates();
        double length = 0;
        for (std::size_t i = 0; i < waypoints.size()-1; i++)
        {
            // Denote that we are switching to the next saved state
            std::cout << "-----\n";
            ompl::base::AtlasStateSpace::StateType *from, *to;
            from = waypoints[i]->as<ompl::base::AtlasStateSpace::StateType>();
            to = waypoints[i+1]->as<ompl::base::AtlasStateSpace::StateType>();
            
            // Traverse the manifold
            std::vector<ompl::base::AtlasStateSpace::StateType *> stateList;
            atlas->followManifold(from, to, true, &stateList);
            if (!atlas->equalStates(stateList.front(), stateList.back()))
            {
                // Print the intermediate states
                for (std::size_t i = 1; i < stateList.size(); i++)
                {
                    printState(stateList[i]);
                    const Eigen::VectorXd x = stateList[i]->toVector();
                    v << x[0] << " " << x[1] << " " << x[2] << "\n";
                    vcount++;
                    length += atlas->distance(stateList[i-1], stateList[i]);
                }
            }
            
            // Delete the intermediate states
            for (std::size_t i = 0; i < stateList.size(); i++)
                atlas->freeState(stateList[i]);
        }
        std::ofstream pathFile("path.ply");
        pathFile << "ply\n";
        pathFile << "format ascii 1.0\n";
        pathFile << "element vertex " << vcount << "\n";
        pathFile << "property float x\n";
        pathFile << "property float y\n";
        pathFile << "property float z\n";
        pathFile << "end_header\n";
        pathFile << v.str();
        pathFile.close();
        std::cout << "-----\n";
        std::cout << "Length: " << length << "\n";
        std::cout << "Took " << time << " seconds.\n";
    }
    else
    {
        std::cout << "No solution found.\n";
    }
    
    std::cout << "Atlas created " << atlas->getChartCount() << " charts.\n";
    
    std::ofstream atlasFile("atlas.ply");
    atlas->dumpMesh(atlasFile);
    atlasFile.close();
    
    return 0;
}
