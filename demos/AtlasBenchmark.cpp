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
#include <ompl/geometric/planners/prm/SPARS.h>
#include <ompl/geometric/planners/prm/SPARStwo.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/tools/benchmark/Benchmark.h>

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

/** Torus manifold. */
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

/** Initialize the atlas for the torus problem and store the start and goal vectors. */
ompl::base::AtlasStateSpace *initTorusProblem (Eigen::VectorXd &x, Eigen::VectorXd &y, ompl::base::StateValidityCheckerFn &isValid)
{
    const std::size_t dim = 3;
    
    // Start and goal points;
    x = Eigen::VectorXd(dim); x << -3, 0, 0;
    y = Eigen::VectorXd(dim); y << 3, 0, 0;
    
    // Validity checker
    isValid = &always;
    
    // Atlas initialization
    return new ompl::base::AtlasStateSpace(dim, Ftorus, Jtorus);
}

void resetStateSpace (const ompl::base::PlannerPtr &planner)
{
    planner->getSpaceInformation()->getStateSpace()->as<ompl::base::AtlasStateSpace>()->clear();
}

int main (int, char *[])
{
    // Initialize the atlas for a problem
    Eigen::VectorXd x, y;
    ompl::base::StateValidityCheckerFn isValid;
    ompl::base::AtlasStateSpacePtr atlas(initTorusProblem(x, y, isValid));
    ompl::base::StateSpacePtr space(atlas);
    ompl::geometric::SimpleSetup ss(space);
    ompl::base::SpaceInformationPtr si = ss.getSpaceInformation();
    atlas->setSpaceInformation(si);
    ss.setStateValidityChecker(isValid);
    const ompl::base::AtlasChart &startChart = atlas->anchorChart(x);
    const ompl::base::AtlasChart &goalChart = atlas->anchorChart(y);
    ompl::base::ScopedState<> start(space);
    ompl::base::ScopedState<> goal(space);
    start->as<ompl::base::AtlasStateSpace::StateType>()->setRealState(x, startChart);
    goal->as<ompl::base::AtlasStateSpace::StateType>()->setRealState(y, goalChart);
    ss.getProblemDefinition()->setStartAndGoalStates(start, goal);
    
    // Bounds
    ompl::base::RealVectorBounds bounds(atlas->getAmbientDimension());
    bounds.setLow(-10);
    bounds.setHigh(10);
    atlas->setBounds(bounds);
    
    // Parameter tuning
    atlas->setExploration(0.8);
    atlas->setRho(0.1);
    atlas->setAlpha(M_PI/32);
    atlas->setEpsilon(0.05);
    atlas->setDelta(0.01);
    
    // Benchmark the planners
    ompl::tools::Benchmark bench(ss, "Atlas");
    const double runtime_limit = 120;
    const double memory_limit = 1024;
    const int run_count = 20;
    const ompl::tools::Benchmark::Request request(runtime_limit, memory_limit, run_count);
    bench.addPlanner(ompl::base::PlannerPtr(new ompl::geometric::RRT(si)));
    bench.addPlanner(ompl::base::PlannerPtr(new ompl::geometric::RRTConnect(si)));
    bench.addPlanner(ompl::base::PlannerPtr(new ompl::geometric::EST(si)));
    bench.addPlanner(ompl::base::PlannerPtr(new ompl::geometric::KPIECE1(si)));
    bench.addPlanner(ompl::base::PlannerPtr(new ompl::geometric::PRM(si)));
    bench.setPreRunEvent(&resetStateSpace);
    
    bench.benchmark(request);
    bench.saveResultsToFile("atlas.log");
    
    return 0;
}
