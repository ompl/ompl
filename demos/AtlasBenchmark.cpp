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

#include <fstream>

#include <ompl/base/ScopedState.h>
#include <ompl/base/spaces/AtlasChart.h>
#include <ompl/base/spaces/AtlasStateSpace.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/est/EST.h>
#include <ompl/geometric/planners/kpiece/BKPIECE1.h>
#include <ompl/geometric/planners/kpiece/LBKPIECE1.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/pdst/PDST.h>
#include <ompl/geometric/planners/prm/LazyPRM.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/prm/SPARS.h>
#include <ompl/geometric/planners/prm/SPARStwo.h>
#include <ompl/geometric/planners/rrt/LazyRRT.h>
#include <ompl/geometric/planners/rrt/LBTRRT.h>
#include <ompl/geometric/planners/rrt/pRRT.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/TRRT.h>
#include <ompl/geometric/planners/sbl/pSBL.h>
#include <ompl/geometric/planners/sbl/SBL.h>
#include <ompl/geometric/planners/stride/STRIDE.h>
#include <ompl/tools/benchmark/Benchmark.h>

#include <eigen3/Eigen/Dense>

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

/** Klein bottle manifold. */
Eigen::VectorXd FKleinBottle (const Eigen::VectorXd &x)
{
    const double p = x.squaredNorm() + 2*x[1] - 1;
    const double n = x.squaredNorm() - 2*x[1] - 1;
    const double u = n*n - 8*x[2]*x[2];
    
    Eigen::VectorXd f(1);
    f[0] = p*u + 16*x[0]*x[1]*n;
    
    return f;
}

/** Jacobian of FKleinBottle(x). */
Eigen::MatrixXd JKleinBottle (const Eigen::VectorXd &x)
{
    const double p = x.squaredNorm() + 2*x[1] - 1;
    const double n = x.squaredNorm() - 2*x[1] - 1;
    const double u = n*n - 8*x[2]*x[2];
    
    Eigen::MatrixXd j(1,3);
    j(0,0) = 32*x[0]*x[0]*x[1] + 16*x[1]*n + 4*x[0]*n*p + 2*x[0]*u;
    j(0,1) = 32*x[0]*x[1]*(x[1]-1) + 16*x[0]*n + 4*(x[1]-1)*n*p + 2*(x[1]+1)*u;
    j(0,2) = 2*x[2]*(16*x[0]*x[1] + 2*p*(n-4) + u);
    
    return j;
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

/** Jacobian of Ftorus(x). */
Eigen::MatrixXd Jtorus (const Eigen::VectorXd &x)
{
    Eigen::MatrixXd j(1,3);
    const double r1 = 2;
    const double xySquaredNorm = x[0]*x[0] + x[1]*x[1];
    const double xyNorm = std::sqrt(xySquaredNorm);
    const double denom = std::sqrt(x[2]*x[2] + (xyNorm - r1)*(xyNorm - r1));
    const double c = (xyNorm - r1) * (xyNorm*xySquaredNorm) / (xySquaredNorm * xySquaredNorm * denom);
    j(0,0) = x[0] * c;
    j(0,1) = x[1] * c;
    j(0,2) = x[2] / denom;
    
    return j;
}

#define CHAINDIM    3
#define CHAINLINKS  5
/** Kinematic chain manifold. 5 links in 3D space. */
Eigen::VectorXd Fchain (const Eigen::VectorXd &x)
{
    Eigen::VectorXd f(CHAINLINKS+1);
    
    // Consecutive joints must be a fixed distance apart
    const double linkLength = 1;
    Eigen::VectorXd joint1 = Eigen::VectorXd::Zero(CHAINDIM);
    for (std::size_t i = 0; i < CHAINLINKS; i++)
    {
        const Eigen::VectorXd joint2 = x.segment(CHAINDIM*i, CHAINDIM);
        f[i] = (joint1 - joint2).norm() - linkLength;
        
        joint1 = joint2;
    }
    
    // End effector must lie on a sphere of radius 3
    f[CHAINLINKS] = x.tail(CHAINDIM).norm() - 3;
    
    return f;
}

/** Jacobian of Fchain(x). */
Eigen::MatrixXd Jchain (const Eigen::VectorXd &x)
{
    Eigen::MatrixXd j = Eigen::MatrixXd::Zero(CHAINLINKS+1, CHAINDIM*CHAINLINKS);
    Eigen::VectorXd plus(CHAINDIM*(CHAINLINKS+1)); plus.head(CHAINDIM*CHAINLINKS) = x; plus.tail(CHAINDIM) = Eigen::VectorXd::Zero(CHAINDIM);
    Eigen::VectorXd minus(CHAINDIM*(CHAINLINKS+1)); minus.head(CHAINDIM) = Eigen::VectorXd::Zero(CHAINDIM); minus.tail(CHAINDIM*CHAINLINKS) = x;
    const Eigen::VectorXd diagonal = plus - minus;
    for (std::size_t i = 0; i < CHAINLINKS; i++)
        j.row(i).segment(CHAINDIM*i, CHAINDIM) = diagonal.segment(CHAINDIM*i, CHAINDIM).normalized();
    j.block(1, 0, CHAINLINKS, CHAINDIM*(CHAINLINKS-1)) -= j.block(1, CHAINDIM, CHAINLINKS, CHAINDIM*(CHAINLINKS-1));
    j.row(CHAINLINKS).tail(CHAINDIM) = -diagonal.tail(CHAINDIM).normalized().transpose();
    
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

/** For the chain example. Joints may not get too close to each other. */
bool noIntersect (const ompl::base::State *state)
{
    const Eigen::VectorXd x = state->as<ompl::base::AtlasStateSpace::StateType>()->toVector();
    for (std::size_t i = 0; i < CHAINLINKS-1; i++)
    {
        if (x.segment(CHAINDIM*i, CHAINDIM).cwiseAbs().maxCoeff() < 0.2)
            return false;
        for (std::size_t j = i+1; j < CHAINLINKS; j++)
        {
            if ((x.segment(CHAINDIM*i, CHAINDIM) - x.segment(CHAINDIM*j, CHAINDIM)).cwiseAbs().maxCoeff() < 0.2)
                return false;
        }
    }
    
    return true;
}

/** Initialize the atlas for the sphere problem and store the start and goal vectors. */
ompl::base::AtlasStateSpace *initSphereProblem (Eigen::VectorXd &x, Eigen::VectorXd &y,
                                                ompl::base::StateValidityCheckerFn &isValid, double &plannerRange)
{
    plannerRange = 0.75;
    const std::size_t dim = 3;
    
    // Start and goal points
    x = Eigen::VectorXd(dim); x << 0, 0, -1;
    y = Eigen::VectorXd(dim); y << 0, 0, 1;
    
    // Validity checker
    isValid = &sphereValid;
    
    // Atlas initialization (can use numerical methods to compute the Jacobian, but giving an explicit function is faster)
    return new ompl::base::AtlasStateSpace(dim, Fsphere, Jsphere);
}

/** Initialize the atlas for the sphere problem and store the start and goal vectors. */
ompl::base::AtlasStateSpace *initKleinBottleProblem (Eigen::VectorXd &x, Eigen::VectorXd &y,
                                                     ompl::base::StateValidityCheckerFn &isValid, double &plannerRange)
{
    plannerRange = 1.5;
    const std::size_t dim = 3;
    
    // Start and goal points
    x = Eigen::VectorXd(dim); x << -0.5, -0.25, 0.1892222244330081;
    y = Eigen::VectorXd(dim); y << 2.5, -1.5, 1.0221854181962458;
    
    // Validity checker
    isValid = boost::bind(&unreachable, _1, y);
    
    // Atlas initialization
    return new ompl::base::AtlasStateSpace(dim, FKleinBottle, JKleinBottle);
}

/** Initialize the atlas for the torus problem and store the start and goal vectors. */
ompl::base::AtlasStateSpace *initTorusProblem (Eigen::VectorXd &x, Eigen::VectorXd &y,
                                               ompl::base::StateValidityCheckerFn &isValid, double &plannerRange)
{
    plannerRange = 1;
    const std::size_t dim = 3;
    
    // Start and goal points
    x = Eigen::VectorXd(dim); x << -2, 0, -1;
    y = Eigen::VectorXd(dim); y << 2, 0, 1;
    
    // Validity checker
    isValid = &always;
    
    // Atlas initialization
    return new ompl::base::AtlasStateSpace(dim, Ftorus, Jtorus);
}

/** Initialize the atlas for the kinematic chain problem. */
ompl::base::AtlasStateSpace *initChainProblem (Eigen::VectorXd &x, Eigen::VectorXd &y,
                                               ompl::base::StateValidityCheckerFn &isValid, double &plannerRange)
{
    plannerRange = 3;
    const std::size_t dim = 15;
    
    // Start and goal points
    x = Eigen::VectorXd(dim); x << 1, 0, 0, 2, 0, 0, 2, -1, 0, 3, -1, 0, 3, 0, 0;
    y = Eigen::VectorXd(dim); y << 0, -1, 0, -1, -1, 0, -1, 0, 0, -2, 0, 0, -3, 0, 0;
    
    // Validity checker
    isValid = &noIntersect;
    
    return new ompl::base::AtlasStateSpace(dim, Fchain, Jchain);
}

/** Allocate a sampler for the atlas that only returns valid points. */
ompl::base::ValidStateSamplerPtr vssa (const ompl::base::AtlasStateSpacePtr &atlas, const ompl::base::SpaceInformation *si)
{
    return ompl::base::ValidStateSamplerPtr(new ompl::base::AtlasValidStateSampler(atlas, si));
}

/** Print usage information. Does not return. */
void usage (void)
{
    std::cout << "Usage: demo_AtlasPlanning <problem> <timelimit>\n";
    std::cout << "Available problems:\n";
    std::cout << "    sphere torus klein chain\n";
    exit(0);
}

ompl::base::AtlasStateSpace *parseProblem (const char *const problem, Eigen::VectorXd &x, Eigen::VectorXd &y,
                                           ompl::base::StateValidityCheckerFn &isValid, double &plannerRange)
{
    if (std::strcmp(problem, "sphere") == 0)
        return initSphereProblem(x, y, isValid, plannerRange);
    else if (std::strcmp(problem, "torus") == 0)
        return initTorusProblem(x, y, isValid, plannerRange);
    else if (std::strcmp(problem, "klein") == 0)
        return initKleinBottleProblem(x, y, isValid, plannerRange);
    else if (std::strcmp(problem, "chain") == 0)
        return initChainProblem(x, y, isValid, plannerRange);
    else
        usage();
    
    std::abort();
}

ompl::base::Planner *parsePlanner (const char *const planner, const ompl::base::SpaceInformationPtr &si, const double range)
{
    if (std::strcmp(planner, "EST") == 0)
    {
        ompl::geometric::EST *est = new ompl::geometric::EST(si);
        est->setRange(range);
        return est;
    }
    else if (std::strcmp(planner, "RRT") == 0)
    {
        ompl::geometric::RRT *rrt = new ompl::geometric::RRT(si);
        rrt->setRange(range);
        return rrt;
    }
    else if (std::strcmp(planner, "AtlasRRT") == 0)
    {
        ompl::geometric::RRT *atlasrrt = new ompl::geometric::RRT(si);
        atlasrrt->setName("AtlasRRT");
        atlasrrt->setIntermediateStates(true);
        atlasrrt->setRange(range);
        return atlasrrt;
    }
    else if (std::strcmp(planner, "RRTConnect") == 0)
    {
        ompl::geometric::RRTConnect *rrtconnect = new ompl::geometric::RRTConnect(si);
        rrtconnect->setRange(range);
        return rrtconnect;
    }
    else if (std::strcmp(planner, "RRTstar") == 0)
    {
        ompl::geometric::RRTstar *rrtstar = new ompl::geometric::RRTstar(si);
        rrtstar->setRange(range);
        return rrtstar;
    }
    else if (std::strcmp(planner, "LazyRRT") == 0)
    {
        ompl::geometric::LazyRRT *lazyrrt = new ompl::geometric::LazyRRT(si);
        lazyrrt->setRange(range);
        return lazyrrt;
    }
    else if (std::strcmp(planner, "TRRT") == 0)
    {
        ompl::geometric::TRRT *trrt = new ompl::geometric::TRRT(si);
        trrt->setRange(range);
        return trrt;
    }
    else if (std::strcmp(planner, "LBTRRT") == 0)
    {
        ompl::geometric::LBTRRT *lbtrrt = new ompl::geometric::LBTRRT(si);
        lbtrrt->setRange(range);
        return lbtrrt;
    }
    else if (std::strcmp(planner, "pRRT0") > 0 && std::strcmp(planner, "pRRT9") <= 0)
    {
        ompl::geometric::pRRT *prrt = new ompl::geometric::pRRT(si);
        prrt->setRange(range);
        const unsigned int nthreads = std::atoi(&planner[4]);
        prrt->setThreadCount(nthreads);
        std::cout << "Using " << nthreads << " threads.\n";
        return prrt;
    }
    else if (std::strcmp(planner, "KPIECE1") == 0)
    {
        ompl::geometric::KPIECE1 *kpiece1 = new ompl::geometric::KPIECE1(si);
        kpiece1->setRange(range);
        return kpiece1;
    }
    else if (std::strcmp(planner, "BKPIECE1") == 0)
    {
        ompl::geometric::BKPIECE1 *bkpiece1 = new ompl::geometric::BKPIECE1(si);
        bkpiece1->setRange(range);
        return bkpiece1;
    }
    else if (std::strcmp(planner, "LBKPIECE1") == 0)
    {
        ompl::geometric::LBKPIECE1 *lbkpiece1 = new ompl::geometric::LBKPIECE1(si);
        lbkpiece1->setRange(range);
        return lbkpiece1;
    }
    else if (std::strcmp(planner, "PDST") == 0)
        return new ompl::geometric::PDST(si);
    else if (std::strcmp(planner, "PRM") == 0)
        return new ompl::geometric::PRM(si);
    else if (std::strcmp(planner, "PRMstar") == 0)
        return new ompl::geometric::PRMstar(si);
    else if (std::strcmp(planner, "LazyPRM") == 0)
        return new ompl::geometric::PRM(si);
    else if (std::strcmp(planner, "SBL") == 0)
    {
        ompl::geometric::SBL *sbl = new ompl::geometric::SBL(si);
        sbl->setRange(range);
        return sbl;
    }
    else if (std::strcmp(planner, "pSBL0") > 0 && std::strcmp(planner, "pSBL9") <= 0)
    {
        ompl::geometric::pSBL *psbl = new ompl::geometric::pSBL(si);
        psbl->setRange(range);
        const unsigned int nthreads = std::atoi(&planner[4]);
        psbl->setThreadCount(nthreads);
        std::cout << "Using " << nthreads << " threads.\n";
        return psbl;
    }
    else if (std::strcmp(planner, "SPARS") == 0)
        return new ompl::geometric::SPARS(si);
    else if (std::strcmp(planner, "SPARStwo") == 0)
        return new ompl::geometric::SPARStwo(si);
    else if (std::strcmp(planner, "STRIDE") == 0)
    {
        ompl::geometric::STRIDE *stride = new ompl::geometric::STRIDE(si);
        stride->setRange(range);
        return stride;
    }
    else
        usage();
    
    std::abort();
}

void resetStateSpace (const ompl::base::PlannerPtr &planner)
{
    planner->getSpaceInformation()->getStateSpace()->as<ompl::base::AtlasStateSpace>()->clear();
}

int main (int argc, char **argv)
{
    if (argc != 3)
        usage();
    
    // Initialize the atlas for a problem (you can try the other one too)
    Eigen::VectorXd x, y;
    ompl::base::StateValidityCheckerFn isValid;
    double plannerRange;
    ompl::base::AtlasStateSpacePtr atlas(parseProblem(argv[1], x, y, isValid, plannerRange));
    ompl::base::StateSpacePtr space(atlas);
    ompl::geometric::SimpleSetup ss(space);
    ompl::base::SpaceInformationPtr si = ss.getSpaceInformation();
    atlas->setSpaceInformation(si);
    ss.setStateValidityChecker(isValid);
    si->setValidStateSamplerAllocator(boost::bind(vssa, atlas, _1));
    const ompl::base::AtlasChart &startChart = atlas->anchorChart(x);
    const ompl::base::AtlasChart &goalChart = atlas->anchorChart(y);
    ompl::base::ScopedState<> start(space);
    ompl::base::ScopedState<> goal(space);
    start->as<ompl::base::AtlasStateSpace::StateType>()->setRealState(x, startChart);
    goal->as<ompl::base::AtlasStateSpace::StateType>()->setRealState(y, goalChart);
    ss.setStartAndGoalStates(start, goal);
    
    // Bounds
    ompl::base::RealVectorBounds bounds(atlas->getAmbientDimension());
    bounds.setLow(-10);
    bounds.setHigh(10);
    atlas->setBounds(bounds);
    
    // Atlas parameters
    atlas->setExploration(0.9);
    atlas->setRho(0.1);
    atlas->setAlpha(M_PI/32);
    atlas->setEpsilon(0.05);
    atlas->setDelta(0.01);
    atlas->setMaxChartsPerExtension(200);
    atlas->setMonteCarloSampleCount(0);
    
    // Benchmark the planners
    ompl::tools::Benchmark bench(ss, "Atlas");
    const double runtime_limit = std::atof(argv[2]);
    if (runtime_limit <= 0)
        usage();
    const double memory_limit = 1024;
    const int run_count = 5;
    const double update_interval = 0.1;
    const bool progress = true;
    const bool save_output = false;
    const bool use_threads = true;
    const ompl::tools::Benchmark::Request request(runtime_limit, memory_limit, run_count, update_interval, progress, save_output, use_threads);
    const char *planners[] = {"EST", "RRT", "AtlasRRT", "RRTConnect", "LazyRRT", "TRRT", "LBTRRT", "KPIECE1", "BKPIECE1", "LBKPIECE1",
                              "PDST", "PRM", "LazyPRM", "SBL", "SPARS", "SPARStwo", "STRIDE"};
    for (std::size_t i = 0; i < sizeof(planners)/sizeof(char *); i++)
        bench.addPlanner(ompl::base::PlannerPtr(parsePlanner(planners[i], si, plannerRange)));
    bench.setPreRunEvent(&resetStateSpace);
    
    bench.benchmark(request);
    bench.saveResultsToFile("atlas.log");
    
    return 0;
}
