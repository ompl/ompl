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
#include <ompl/base/spaces/AtlasConstraint.h>
#include <ompl/base/spaces/AtlasStateSpace.h>
#include <ompl/geometric/ConstrainedSimpleSetup.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/planners/est/EST.h>
#include <ompl/geometric/planners/kpiece/BKPIECE1.h>
#include <ompl/geometric/planners/kpiece/LBKPIECE1.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/pdst/PDST.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/prm/SPARS.h>
#include <ompl/geometric/planners/prm/SPARStwo.h>
#include <ompl/geometric/planners/rrt/CBiRRT2.h>
#include <ompl/geometric/planners/rrt/ConstrainedRRT.h>
#include <ompl/geometric/planners/rrt/LazyRRT.h>
#include <ompl/geometric/planners/rrt/LBTRRT.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/TRRT.h>
#include <ompl/geometric/planners/sbl/SBL.h>
#include <ompl/geometric/planners/stride/STRIDE.h>

#include <eigen3/Eigen/Dense>

#include <png++/png.hpp>

/** Simple manifold example: the unit sphere. */
class SphereManifold : public ompl::base::AtlasStateSpace
{
public:
    
    SphereManifold ()
    : ompl::base::AtlasStateSpace(3, 2)
    {
    }
    
    void bigF (const Eigen::VectorXd &x, Eigen::Ref<Eigen::VectorXd> out) const
    {
        out[0] = x.norm() - 1;
    }

    void bigJ (const Eigen::VectorXd &x, Eigen::Ref<Eigen::MatrixXd> out) const
    {
        out = x.transpose().normalized();
    }
};

/** Simple manifold example: the xy plane. */
class PlaneManifold : public ompl::base::AtlasStateSpace
{
public:
    
    PlaneManifold ()
    : ompl::base::AtlasStateSpace(3, 2)
    {
    }
        
    void bigF (const Eigen::VectorXd &x, Eigen::Ref<Eigen::VectorXd> out) const
    {
        out[0] = x[3];
    }

    void bigJ (const Eigen::VectorXd &x, Eigen::Ref<Eigen::MatrixXd> out) const
    {
        out(0,0) = 0;
        out(0,1) = 0;
        out(0,2) = 1;
    }
};

/** Klein bottle embedded in R^3 manifold. (Self-intersecting -> nasty chart breakdown.) */
class KleinManifold : public ompl::base::AtlasStateSpace
{
public:
    
    KleinManifold ()
    : ompl::base::AtlasStateSpace(3, 2)
    {
    }
        
    void bigF (const Eigen::VectorXd &x, Eigen::Ref<Eigen::VectorXd> out) const
    {
        const double p = x.squaredNorm() + 2*x[1] - 1;
        const double n = x.squaredNorm() - 2*x[1] - 1;
        const double u = n*n - 8*x[2]*x[2];
        
        out[0] = p*u + 16*x[0]*x[1]*n;
    }

    void bigJ (const Eigen::VectorXd &x, Eigen::Ref<Eigen::MatrixXd> out) const
    {
        const double p = x.squaredNorm() + 2*x[1] - 1;
        const double n = x.squaredNorm() - 2*x[1] - 1;
        const double u = n*n - 8*x[2]*x[2];
        
        out(0,0) = 32*x[0]*x[0]*x[1] + 16*x[1]*n + 4*x[0]*n*p + 2*x[0]*u;
        out(0,1) = 32*x[0]*x[1]*(x[1]-1) + 16*x[0]*n + 4*(x[1]-1)*n*p + 2*(x[1]+1)*u;
        out(0,2) = 2*x[2]*(16*x[0]*x[1] + 2*p*(n-4) + u);
    }
};

/** Torus manifold. */
#define TORUSR1 2.0
#define TORUSR2 1.0
class TorusManifold : public ompl::base::AtlasStateSpace
{
public:
    
    TorusManifold ()
    : ompl::base::AtlasStateSpace(3, 2)
    {
    }
    
    void bigF (const Eigen::VectorXd &x, Eigen::Ref<Eigen::VectorXd> out) const
    {
        Eigen::VectorXd c(3); c << x[0], x[1], 0;
        out[0] = (x - TORUSR1 * c.normalized()).norm() - TORUSR2;
    }

    void bigJ (const Eigen::VectorXd &x, Eigen::Ref<Eigen::MatrixXd> out) const
    {
        const double xySquaredNorm = x[0]*x[0] + x[1]*x[1];
        const double xyNorm = std::sqrt(xySquaredNorm);
        const double denom = std::sqrt(x[2]*x[2] + (xyNorm - TORUSR1)*(xyNorm - TORUSR1));
        const double c = (xyNorm - TORUSR1) * (xyNorm*xySquaredNorm) / (xySquaredNorm * xySquaredNorm * denom);
        out(0,0) = x[0] * c;
        out(0,1) = x[1] * c;
        out(0,2) = x[2] / denom;
    }
};

/** Kinematic chain manifold. 5 links in 3D space. */
#define CHAINDIM            3
#define CHAINLINKS          5
#define CHAINJOINTWIDTH     0.2
#define CHAINLINKLENGTH     1.0
#define CHAINEFFECTORRADIUS 3.0
class ChainManifold : public ompl::base::AtlasStateSpace
{
public:
    
    ChainManifold ()
    : ompl::base::AtlasStateSpace(CHAINDIM*CHAINLINKS, (CHAINDIM-1)*CHAINLINKS - 1)
    {
    }
    
    void bigF (const Eigen::VectorXd &x, Eigen::Ref<Eigen::VectorXd> out) const
    {
        // Consecutive joints must be a fixed distance apart
        Eigen::VectorXd joint1 = Eigen::VectorXd::Zero(CHAINDIM);
        for (std::size_t i = 0; i < CHAINLINKS; i++)
        {
            const Eigen::VectorXd joint2 = x.segment(CHAINDIM*i, CHAINDIM);
            out[i] = (joint1 - joint2).norm() - CHAINLINKLENGTH;
            joint1 = joint2;
        }
        
        // End effector must lie on a sphere
        out[CHAINLINKS] = x.tail(CHAINDIM).norm() - CHAINEFFECTORRADIUS;
    }

    void bigJ (const Eigen::VectorXd &x, Eigen::Ref<Eigen::MatrixXd> out) const
    {
        Eigen::VectorXd plus(CHAINDIM*(CHAINLINKS+1)); plus.head(CHAINDIM*CHAINLINKS) = x; plus.tail(CHAINDIM) = Eigen::VectorXd::Zero(CHAINDIM);
        Eigen::VectorXd minus(CHAINDIM*(CHAINLINKS+1)); minus.head(CHAINDIM) = Eigen::VectorXd::Zero(CHAINDIM); minus.tail(CHAINDIM*CHAINLINKS) = x;
        const Eigen::VectorXd diagonal = plus - minus;
        for (std::size_t i = 0; i < CHAINLINKS; i++)
            out.row(i).segment(CHAINDIM*i, CHAINDIM) = diagonal.segment(CHAINDIM*i, CHAINDIM).normalized();
        out.block(1, 0, CHAINLINKS, CHAINDIM*(CHAINLINKS-1)) -= out.block(1, CHAINDIM, CHAINLINKS, CHAINDIM*(CHAINLINKS-1));
        out.row(CHAINLINKS).tail(CHAINDIM) = -diagonal.tail(CHAINDIM).normalized().transpose();
    }
};

/**
 * State validity checking functions implicitly define the free space where they return true.
 */

/** 3 ring-shaped obstacles on latitudinal lines, with a small gap in each. */
bool sphereValid_helper (const Eigen::VectorXd &x)
{
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

/** Correct path on the sphere must snake around. */
bool sphereValid (const ompl::base::State *state)
{
    return sphereValid_helper(state->as<ompl::base::AtlasStateSpace::StateType>()->constVectorView());
}

/** Every state is valid. */
bool always (const ompl::base::State *)
{
    return true;
}

/** States surrounding the goal are invalid, making it unreachable. We can use this to build up an atlas
 * until time runs out, so we can see the big picture. */
bool unreachable (const ompl::base::State *state, const Eigen::VectorXd &goal, const double radius)
{
    return std::abs((state->as<ompl::base::AtlasStateSpace::StateType>()->constVectorView() - goal).norm() - radius) > radius-0.01;
}

/** Maze-like obstacle in the xy plane. */
bool mazePlaneValid (png::image<png::index_pixel_1> &maze, const ompl::base::State *state)
{
    const ompl::base::AtlasStateSpace::StateType *astate = state->as<ompl::base::AtlasStateSpace::StateType>();
    Eigen::VectorXd vec = astate->constVectorView();
    vec[0] *= 0.2*maze.get_width();
    vec[1] *= 0.2*maze.get_height();
    if (vec[0] < 0 || vec[0] >= maze.get_width() || vec[1] < 0 || vec[1] >= maze.get_height())
        return false;
    return !maze.get_pixel(vec[0], vec[1]).operator png::byte ();
}

/** Maze-like obstacle on a torus plane. */
bool mazeTorusValid (png::image<png::index_pixel_1> &maze, const ompl::base::State *state)
{
    const ompl::base::AtlasStateSpace::StateType *astate = state->as<ompl::base::AtlasStateSpace::StateType>();
    Eigen::Ref<const Eigen::VectorXd> p = astate->constVectorView();
    Eigen::VectorXd vec(2);
    Eigen::VectorXd c(3); c << p[0], p[1], 0;
    vec[0] = maze.get_width()*std::atan2(p[1], p[0])/(2*M_PI);
    vec[0] += 0.5*(vec[0] < 0);
    vec[1] = maze.get_height()*std::atan2(p[2], c.norm()-TORUSR1)/(2*M_PI);
    vec[1] += 0.5*(vec[1] < 0);
    if (vec[0] < 0 || vec[0] >= maze.get_width() || vec[1] < 0 || vec[1] >= maze.get_height())
        return false;
    return !maze.get_pixel(vec[0], vec[1]).operator png::byte ();
}

/** For the chain example. Joints may not get too close to each other. If \a tough, then the end effector
 * may not occupy states similar to the sphereValid() obstacles (but rotated and scaled). */
bool chainValid (const ompl::base::State *state, const bool tough)
{
    Eigen::Ref<const Eigen::VectorXd> x = state->as<ompl::base::AtlasStateSpace::StateType>()->constVectorView();
    for (std::size_t i = 0; i < CHAINLINKS-1; i++)
    {
        if (x.segment(CHAINDIM*i, CHAINDIM).cwiseAbs().maxCoeff() < CHAINJOINTWIDTH)
            return false;
        for (std::size_t j = i+1; j < CHAINLINKS; j++)
        {
            if ((x.segment(CHAINDIM*i, CHAINDIM) - x.segment(CHAINDIM*j, CHAINDIM)).cwiseAbs().maxCoeff() < CHAINJOINTWIDTH)
                return false;
        }
    }
    
    if (!tough)
        return true;
    
    Eigen::VectorXd end = x.tail(CHAINDIM)/CHAINEFFECTORRADIUS;
    const double tmp = end[0];
    end[0] = end[2];
    end[2] = tmp;
    return sphereValid_helper(end);
}

/**
 * Problem initialization functions set the dimension, the manifold, start and goal points \a x and \a y,
 * and the validity checker \a isValid.
 */

/** Initialize the atlas for the sphere problem and store the start and goal vectors. */
ompl::base::AtlasStateSpace *initSphereProblem (Eigen::VectorXd &x, Eigen::VectorXd &y, ompl::base::StateValidityCheckerFn &isValid)
{
    const std::size_t dim = 3;
    
    // Start and goal points
    x = Eigen::VectorXd(dim); x << 0, 0, -1;
    y = Eigen::VectorXd(dim); y << 0, 0,  1;
    
    // Validity checker
    isValid = &sphereValid;
    
    // Atlas initialization (can use numerical methods to compute the Jacobian, but giving an explicit function is faster)
    return new SphereManifold();
}

/** Initialize the atlas for the torus problem and store the start and goal vectors. */
ompl::base::AtlasStateSpace *initTorusProblem (Eigen::VectorXd &x, Eigen::VectorXd &y, ompl::base::StateValidityCheckerFn &isValid)
{
    const std::size_t dim = 3;
    
    // Start and goal points
    x = Eigen::VectorXd(dim); x << -2, 0, -1;
    y = Eigen::VectorXd(dim); y <<  2, 0,  1;
    
    // Validity checker
    isValid = boost::bind(&unreachable, _1, y, 0.1);
    
    return new TorusManifold();
}

/** Initialize the atlas for the sphere problem and store the start and goal vectors. */
ompl::base::AtlasStateSpace *initKleinBottleProblem (Eigen::VectorXd &x, Eigen::VectorXd &y, ompl::base::StateValidityCheckerFn &isValid)
{
    const std::size_t dim = 3;
    
    // Start and goal points
    x = Eigen::VectorXd(dim); x << -0.5, -0.25, 0.1892222244330081;
    y = Eigen::VectorXd(dim); y <<  2.5, -1.5,  1.0221854181962458;
    
    // Validity checker
    isValid = boost::bind(&unreachable, _1, y, 0.2);
    
    return new KleinManifold();
}

/** Initialize the atlas for the kinematic chain problem. */
ompl::base::AtlasStateSpace *initChainProblem (Eigen::VectorXd &x, Eigen::VectorXd &y, ompl::base::StateValidityCheckerFn &isValid, const bool tough)
{
    const std::size_t dim = CHAINDIM*CHAINLINKS;
    
    // Start and goal points (each triple is the 3D location of a joint)
    x = Eigen::VectorXd(dim); x << 1,  0, 0,  2,  0, 0,  2, -1, 0,  3, -1, 0,  3, 0, 0;
    y = Eigen::VectorXd(dim); y << 0, -1, 0, -1, -1, 0, -1,  0, 0, -2,  0, 0, -3, 0, 0;
    
    // Validity checker
    isValid = boost::bind(&chainValid, _1, tough);
    
    return new ChainManifold();
}

/** Initialize the atlas for the planar maze problem. */
ompl::base::AtlasStateSpace *initPlanarMazeProblem (Eigen::VectorXd &x, Eigen::VectorXd &y, ompl::base::StateValidityCheckerFn &isValid, const char *filename)
{
    const std::size_t dim = 3;
    
    // Start and goal points
    x = Eigen::VectorXd(dim); x << 2.25, 0.1, 0;
    y = Eigen::VectorXd(dim); y << 4.9, 4.45, 0;
    
    // Load maze (memory leak!)
    png::image<png::index_pixel_1> *img = new png::image<png::index_pixel_1>(filename, png::require_color_space<png::index_pixel_1>());
    
    // Validity checker
    isValid = boost::bind(&mazePlaneValid, *img, _1);
    
    return new PlaneManifold();
}

/** Initialize the atlas for the torus maze problem. */
ompl::base::AtlasStateSpace *initTorusMazeProblem (Eigen::VectorXd &x, Eigen::VectorXd &y, ompl::base::StateValidityCheckerFn &isValid, const char *filename)
{
    const std::size_t dim = 3;
    
    // Start and goal points
    x = Eigen::VectorXd(dim);
    y = Eigen::VectorXd(dim);
    Eigen::VectorXd xA(2); xA << 0.45, 0.02;
    Eigen::VectorXd yA(2); yA << 0.98, 0.89;
    Eigen::VectorXd xB(3);
    Eigen::VectorXd yB(3);
    xA *= 2*M_PI;
    yA *= 2*M_PI;
    xB << std::cos(xA[0]), 0, std::sin(xA[0]);
    yB << std::cos(yA[0]), 0, std::sin(yA[0]);
    xB *= TORUSR2;
    yB *= TORUSR2;
    xB[0] += TORUSR1;
    yB[0] += TORUSR1;
    double nX = std::sqrt(xB[0]*xB[0] + xB[1]*xB[1]);
    double nY = std::sqrt(yB[0]*yB[0] + yB[1]*yB[1]);
    x << std::cos(xA[1]), std::sin(xA[1]), 0;
    y << std::cos(yA[1]), std::sin(yA[1]), 0;
    x *= nX;
    y *= nY;
    x[2] = xB[2];
    y[2] = yB[2];
    
    // Load maze (memory leak!)
    png::image<png::index_pixel_1> *img = new png::image<png::index_pixel_1>(filename, png::require_color_space<png::index_pixel_1>());
    
    // Validity checker
    isValid = boost::bind(&mazeTorusValid, *img, _1);
    
    return new TorusManifold();
}

/** Allocator function for a sampler for the atlas that only returns valid points. */
ompl::base::ValidStateSamplerPtr vssa (const ompl::base::AtlasStateSpacePtr &atlas, const ompl::base::SpaceInformation *si)
{
    return ompl::base::ValidStateSamplerPtr(new ompl::base::AtlasValidStateSampler(atlas, si));
}

/** Print usage information. */
void printProblems (void)
{
    std::cout << "Available problems:\n";
    std::cout << "    sphere torus klein chain chain_tough planar_maze\n";
}

/** Print usage information. */
void printPlanners (void)
{
    std::cout << "Available planners:\n";
    std::cout << "    EST RRT AtlasRRT RRTConnect RRTstar LazyRRT TRRT LBTRRT \n";
    std::cout << "    ConstrainedRRT CBiRRT2 KPIECE1 BKPIECE1 LBKPIECE1 PDST\n";
    std::cout << "    PRM PRMstar SBL SPARS SPARStwo STRIDE\n";
}

/** Initialize the problem specified in the string. */
ompl::base::AtlasStateSpace *parseProblem (const char *const problem, Eigen::VectorXd &x, Eigen::VectorXd &y,
                                           ompl::base::StateValidityCheckerFn &isValid)
{
    if (std::strcmp(problem, "sphere") == 0)
        return initSphereProblem(x, y, isValid);
    else if (std::strcmp(problem, "torus") == 0)
        return initTorusProblem(x, y, isValid);
    else if (std::strcmp(problem, "klein") == 0)
        return initKleinBottleProblem(x, y, isValid);
    else if (std::strcmp(problem, "chain") == 0)
        return initChainProblem(x, y, isValid, false);
    else if (std::strcmp(problem, "chain_tough") == 0)
        return initChainProblem(x, y, isValid, true);
    else if (std::strcmp(problem, "planar_maze") == 0)
        return initPlanarMazeProblem(x, y, isValid, "maze.png");
    else
        return NULL;
}

/** Initialize the planner specified in the string. */
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
        return rrt;
    }
    else if (std::strcmp(planner, "AtlasRRT") == 0)
    {
        ompl::geometric::RRT *atlasrrt = new ompl::geometric::RRT(si);
        atlasrrt->setName("AtlasRRT");
        atlasrrt->setIntermediateStates(true);
        return atlasrrt;
    }
    else if (std::strcmp(planner, "RRTConnect") == 0)
    {
        ompl::geometric::RRTConnect *rrtconnect = new ompl::geometric::RRTConnect(si);
        return rrtconnect;
    }
    else if (std::strcmp(planner, "RRTstar") == 0)
    {
        ompl::geometric::RRTstar *rrtstar = new ompl::geometric::RRTstar(si);
        return rrtstar;
    }
    else if (std::strcmp(planner, "LazyRRT") == 0)
    {
        ompl::geometric::LazyRRT *lazyrrt = new ompl::geometric::LazyRRT(si);
        return lazyrrt;
    }
    else if (std::strcmp(planner, "TRRT") == 0)
    {
        ompl::geometric::TRRT *trrt = new ompl::geometric::TRRT(si);
        return trrt;
    }
    else if (std::strcmp(planner, "LBTRRT") == 0)
    {
        ompl::geometric::LBTRRT *lbtrrt = new ompl::geometric::LBTRRT(si);
        return lbtrrt;
    }
    else if (std::strcmp(planner, "ConstrainedRRT") == 0)
    {
        ompl::geometric::ConstrainedRRT *constrainedrrt = new ompl::geometric::ConstrainedRRT(si);
        return constrainedrrt;
    }
    else if (std::strcmp(planner, "CBiRRT2") == 0)
    {
        ompl::geometric::CBiRRT2 *cbirrt2 = new ompl::geometric::CBiRRT2(si);
        return cbirrt2;
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
    else if (std::strcmp(planner, "SBL") == 0)
    {
        ompl::geometric::SBL *sbl = new ompl::geometric::SBL(si);
        sbl->setRange(range);
        return sbl;
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
        return NULL;
}
