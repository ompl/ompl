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
        out[0] = x[2];
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
class TorusManifold : public ompl::base::AtlasStateSpace
{
public:
    
    const double R1;
    const double R2;
    
    TorusManifold (double r1, double r2)
    : ompl::base::AtlasStateSpace(3, 2), R1(r1), R2(r2)
    {
    }
    
    void bigF (const Eigen::VectorXd &x, Eigen::Ref<Eigen::VectorXd> out) const
    {
        Eigen::VectorXd c(3); c << x[0], x[1], 0;
        out[0] = (x - R1 * c.normalized()).norm() - R2;
    }

    void bigJ (const Eigen::VectorXd &x, Eigen::Ref<Eigen::MatrixXd> out) const
    {
        const double xySquaredNorm = x[0]*x[0] + x[1]*x[1];
        const double xyNorm = std::sqrt(xySquaredNorm);
        const double denom = std::sqrt(x[2]*x[2] + (xyNorm - R1)*(xyNorm - R1));
        const double c = (xyNorm - R1) * (xyNorm*xySquaredNorm) / (xySquaredNorm * xySquaredNorm * denom);
        out(0,0) = x[0] * c;
        out(0,1) = x[1] * c;
        out(0,2) = x[2] / denom;
    }
};

bool sphereValid_helper (const Eigen::VectorXd &);

/** Kinematic chain manifold. */
class ChainManifold : public ompl::base::AtlasStateSpace
{
public:
    
    const unsigned int DIM;
    const unsigned int LINKS;
    const double LINKLENGTH;
    const double ENDEFFECTORRADIUS;
    const double JOINTWIDTH;
    
    ChainManifold (unsigned int dim, unsigned int links)
    : ompl::base::AtlasStateSpace(dim*links, (dim-1)*links - 1), DIM(dim), LINKS(links), LINKLENGTH(1), ENDEFFECTORRADIUS(3), JOINTWIDTH(0.2)
    {
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
        
        // End effector must lie on a sphere
        out[LINKS] = x.tail(DIM).norm() - ENDEFFECTORRADIUS;
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
        out.row(LINKS).tail(DIM) = -diagonal.tail(DIM).normalized().transpose();
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

/** Kinematic chain solving the torus maze. */
class ChainTorusManifold : public ompl::base::AtlasStateSpace
{
    inline double poly (const double &A, const double &B, const double &C, const double &D,
                        const double &E, const double &t) const
    {
        return (t == 0) ? E : (((A*t + B)*t + C)*t + D)*t + E;
    }

public:
    
    const double R1;
    const double R2;
    const unsigned int DIM;
    const unsigned int LINKS;
    const std::vector<double> LINKLENGTH;
    const double JOINTWIDTH;
    
    const png::image<png::index_pixel_1> maze;

    ChainTorusManifold (unsigned int links, std::vector<double> linklength, double r1, double r2)
        : ompl::base::AtlasStateSpace(3*links, (3-1)*links - 1), R1(r1), R2(r2), DIM(3),
          LINKS(links), LINKLENGTH(linklength), JOINTWIDTH(0.2),
          maze("../../demos/atlas/maze-wide.png", png::require_color_space<png::index_pixel_1>())
    {
    }
    
    void bigF (const Eigen::VectorXd &x, Eigen::Ref<Eigen::VectorXd> out) const
    {
        // Consecutive joints must be a fixed distance apart
        Eigen::VectorXd joint1 = Eigen::VectorXd::Zero(DIM);
        for (unsigned int i = 0; i < LINKS; i++)
        {
            const Eigen::VectorXd joint2 = x.segment(DIM*i, DIM);
            out[i] = (joint1 - joint2).norm() - LINKLENGTH[i];
            joint1 = joint2;
        }
        
        // End effector must lie on the torus
        Eigen::VectorXd c = x.tail(DIM);
        c[2] = 0;
        out[LINKS] = (x.tail(DIM) - R1 * c.normalized()).norm() - R2;
    }

    void bigJ (const Eigen::VectorXd &x, Eigen::Ref<Eigen::MatrixXd> out) const
    {
        out.setZero();
        Eigen::VectorXd plus(DIM*(LINKS+1));
        plus.head(DIM*LINKS) = x; plus.tail(DIM) = Eigen::VectorXd::Zero(DIM);
        Eigen::VectorXd minus(DIM*(LINKS+1));
        minus.head(DIM) = Eigen::VectorXd::Zero(DIM); minus.tail(DIM*LINKS) = x;
        const Eigen::VectorXd diagonal = plus - minus;
        for (unsigned int i = 0; i < LINKS; i++)
            out.row(i).segment(DIM*i, DIM) = diagonal.segment(DIM*i, DIM).normalized();
        out.block(1, 0, LINKS, DIM*(LINKS-1)) -= out.block(1, DIM, LINKS, DIM*(LINKS-1));
        
        // Torus part
        Eigen::VectorXd end = x.tail(DIM);
        const double xySquaredNorm = end[0]*end[0] + end[1]*end[1];
        const double xyNorm = std::sqrt(xySquaredNorm);
        const double denom = std::sqrt(end[2]*end[2] + (xyNorm - R1)*(xyNorm - R1));
        const double c = (xyNorm - R1) * (xyNorm*xySquaredNorm) /
            (xySquaredNorm * xySquaredNorm * denom);
        end[0] *= c; end[1] *= c; end[2] /= denom;
        out.row(LINKS).tail(DIM) = end;
    }
    
    /** 
     * Joints may not get too close to each other. The end effector must not touch the walls of the
     * maze.
     */
    bool isValid (double sleep, const ompl::base::State *state)
    {
        boost::this_thread::sleep(ompl::time::seconds(sleep));
        Eigen::Ref<const Eigen::VectorXd> x =
          state->as<ompl::base::AtlasStateSpace::StateType>()->constVectorView();
        // Check link lengths.
        for (unsigned int i = 0; i < LINKS; i++)
        {
            Eigen::VectorXd joint1;
            if (i == 0)
                joint1 = Eigen::VectorXd::Zero(DIM);
            else
                joint1 = x.segment(DIM*(i-1), DIM);

            Eigen::VectorXd joint2(x.segment(DIM*i, DIM));
            if (std::abs((joint1 - joint2).norm() - LINKLENGTH[i]) > 0.1)
                return false;
        }
        // Check joint proximity.
        /*for (unsigned int i = 0; i < LINKS-1; i++)
        {
            if (x.segment(DIM*i, DIM).cwiseAbs().maxCoeff() < JOINTWIDTH)
                return false;
            for (unsigned int j = i+1; j < LINKS; j++)
            {
                if ((x.segment(DIM*i, DIM) - x.segment(DIM*j, DIM)).cwiseAbs().maxCoeff()
                  < JOINTWIDTH)
                    return false;
            }
        }
        */
        

        // Check links and torus
        for (unsigned int i = 0; i < LINKS; i++)
        {
            Eigen::VectorXd P0, P1;
            if (i == 0)
                P0 = Eigen::VectorXd::Zero(DIM);
            else
                P0 = x.segment(DIM*(i-1), DIM);
            P1 = x.segment(DIM*i, DIM) - P0;

            // Does the line segment P0 + tP1 intersect the torus with 0 <= t <= 1?
            
            // Compute polynomial At^4 + Bt^3 + Ct^2 + Dt + E, whose roots are the intersections.
            double a, b, c, d, e, f, g;
            a = f = P1.dot(P1);
            a -= P1[2]*P1[2];
            b = e = 2*P0.dot(P1);
            b -= 2*P0[2]*P1[2];
            c = d = P0.dot(P0);
            c -= P0[2]*P0[2];
            d += R1*R1 - R2*R2;
            g = -4*R1*R1;
            const double A = f*f;
            const double B = 2*f*e;
            const double C = 2*f*d + e*e + g*a;
            const double D = 2*e*d + g*b;
            const double E = d*d + g*c;

            // Rather than compute t, we only need to know if 0 <= t <= 1. So instead we'll check
            // the signs of the critical points, which involves finding the roots of the derivative.
            const double A2 = 4*A;
            const double B2 = 3*B;
            const double C2 = 2*C;
            const double D2 = D;

            // Explicit formula for roots of cubic polynomial.
            const double discr0 = B2*B2 - 3*A2*C2;
            const double discr1 = 2*B2*B2*B2 - 9*A2*(B2*C2 - 3*A2*D2);
            const double sign = (discr1 > 0) ? 1 : -1;
            std::complex<double> z1 = discr1*discr1 - 4*discr0*discr0*discr0;
            z1 = std::pow((discr1 + sign * std::sqrt(z1))/2.0, 1.0/3);

            Eigen::VectorXd crit(5);
            crit[0] = 0; crit[4] = 1;
            if (std::abs(z1) < 1e-6)
            {
                // Special case, ignoring redundant roots.
                crit[3] = -B2/(3*A2);
                crit[1] = crit[2] = 0;
            }
            else
            {
                // General case, ignoring imaginary roots.
                std::complex<double> z2 = (-1.0/(3*A2)) * (B2 + z1 + discr0/z1);
                crit[1] = (std::abs(std::imag(z2)) < 1e-6) ? std::real(z2) : 0;

                std::complex<double> u(-1.0/2, std::sqrt(3.0)/2);
                z2 = (-1.0/(3*A2)) * (B2 + z1*u + discr0/(z1*u));
                crit[2] = (std::abs(std::imag(z2)) < 1e-6) ? std::real(z2) : 0;

                u = std::conj(u);
                z2 = (-1.0/(3*A2)) * (B2 + z1*u + discr0/(z1*u));
                crit[3] = (std::abs(std::imag(z2)) < 1e-6) ? std::real(z2) : 0;

                // Sort the roots.
                if (crit[1] > crit[2])
                    std::swap(crit[1], crit[2]);
                if (crit[2] > crit[3])
                    std::swap(crit[2], crit[3]);
                if (crit[1] > crit[2])
                    std::swap(crit[1], crit[2]);
            }
            // Cut off all values at [0+e,1-e].
            const double wiggleroom = JOINTWIDTH/2;
            crit = crit.cwiseMax(wiggleroom).cwiseMin(1-wiggleroom);

            // Intersection occurs if the polynomial changes sign between any pair of crits.
            crit[0] = poly(A,B,C,D,E, crit[0]);
            for (int j = 0; j < 4; j++)
            {
                crit[j+1] = poly(A,B,C,D,E, crit[j+1]);
                if (crit[j] * crit[j+1] <= 0)
                    return false;
            }
        }
            

        // Check maze
        Eigen::Ref<const Eigen::VectorXd> p = x.tail(DIM);
        Eigen::VectorXd vec(2);
        Eigen::VectorXd c(3); c << p[0], p[1], 0;
        vec[0] = std::atan2(p[2], c.norm()-R1)/(2*M_PI);
        vec[0] += (vec[0] < 0);
        vec[0] *= maze.get_height();
        vec[1] = std::atan2(p[1], p[0])/(2*M_PI);
        vec[1] += (vec[1] < 0);
        vec[1] *= maze.get_width();
        // I don't think this should ever happen...
        if (vec[0] < 0 || vec[0] >= maze.get_width() || vec[1] < 0 || vec[1] >= maze.get_height())
            return false;
        return !maze.get_pixel(vec[0], vec[1]).operator png::byte ();
    }
};


/**
 * State validity checking functions implicitly define the free space where they return true.
 */

/** 3 ring-shaped obstacles on latitudinal lines, with a small gap in each. */
bool sphereValid_helper (const Eigen::VectorXd &x)
{
    /*if (-0.75 < x[2] && x[2] < -0.65)
    {
        if (-1 < x[1] && x[1] < 1)
            return x[0] > 0;
        return false;
    }
    else*/ if (-0.5 < x[2] && x[2] < 0.5)
    {
        if (-1 < x[1] && x[1] < 1)
            return x[0] < 0;
        return false;
    }
    /*else if (0.65 < x[2] && x[2] < 0.75)
    {
        if (-1 < x[0] && x[0] < 1)
            return x[1] > 0;
        return false;
        }*/
    return true;
}

/** Correct path on the sphere must snake around. */
bool sphereValid (double sleep, const ompl::base::State *state)
{
    boost::this_thread::sleep(ompl::time::seconds(sleep));
    return sphereValid_helper(state->as<ompl::base::AtlasStateSpace::StateType>()->constVectorView());
}

/** Every state is valid. */
bool always (double sleep, const ompl::base::State *)
{
    boost::this_thread::sleep(ompl::time::seconds(sleep));
    return true;
}

/** States surrounding the goal are invalid, making it unreachable. We can use this to build up an atlas
 * until time runs out, so we can see the big picture. */
bool unreachable (double sleep, const ompl::base::State *state, const Eigen::VectorXd &goal, const double radius)
{
    boost::this_thread::sleep(ompl::time::seconds(sleep));
    return std::abs((state->as<ompl::base::AtlasStateSpace::StateType>()->constVectorView() - goal).norm() - radius) > radius-0.01;
}

/** Maze-like obstacle in the xy plane. */
bool mazePlaneValid (double sleep, png::image<png::index_pixel_1> &maze, const ompl::base::State *state)
{
    boost::this_thread::sleep(ompl::time::seconds(sleep));
    const ompl::base::AtlasStateSpace::StateType *astate = state->as<ompl::base::AtlasStateSpace::StateType>();
    Eigen::VectorXd vec = astate->constVectorView();
    // The factor of 1/7 is to preserve approximate length of the solution with the torus maze
    vec[0] *= maze.get_width()/7;
    vec[1] *= maze.get_height()/7;
    if (vec[0] < 0 || vec[0] >= maze.get_width() || vec[1] < 0 || vec[1] >= maze.get_height())
        return false;
    return !maze.get_pixel(vec[0], vec[1]).operator png::byte ();
}

/** Maze-like obstacle on a torus plane. */
bool mazeTorusValid (double sleep, png::image<png::index_pixel_1> &maze, const ompl::base::State *state, double r1)
{
    boost::this_thread::sleep(ompl::time::seconds(sleep));
    const ompl::base::AtlasStateSpace::StateType *astate = state->as<ompl::base::AtlasStateSpace::StateType>();
    Eigen::Ref<const Eigen::VectorXd> p = astate->constVectorView();
    Eigen::VectorXd vec(2);
    Eigen::VectorXd c(3); c << p[0], p[1], 0;
    vec[0] = std::atan2(p[2], c.norm()-r1)/(2*M_PI);
    vec[0] += (vec[0] < 0);
    vec[0] *= maze.get_height();
    vec[1] = std::atan2(p[1], p[0])/(2*M_PI);
    vec[1] += (vec[1] < 0);
    vec[1] *= maze.get_width();
    if (vec[0] < 0 || vec[0] >= maze.get_width() || vec[1] < 0 || vec[1] >= maze.get_height())
        return false;
    return !maze.get_pixel(vec[0], vec[1]).operator png::byte ();
}

/**
 * Problem initialization functions set the dimension, the manifold, start and goal points \a x and \a y,
 * and the validity checker \a isValid.
 */

/** Initialize the atlas for the sphere problem and store the start and goal vectors. */
ompl::base::AtlasStateSpace *initSphereProblem (Eigen::VectorXd &x, Eigen::VectorXd &y, ompl::base::StateValidityCheckerFn &isValid, double sleep)
{
    const std::size_t dim = 3;
    
    // Start and goal points
    x = Eigen::VectorXd(dim); x << 0, 0, -1;
    y = Eigen::VectorXd(dim); y << 0, 0,  1;
    
    // Validity checker
    isValid = boost::bind(&sphereValid, sleep, _1);;
    
    // Atlas initialization (can use numerical methods to compute the Jacobian, but giving an explicit function is faster)
    return new SphereManifold();
}

/** Initialize the atlas for the torus problem and store the start and goal vectors. */
ompl::base::AtlasStateSpace *initTorusProblem (Eigen::VectorXd &x, Eigen::VectorXd &y, ompl::base::StateValidityCheckerFn &isValid, double sleep)
{
    const std::size_t dim = 3;
    
    // Start and goal points
    x = Eigen::VectorXd(dim); x << -3, 0, -1;
    y = Eigen::VectorXd(dim); y <<  3, 0,  1;
    
    // Validity checker
    isValid = boost::bind(&unreachable, sleep, _1, y, 0.1);
    
    return new TorusManifold(3, 1);
}

/** Initialize the atlas for the sphere problem and store the start and goal vectors. */
ompl::base::AtlasStateSpace *initKleinBottleProblem (Eigen::VectorXd &x, Eigen::VectorXd &y, ompl::base::StateValidityCheckerFn &isValid, double sleep)
{
    const std::size_t dim = 3;
    
    // Start and goal points
    x = Eigen::VectorXd(dim); x << -0.5, -0.25, 0.1892222244330081;
    y = Eigen::VectorXd(dim); y <<  2.5, -1.5,  1.0221854181962458;
    
    // Validity checker
    isValid = boost::bind(&unreachable, sleep, _1, y, 0.2);
    
    return new KleinManifold();
}

/** Initialize the atlas for the kinematic chain problem. */
ompl::base::AtlasStateSpace *initChainProblem (Eigen::VectorXd &x, Eigen::VectorXd &y, ompl::base::StateValidityCheckerFn &isValid, const bool tough, double sleep)
{
    const std::size_t dim = 3*5;
    
    // Start and goal points (each triple is the 3D location of a joint)
    x = Eigen::VectorXd(dim); x << 1,  0, 0,  2,  0, 0,  2, -1, 0,  3, -1, 0,  3, 0, 0;
    y = Eigen::VectorXd(dim); y << 0, -1, 0, -1, -1, 0, -1,  0, 0, -2,  0, 0, -3, 0, 0;
    
    ChainManifold *atlas = new ChainManifold(3, 5);
    isValid = boost::bind(&ChainManifold::isValid, atlas, sleep, _1, tough);
    return atlas;
}

/** Initialize the atlas for the planar maze problem. */
ompl::base::AtlasStateSpace *initPlanarMazeProblem (Eigen::VectorXd &x, Eigen::VectorXd &y, ompl::base::StateValidityCheckerFn &isValid, const char *filename, double sleep)
{
    const std::size_t dim = 3;
    
    // Start and goal points
    x = Eigen::VectorXd(dim); x << 0.45, 0.02, 0;
    y = Eigen::VectorXd(dim); y << 0.98, 0.89, 0;
    x *= 7; y *= 7;
    
    // Load maze (memory leak!)
    png::image<png::index_pixel_1> *img = new png::image<png::index_pixel_1>(filename, png::require_color_space<png::index_pixel_1>());
    
    // Validity checker
    isValid = boost::bind(&mazePlaneValid, sleep, *img, _1);
    
    return new PlaneManifold();
}

/** Initialize the atlas for the torus maze problem. */
ompl::base::AtlasStateSpace *initTorusMazeProblem (Eigen::VectorXd &x, Eigen::VectorXd &y, ompl::base::StateValidityCheckerFn &isValid, const char *filename, double sleep)
{
    const std::size_t dim = 3;
    const double r1 = 2;
    const double r2 = 1;
    
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
    xB *= r2;
    yB *= r2;
    xB[0] += r1;
    yB[0] += r1;
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
    isValid = boost::bind(&mazeTorusValid, sleep, *img, _1, r1);
    
    ompl::base::AtlasStateSpace *atlas = new TorusManifold(r1, r2);
    return atlas;
}

/** Maps maze coordinates from [0,1]x[0,1] to 3D space on the torus. */
void mazeToTorusCoords (double a, double b, Eigen::Ref<Eigen::VectorXd> x, double r1, double r2)
{
    Eigen::VectorXd xA(2); xA << a, b;
    Eigen::VectorXd xB(3);
    xA *= 2*M_PI;
    xB << std::cos(xA[0]), 0, std::sin(xA[0]);
    xB *= r2;
    xB[0] += r1;
    x[0] = std::cos(xA[1]); x[1] = std::sin(xA[1]); x[2] = 0;
    x *= std::sqrt(xB[0]*xB[0] + xB[1]*xB[1]);;
    x[2] = xB[2];
}

/** Find valid configurations for points x1, x2, given x3. */
void threeLinkSolve (Eigen::Ref<Eigen::VectorXd> x1, Eigen::Ref<Eigen::VectorXd> x2,
                     Eigen::Ref<const Eigen::VectorXd> x3, std::vector<double> linklength)
{
    // The first one has to have norm linklength[0]. We'll put it facing straight down.
    //x1 = linklength[0] * x3.normalized();
    x1[0] = 0; x1[1] = 0; x1[2] = -linklength[0];

    // The second one must be at appropriate distance from the first and from the end point.
    // Compute s, the altitude of the triangle x1, x3, x2, with base x1, x3.
    const double base = (x3-x1).norm();
    const double halfp = (base + linklength[1] + linklength[2])/2;
    const double s = (2.0/base)*std::sqrt(
        halfp*(halfp-base)*(halfp-linklength[1])*(halfp-linklength[2]));
    // Compute t, the distance between x1 and the altitude line.
    const double t = std::sqrt(linklength[1]*linklength[1]-s*s);
    x2 = x1 + t*((x3-x1).normalized());
    // Add a vector v, of length s, to bring it to the third vertex of the triangle.
    // Fix all but one coefficient at -1, and solve for the final one.
    Eigen::VectorXd v = -Eigen::VectorXd::Ones(x2.size());
    int i;
    Eigen::VectorXd w = x3-x1;
    w.array().abs().maxCoeff(&i);
    v[i] = 0;
    v[i] = - (v.dot(w)) / w[i];
    x2 += s*v.normalized();
}

/** Initialize the atlas for the chain torus maze problem. */
ompl::base::AtlasStateSpace *initChainTorusMazeProblem (Eigen::VectorXd &x, Eigen::VectorXd &y,
                                                        ompl::base::StateValidityCheckerFn &isValid, double sleep)
{
    const std::size_t dim = 3;
    const std::size_t links = 3;
    std::vector<double> linklength;
    linklength.push_back(0.8);
    linklength.push_back(2);
    linklength.push_back(2);
    const double r1 = 2;
    const double r2 = 1;
    
    // Start and goal locations for the end effector
    x.resize(dim*links);
    y.resize(dim*links);
    Eigen::Ref<Eigen::VectorXd> x3(x.tail(dim));
    Eigen::Ref<Eigen::VectorXd> y3(y.tail(dim));
    mazeToTorusCoords(0.45, 0.02, x3, r1, r2);
    mazeToTorusCoords(0.8, 0.89, y3, r1, r2);

    // Determine locations of first two joints to satisfy the system
    Eigen::Ref<Eigen::VectorXd> x1(x.head(dim)), y1(y.head(dim));
    Eigen::Ref<Eigen::VectorXd> x2(x.segment(dim, dim)), y2(y.segment(dim, dim));
    threeLinkSolve(x1, x2, x3, linklength);
    threeLinkSolve(y1, y2, y3, linklength);

    ChainTorusManifold *atlas = new ChainTorusManifold(links, linklength, r1, r2);
    isValid = boost::bind(&ChainTorusManifold::isValid, atlas, sleep, _1);

    return atlas;
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
    std::cout << "    sphere torus klein chain chain_tough planar_maze torus_maze chain_torus_maze\n";
}

/** Print usage information. */
void printPlanners (void)
{
    std::cout << "Available planners:\n";
    std::cout << "    EST RRT RRTintermediate AtlasRRT RRTConnect RRTstar LazyRRT TRRT\n";
    std::cout << "    LBTRRT ConstrainedRRT CBiRRT2 KPIECE1 BKPIECE1 LBKPIECE1 PDST\n";
    std::cout << "    PRM PRMstar SBL SPARS SPARStwo STRIDE\n";
}

/** Initialize the problem specified in the string. */
ompl::base::AtlasStateSpace *parseProblem (const char *const problem, Eigen::VectorXd &x, Eigen::VectorXd &y,
                                           ompl::base::StateValidityCheckerFn &isValid, double sleep = 0)
{
    if (std::strcmp(problem, "sphere") == 0)
        return initSphereProblem(x, y, isValid, sleep);
    else if (std::strcmp(problem, "torus") == 0)
        return initTorusProblem(x, y, isValid, sleep);
    else if (std::strcmp(problem, "klein") == 0)
        return initKleinBottleProblem(x, y, isValid, sleep);
    else if (std::strcmp(problem, "chain") == 0)
        return initChainProblem(x, y, isValid, false, sleep);
    else if (std::strcmp(problem, "chain_tough") == 0)
        return initChainProblem(x, y, isValid, true, sleep);
    else if (std::strcmp(problem, "planar_maze") == 0)
        return initPlanarMazeProblem(x, y, isValid, "../../demos/atlas/maze.png", sleep);
    else if (std::strcmp(problem, "torus_maze") == 0)
        return initTorusMazeProblem(x, y, isValid, "../../demos/atlas/maze.png", sleep);
    else if (std::strcmp(problem, "chain_torus_maze") == 0)
        return initChainTorusMazeProblem(x, y, isValid, sleep);
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
    else if (std::strcmp(planner, "RRTintermediate") == 0)
    {
        ompl::geometric::RRT *rrt = new ompl::geometric::RRT(si, true);
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
        stride->setEstimatedDimension(6);
        return stride;
    }
    else
        return NULL;
}
