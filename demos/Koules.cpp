/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2012, Rice University
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

/* Author: Beck Chen, Mark Moll */

/**
\file Koules.cpp
\brief This file contains an elaborate demo to solve the game of
[Koules](http://www.ucw.cz/~hubicka/koules/English/).

This problem was used to illustrate the capabilities of the PDST planner to
find trajectories for underactuated systems with drift. The details can be
found in the references below [1,2]. The physics have been made significantly
harder compared to the original game. We have tried to recreate the problem as
closely as possible to the one described in [2]. The demo can solve just one
level of Koules, all levels, or run a number of planners on one level as a
benchmarking run.

This demo illustrates also many advanced OMPL concepts, such as custom state
space, control sampler, projection, propagator, and goal classes.

[1] A. M. Ladd and L. E. Kavraki, “Motion planning in the presence of drift,
underactuation and discrete system changes,” in Robotics: Science and Systems
I, (Boston, MA), pp. 233–241, MIT Press, June 2005.

[2] A. M. Ladd, Motion Planning for Physical Simulation. PhD thesis, Dept. of
Computer Science, Rice University, Houston, TX, Dec. 2006.
*/

#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/datastructures/NearestNeighborsSqrtApprox.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>
#include <ompl/control/planners/est/EST.h>
#include <ompl/control/planners/pdst/PDST.h>
#include <ompl/tools/benchmark/Benchmark.h>
#include <ompl/config.h>
#include <boost/math/constants/constants.hpp>
#include <boost/program_options.hpp>

// size of the square that defines workspace
const double sideLength = 1.;
// koule properties
const unsigned numKoules = 2;
const double kouleMass = .5;
const double kouleRadius = .015;
// ship properties
const double shipAcceleration = 1.;
const double shipRotVel = boost::math::constants::pi<double>();
const double shipMass = .75;
const double shipRadius = .03;
const double shipVmin = .05;
const double shipVmax = .5;
// dynamics, propagation, integration, control constants
const double lambda_c = 4.;
const double h = .05;
const double integrationStepSize = .005;
const double propagationStepSize = .1;
const unsigned int propagationMinSteps = 1;
const unsigned int propagationMaxSteps = ceil(boost::math::constants::pi<double>() / propagationStepSize);
const double shipDelta = .5 * shipAcceleration * propagationStepSize;
const double shipEps = .5 * shipRotVel * propagationStepSize;
// number of attempts at each level when solving n-level koules
const unsigned int numAttempts = 1;

namespace ob = ompl::base;
namespace oc = ompl::control;
namespace og = ompl::geometric;
namespace ot = ompl::tools;
namespace po = boost::program_options;

// A projection for the KoulesStateSpace
class KoulesProjection : public ob::ProjectionEvaluator
{
public:
    KoulesProjection(const ob::StateSpace* space, unsigned int numDimensions = 3)
        : ob::ProjectionEvaluator(space), numDimensions_(numDimensions)
    {
        unsigned int n = (space_->getDimension() - 1) / 2 + 1;
        if (numDimensions_ > n)
            numDimensions_ = n;
        else if (numDimensions_ < 3)
            numDimensions_ = 3;
    }

    virtual unsigned int getDimension(void) const
    {
        return numDimensions_;
    }
    virtual void defaultCellSizes(void)
    {
        cellSizes_.resize(numDimensions_, .05);
    }
    virtual void project(const ob::State *state, ob::EuclideanProjection &projection) const
    {
        const ob::CompoundStateSpace::StateType* cs = state->as<ob::CompoundStateSpace::StateType>();
        const double* xv = cs->as<ob::RealVectorStateSpace::StateType>(0)->values;
        const double theta = cs->as<ob::SO2StateSpace::StateType>(1)->value;
        unsigned int numKoules = (numDimensions_ - 3) / 2;
        // projection with coordinates in the same order as described in Andrew Ladd's thesis
        projection[0] = xv[4 * numKoules];
        projection[1] = xv[4 * numKoules + 1];
        projection[2] = theta;
        for (unsigned int i = 0; i < numKoules; ++i)
        {
            projection[2 * i + 3] = xv[4 * i];
            projection[2 * i + 4] = xv[4 * i + 1];
        }
    }
protected:
    unsigned int numDimensions_;
};

class KoulesStateSpace : public ob::CompoundStateSpace
{
public:
    KoulesStateSpace(unsigned int numKoules)
        : CompoundStateSpace(), mass_(numKoules + 1, kouleMass), radius_(numKoules + 1, kouleRadius)
    {
        mass_[numKoules] = shipMass;
        radius_[numKoules] = shipRadius;
        setName("Koules" + boost::lexical_cast<std::string>(numKoules) + getName());
        // layout: (... x_i y_i vx_i vy_i ... x_s y_s vx_s vy_s theta_s),
        // where (x_i, y_i) is the position of koule i (i=1,..,numKoules),
        // (vx_i, vy_i) its velocity, (x_s, y_s) the position of the ship,
        // (vx_s, vy_s) its velocity, and theta_s its orientation.
        addSubspace(ob::StateSpacePtr(new ob::RealVectorStateSpace(4 * (numKoules + 1))), 1.);
        addSubspace(ob::StateSpacePtr(new ob::SO2StateSpace), .5);
        lock();

        // create the bounds
        ob::RealVectorBounds bounds((numKoules + 1) * 4);
        unsigned int j = 0;
        for (unsigned int i = 0; i < numKoules; ++i)
        {
            // set the bounds for koule i's position
            bounds.setLow(j, -.1);
            bounds.setHigh(j++, sideLength + .1);
            bounds.setLow(j, -.1);
            bounds.setHigh(j++, sideLength + .1);
            // set the bounds for koule i's velocity
            bounds.setLow(j, -10.);
            bounds.setHigh(j++, 10.);
            bounds.setLow(j, -10.);
            bounds.setHigh(j++, 10.);
        }
        // set the bounds for the ship's position
        bounds.setLow(j, 0);
        bounds.setHigh(j++, sideLength);
        bounds.setLow(j, 0);
        bounds.setHigh(j++, sideLength);
        // set the bounds for the ship's velocity
        bounds.setLow(j, -5.);
        bounds.setHigh(j++, 5.);
        bounds.setLow(j, -5.);
        bounds.setHigh(j++, 5.);
        as<ob::RealVectorStateSpace>(0)->setBounds(bounds);
    }
    virtual void registerProjections(void)
    {
        registerDefaultProjection(ob::ProjectionEvaluatorPtr(new KoulesProjection(this)));
        registerProjection("PDSTProjection", ob::ProjectionEvaluatorPtr(
            new KoulesProjection(this, (getDimension() - 1) / 2 + 1)));
    }
    double getMass(unsigned int i) const
    {
        return mass_[i];
    }
    double getRadius(unsigned int i) const
    {
        return radius_[i];
    }
protected:
    std::vector<double> mass_;
    std::vector<double> radius_;
};

// Control sampler for KouleStateSpace
class KoulesControlSampler : public oc::ControlSampler
{
public:
    KoulesControlSampler(const oc::ControlSpace *space) : oc::ControlSampler(space)
    {
    }
    // Sample random velocity with magnitude between vmin and vmax and
    // orientation uniformly random over [0, 2*pi].
    // (This method is not actually ever called.)
    void sample(oc::Control *control)
    {
        const unsigned int dim = space_->getDimension();
        const ob::RealVectorBounds &bounds = static_cast<const oc::RealVectorControlSpace*>(space_)->getBounds();
        oc::RealVectorControlSpace::ControlType *rcontrol = static_cast<oc::RealVectorControlSpace::ControlType*>(control);
        double r = rng_.uniformReal(bounds.low[0], bounds.high[0]);
        double theta = rng_.uniformReal(0., 2. * boost::math::constants::pi<double>());
        rcontrol->values[0] = r * cos(theta);
        rcontrol->values[1] = r * sin(theta);
    }
    // sample random velocity with magnitude between vmin and vmax and
    // direction given by the normalized vector from the current position
    // in state and a random point in the workspace
    virtual void sample(oc::Control *control, const ob::State *state)
    {
        const KoulesStateSpace::StateType* s = static_cast<const KoulesStateSpace::StateType*>(state);
        const double* r = s->as<ob::RealVectorStateSpace::StateType>(0)->values;
        unsigned int dim = space_->getStateSpace()->getDimension();
        double dx = rng_.uniformReal(0., sideLength) - r[dim - 5];
        double dy = rng_.uniformReal(0., sideLength) - r[dim - 4];
        double xNrm2 = dx*dx + dy*dy;
        if (xNrm2 > std::numeric_limits<float>::epsilon())
        {
            const ob::RealVectorBounds &bounds = static_cast<const oc::RealVectorControlSpace*>(space_)->getBounds();
            double v = rng_.uniformReal(bounds.low[0], bounds.high[0]) / sqrt(xNrm2);
            oc::RealVectorControlSpace::ControlType *rcontrol = static_cast<oc::RealVectorControlSpace::ControlType*>(control);
            rcontrol->values[0] = v * dx;
            rcontrol->values[1] = v * dy;
        }
        else
            sample(control);
    }
    virtual void sampleNext(oc::Control *control, const oc::Control * /* previous */, const ob::State *state)
    {
        sample(control, state);
    }
protected:
    ompl::RNG rng_;
};

// Directed control sampler
class KoulesDirectedControlSampler : public oc::DirectedControlSampler
{
public:
    KoulesDirectedControlSampler(const oc::SpaceInformation *si)
        : DirectedControlSampler(si), cs_(si->allocControlSampler())
    {
    }
    virtual unsigned int sampleTo(oc::Control *control, const ob::State *source, ob::State *dest)
    {
        const KoulesStateSpace::StateType* src = static_cast<const KoulesStateSpace::StateType*>(source);
        const KoulesStateSpace::StateType* dst = static_cast<const KoulesStateSpace::StateType*>(dest);
        const double* srcPos = src->as<ob::RealVectorStateSpace::StateType>(0)->values;
        const double* dstPos = dst->as<ob::RealVectorStateSpace::StateType>(0)->values;
        unsigned int dim = si_->getStateSpace()->getDimension();
        // note the difference here with the sample function in KoulesControlSampler
        double dx = dstPos[dim - 5] - srcPos[dim - 5];
        double dy = dstPos[dim - 4] - srcPos[dim - 4];
        double xNrm2 = dx*dx + dy*dy;
        if (xNrm2 > std::numeric_limits<float>::epsilon())
        {
            const ob::RealVectorBounds &bounds = static_cast<const oc::RealVectorControlSpace*>(si_->getControlSpace().get())->getBounds();
            double v = rng_.uniformReal(bounds.low[0], bounds.high[0]) / sqrt(xNrm2);
            oc::RealVectorControlSpace::ControlType *rcontrol = static_cast<oc::RealVectorControlSpace::ControlType*>(control);
            rcontrol->values[0] = v * dx;
            rcontrol->values[1] = v * dy;
        }
        else
            cs_->sample(control);

        const double minDuration = si_->getMinControlDuration();
        const double maxDuration = si_->getMaxControlDuration();
        unsigned int steps = cs_->sampleStepCount(minDuration, maxDuration);
        return si_->propagateWhileValid(source, control, steps, dest);
    }
    virtual unsigned int sampleTo(oc::Control *control, const oc::Control * /* previous */, const ob::State *source, ob::State *dest)
    {
        return sampleTo(control, source, dest);
    }
protected:
    oc::ControlSamplerPtr  cs_;
    ompl::RNG              rng_;
};

oc::ControlSamplerPtr KoulesControlSamplerAllocator(const oc::ControlSpace* cspace)
{
    return oc::ControlSamplerPtr(new KoulesControlSampler(cspace));
}
oc::DirectedControlSamplerPtr KoulesDirectedControlSamplerAllocator(const oc::SpaceInformation *si)
{
    return oc::DirectedControlSamplerPtr(new KoulesDirectedControlSampler(si));
}

// State propagator for KouleModel.
class KoulesStatePropagator : public oc::StatePropagator
{
public:

    KoulesStatePropagator(const oc::SpaceInformationPtr &si) :
        oc::StatePropagator(si), timeStep_(integrationStepSize),
        numDimensions_(si->getStateSpace()->getDimension()),
        numKoules_((numDimensions_ - 5) / 4)
    {
    }

    virtual void propagate(const ob::State *start, const oc::Control* control,
        const double duration, ob::State *result) const
    {
        const double* cval = static_cast<const oc::RealVectorControlSpace::ControlType*>(control)->values;
        unsigned int numSteps = ceil(duration / timeStep_), offset = 4 * numKoules_, u;
        double dt = duration / (double)numSteps;
        std::vector<double> qdot(numDimensions_), q(numDimensions_);
        std::vector<bool> hasCollision(numKoules_ + 1);

        si_->getStateSpace()->copyToReals(q, start);

        double v[2] = { cval[0] - q[offset + 2], cval[1] - q[offset + 3]};
        double deltaTheta = atan2(v[1], v[0]) - q[offset + 4];
        if (v[0]*v[0] + v[1]*v[1] < shipDelta * shipDelta)
            u = 0;
        else if (std::abs(deltaTheta) < shipEps)
            u = 3;
        else if (deltaTheta > 0)
            u = 1;
        else
            u = 2;
        for (unsigned int i = 0; i < numSteps; ++i)
        {
            ode(q, u, qdot);
            update(q, qdot, dt, hasCollision);
        }
        si_->getStateSpace()->copyFromReals(result, q);
        // Normalize orientation between -pi and pi
        ob::SO2StateSpace SO2;
        SO2.enforceBounds(result->as<ob::CompoundStateSpace::StateType>()
            ->as<ob::SO2StateSpace::StateType>(1));
    }

protected:

    void ode(std::vector<double>& q, unsigned int u, std::vector<double>& qdot) const
    {
        // koules: qdot[4*i, 4*i + 1] is xdot, qdot[4*i + 2, 4*i + 3] is vdot
        for (unsigned int i = 0; i < numKoules_; i++)
        {
            qdot[4 * i    ] = q[4 * i + 2];
            qdot[4 * i + 1] = q[4 * i + 3];
            qdot[4 * i + 2] = (.5 * sideLength - q[4 * i    ]) * lambda_c - q[4 * i + 2] * h;
            qdot[4 * i + 3] = (.5 * sideLength - q[4 * i + 1]) * lambda_c - q[4 * i + 3] * h;
        }
        // ship: qdot[offset, offset + 1] is xdot
        // ship: qdot[4 * numKoules_ + ] is thetadot, qdot[3,4] is vdot
        unsigned int offset = 4 * numKoules_;
        qdot[offset    ] = q[offset + 2];
        qdot[offset + 1] = q[offset + 3];
        if (u == 0) // drift
        {
            qdot[offset + 2] = qdot[offset + 3] = qdot[offset + 4] = 0.;
        }
        else if (u == 1) // rotate counterclockwise
        {
            qdot[offset + 2] = qdot[offset + 3] = 0.;
            qdot[offset + 4] = shipRotVel;
        }
        else if (u == 2) // rotate clockwise
        {
            qdot[offset + 2] = qdot[offset + 3] = 0.;
            qdot[offset + 4] = -shipRotVel;
        }
        else if (u == 3) // accelerate
        {
            qdot[offset + 2] = shipAcceleration * cos(q[offset + 4]);
            qdot[offset + 3] = shipAcceleration * sin(q[offset + 4]);
            qdot[offset + 4] = 0.;
        }


    }

    void update(std::vector<double>& q, const std::vector<double>& qdot, double dt,
        std::vector<bool>& hasCollision) const
    {
        // update collisions
        std::fill(hasCollision.begin(), hasCollision.end(), false);
        for (unsigned int i = 0; i < numKoules_; i++)
            for (unsigned int j = i + 1; j <= numKoules_; j++)
                if (checkCollision(q, i, j, dt))
                    hasCollision[i] = hasCollision[j] = true;

        // update objects with no collision according to qdot
        for (unsigned int i = 0; i < numKoules_; ++i)
            if (!hasCollision[i])
                for (unsigned int j = 0; j < 4; ++j)
                    q[4 * i + j] += qdot[4 * i + j] * dt;
        if (!hasCollision[numKoules_])
            for (unsigned int j = 0; j < 5; ++j)
                q[4 * numKoules_ + j] += qdot[4 * numKoules_ + j] * dt;
    }

    double* getState(std::vector<double>& q, unsigned int i) const
    {
        return &q[4 * i];
    }
    // check collision among object i and j
    // compute elastic collision response if i and j collide
    // see http://en.wikipedia.org/wiki/Elastic_collision
    bool checkCollision(std::vector<double>& q, unsigned int i, unsigned int j, double dt) const
    {
        static const float delta = 1e-5;
        double *a = getState(q, i), *b = getState(q, j);
        float dx = a[0] - b[0], dy = a[1] - b[1];
        float dist = dx * dx + dy * dy;
        float minDist = si_->getStateSpace()->as<KoulesStateSpace>()->getRadius(i) +
            si_->getStateSpace()->as<KoulesStateSpace>()->getRadius(j) + delta;
        if (dist < minDist*minDist && ((b[2] - a[2]) * dx + (b[3] - a[3]) * dy > 0))
        // close enough and moving closer; elastic collision happens
        {
            dist = sqrt(dist);
            // compute unit normal and tangent vectors
            float normal[2] = {dx / dist, dy / dist};
            float tangent[2] = {-normal[1], normal[0]};

            // compute scalar projections of velocities onto normal and tangent vectors
            float aNormal = normal[0] * a[2] + normal[1] * a[3];
            float aTangentPrime = tangent[0] * a[2] + tangent[1] * a[3];
            float bNormal = normal[0] * b[2] + normal[1] * b[3];
            float bTangentPrime = tangent[0] * b[2] + tangent[1] * b[3];

            // compute new velocities using one-dimensional elastic collision in the normal direction
            float massA = si_->getStateSpace()->as<KoulesStateSpace>()->getMass(i);
            float massB = si_->getStateSpace()->as<KoulesStateSpace>()->getMass(j);
            float aNormalPrime = (aNormal * (massA - massB) + 2 * massB * bNormal) / (massA + massB);
            float bNormalPrime = (bNormal * (massB - massA) + 2 * massA * aNormal) / (massA + massB);

            // compute new normal and tangential velocity vectors
            float aNewNormalVel [2] = {normal[0] * aNormalPrime, normal[1] * aNormalPrime};
            float aNewTangentVel [2] = {tangent[0] * aTangentPrime, tangent[1] * aTangentPrime};
            float bNewNormalVel [2] = {normal[0] * bNormalPrime, normal[1] * bNormalPrime};
            float bNewTangentVel [2] = {tangent[0] * bTangentPrime, tangent[1] * bTangentPrime};

            // compute new velocities
            float bNewVel [2] = { bNewNormalVel[0] + bNewTangentVel[0], bNewNormalVel[1] + bNewTangentVel[1] };
            float aNewVel [2] = { aNewNormalVel[0] + aNewTangentVel[0], aNewNormalVel[1] + aNewTangentVel[1] };

            // update state if collision happens
            a[0] += aNewVel[0] * dt;
            a[1] += aNewVel[1] * dt;
            a[2] = aNewVel[0];
            a[3] = aNewVel[1];
            b[0] += bNewVel[0] * dt;
            b[1] += bNewVel[1] * dt;
            b[2] = bNewVel[0];
            b[3] = bNewVel[1];

            return true;
        }
        else
            return false;
    }

    double timeStep_;
    unsigned int numDimensions_;
    unsigned int numKoules_;
};


// Sampleable goal region for KoulesModel.
class KoulesGoal : public ob::GoalSampleableRegion
{
public:
    KoulesGoal(const ob::SpaceInformationPtr &si)
        : ob::GoalSampleableRegion(si), stateSampler_(si->allocStateSampler())
    {
        threshold_ = 0.01;
        numKoules_ = (si->getStateDimension() - 5) / 4;
    }

    virtual double distanceGoal(const ob::State *st) const
    {
        // the shortest distance between any koule and an edge
        double minDist = sideLength;
        double minX, minY;
        const double* v = st->as<ob::CompoundStateSpace::StateType>()
            ->as<ob::RealVectorStateSpace::StateType>(0)->values;
        for (unsigned int i = 0; i < numKoules_; ++i)
        {
            minX = std::min(v[4 * i    ], sideLength - v[4 * i    ]);
            minY = std::min(v[4 * i + 1], sideLength - v[4 * i + 1]);
            minDist = std::min(minDist, std::min(minX, minY));
        }
        if (minDist < 0)
            minDist = 0;
        return minDist;
    }

    virtual unsigned int maxSampleCount(void) const
    {
        return 100;
    }

    virtual void sampleGoal(ob::State *st) const
    {
        double* v = st->as<ob::CompoundStateSpace::StateType>()
            ->as<ob::RealVectorStateSpace::StateType>(0)->values;
        stateSampler_->sampleUniform(st);
        for (unsigned i = 0; i < numKoules_; ++i)
        {
            // randomly pick an edge for each koule to collide
            if (rng_.uniformBool())
            {
                v[4 * i    ] = rng_.uniformBool() ? 0. : sideLength;
                v[4 * i + 1] = rng_.uniformReal(0., sideLength);
            }
            else
            {
                v[4 * i    ] = rng_.uniformReal(0., sideLength);
                v[4 * i + 1] = rng_.uniformBool() ? 0. : sideLength;
            }
        }
    }

private:
    mutable ompl::RNG rng_;
    ob::StateSamplerPtr stateSampler_;
    unsigned int numKoules_;
};



bool isStateValid(const oc::SpaceInformation *si, const ob::State *state)
{
    return si->satisfiesBounds(state);
}

oc::SimpleSetup* koulesSetup(unsigned int numKoules, const std::string& plannerName, const std::vector<double>& stateVec = std::vector<double>())
{
    // construct state space
    ob::StateSpacePtr space(new KoulesStateSpace(numKoules));
    space->setup();
    // construct control space
    oc::ControlSpacePtr cspace(new oc::RealVectorControlSpace(space, 2));
    ob::RealVectorBounds cbounds(2);
    cbounds.setLow(shipVmin);
    cbounds.setHigh(shipVmax);
    cspace->as<oc::RealVectorControlSpace>()->setBounds(cbounds);
    // set control sampler allocator
    cspace->setControlSamplerAllocator(KoulesControlSamplerAllocator);

    // define a simple setup class
    oc::SimpleSetup* ss = new oc::SimpleSetup(cspace);
    oc::SpaceInformationPtr si = ss->getSpaceInformation();
    // set min/max propagation steps
    si->setMinMaxControlDuration(propagationMinSteps, propagationMaxSteps);
    // set directed control sampler
    si->setDirectedControlSamplerAllocator(KoulesDirectedControlSamplerAllocator);
    // set planner
    if (plannerName == "rrt")
        ss->setPlanner(ob::PlannerPtr(new oc::RRT(si)));
    else if (plannerName == "est")
        ss->setPlanner(ob::PlannerPtr(new oc::EST(si)));
    else if (plannerName == "kpiece")
        ss->setPlanner(ob::PlannerPtr(new oc::KPIECE1(si)));
    else
    {
        ob::PlannerPtr pdstplanner(new oc::PDST(si));
        pdstplanner->as<oc::PDST>()->setProjectionEvaluator(space->getProjection("PDSTProjection"));
        ss->setPlanner(pdstplanner);
    }

    // set validity checker
    ss->setStateValidityChecker(boost::bind(&isStateValid, si.get(), _1));
    // set state propagator
    ss->setStatePropagator(oc::StatePropagatorPtr(new KoulesStatePropagator(si)));
    // setup start state
    ob::ScopedState<> start(space);
    if (stateVec.size() == space->getDimension())
        space->copyFromReals(start.get(), stateVec);
    else
    {
        // Pick koule positions evenly radially distributed, but at a random distance
        // from the center. The ship's initial position is at the center. Initial
        // velocities are 0.
        std::vector<double> startVec(space->getDimension(), 0.);
        double r, theta=boost::math::constants::pi<double>(), delta = 2.*theta / numKoules, vr, vtheta;
        for (unsigned int i = 0; i < numKoules; ++i, theta += delta)
        {
            r = .1 + i * .1 / numKoules;
            startVec[4 * i    ] = .5 * sideLength + r * cos(theta);
            startVec[4 * i + 1] = .5 * sideLength + r * sin(theta);
        }
        startVec[4 * numKoules    ] = .5 * sideLength;
        startVec[4 * numKoules + 1] = .5 * sideLength;
        space->copyFromReals(start.get(), startVec);
    }
    ss->setStartState(start);
    // set goal
    ss->setGoal(ob::GoalPtr(new KoulesGoal(si)));
    // set propagation step size
    si->setPropagationStepSize(propagationStepSize);
    return ss;
}

void planOneLevel(oc::SimpleSetup& ss, double maxTime, const std::string& plannerName)
{
    if (ss.solve(maxTime))
    {
        oc::PathControl path(ss.getSolutionPath());
        oc::SpaceInformationPtr si(ss.getSpaceInformation());
        // increase the number of interpolated states by a factor 2
        si->setPropagationStepSize(.5 * si->getPropagationStepSize());
        path.interpolate();
        path.printAsMatrix(std::cout);
        if (!ss.haveExactSolutionPath())
            std::cerr << "Solution is approximate. Distance to actual goal is " <<
                ss.getProblemDefinition()->getSolutionDifference() << std::endl;
    }
}

void planAllLevelsRecursive(oc::SimpleSetup* ss, double maxTime, const std::string& plannerName,
    std::vector<ob::PathPtr>& solution)
{
    double timeAttempt = maxTime / numAttempts;
    for (unsigned int i = 0; i < numAttempts; ++i)
    {
        ompl::time::point startTime = ompl::time::now();
        solution.clear();
        ss->clear();
        OMPL_INFORM("Attempt %d of %d to solve for %d koules",
            i + 1, numAttempts, (ss->getStateSpace()->getDimension() - 5)/4);
        if (ss->solve(timeAttempt) != ob::PlannerStatus::EXACT_SOLUTION)
            continue;

        ob::PathPtr path(ss->getProblemDefinition()->getSolutionPath());
        oc::PathControl* cpath = static_cast<oc::PathControl*>(path.get());
        oc::SpaceInformationPtr si(ss->getSpaceInformation());
        const ob::State* goalState = cpath->getStates().back();
        double tol = ss->getProblemDefinition()->getGoal()->as<KoulesGoal>()->getThreshold();
        std::vector<double> s, nextStart;

        ss->getStateSpace()->copyToReals(s, goalState);
        nextStart.reserve(s.size() - 4);
        for (unsigned int j = 0; j < s.size() - 5; j += 4)
            // include koule in next state if it is within workspace
            if (std::min(s[j], s[j+1]) > tol && std::max(s[j], s[j+1]) < sideLength - tol)
                for (unsigned k = 0; k < 4; ++k)
                    nextStart.push_back(s[j + k]);
        // add ship's state
        for (unsigned int j = s.size() - 5; j < s.size(); ++j)
            nextStart.push_back(s[j]);
        assert(nextStart.size() < s.size());

        unsigned int numKoules = (nextStart.size() - 5) / 4;
        if (numKoules > 0)
        {
            double timeElapsed = (ompl::time::now() - startTime).total_microseconds() * 1e-6;
            oc::SimpleSetup* ssNext = koulesSetup(numKoules, plannerName, nextStart);
            planAllLevelsRecursive(ssNext, timeAttempt - timeElapsed, plannerName, solution);
            if (solution.size() == 0)
                delete ssNext;
        }
        if (numKoules == 0 || solution.size())
        {
            si->setPropagationStepSize(.5*si->getPropagationStepSize());
            cpath->interpolate();
            solution.push_back(path);
            OMPL_INFORM("Solution found for %d koules", (s.size() - 5) / 4);
            return;
        }
    }
}

void planAllLevels(oc::SimpleSetup& ss, double maxTime, const std::string& plannerName)
{
    std::vector<ob::PathPtr> solution;
    planAllLevelsRecursive(&ss, maxTime, plannerName, solution);
    if (solution.size())
        for (std::vector<ob::PathPtr>::reverse_iterator p = solution.rbegin(); p != solution.rend(); p++)
            static_cast<oc::PathControl*>((*p).get())->printAsMatrix(std::cout);
}

void benchmarkOneLevel(oc::SimpleSetup& ss, ot::Benchmark::Request request)
{
    // Create a benchmark class
    ompl::tools::Benchmark b(ss, "Koules experiment");
    // Add the planners to evaluate
    b.addPlanner(ob::PlannerPtr(new oc::RRT(ss.getSpaceInformation())));
    b.addPlanner(ob::PlannerPtr(new oc::KPIECE1(ss.getSpaceInformation())));
    b.addPlanner(ob::PlannerPtr(new oc::EST(ss.getSpaceInformation())));
    // PDST uses a different projection
    ob::PlannerPtr pdstplanner(new oc::PDST(ss.getSpaceInformation()));
    pdstplanner->as<oc::PDST>()->setProjectionEvaluator(
        ss.getStateSpace()->getProjection("PDSTProjection"));
    b.addPlanner(pdstplanner);
    // Start benchmark
    b.benchmark(request);
    // This will generate a file of the form ompl_host_time.log
    b.saveResultsToFile();
}

int main(int argc, char **argv)
{
    try
    {
        po::options_description desc("Options");
        desc.add_options()
            ("help", "show help message")
            ("plan", "plan one level of koules")
            ("planall", "plan all levels of koules")
            ("benchmark", "benchmark one level")
            ("numkoules", po::value<unsigned int>()->default_value(numKoules),
                "start from <numkoules> koules")
            ("maxtime", po::value<double>()->default_value(10.),
                "time limit in seconds")
            ("numruns", po::value<unsigned int>()->default_value(10),
                "number of runs for each planner in benchmarking mode")
            ("planner", po::value<std::string>()->default_value("pdst"),
                "planning algorithm to use (pdst, kpiece, rrt, or est)")
        ;

        po::variables_map vm;
        po::store(po::parse_command_line(argc, argv, desc,
            po::command_line_style::unix_style ^ po::command_line_style::allow_short), vm);
        po::notify(vm);

        double maxTime = vm["maxtime"].as<double>();
        unsigned int numRuns = vm["numruns"].as<unsigned int>();
        unsigned int numKoules = vm["numkoules"].as<unsigned int>();
        std::string plannerName = vm["planner"].as<std::string>();
        oc::SimpleSetup* ss = koulesSetup(numKoules, plannerName);
        if (vm.count("help") || argc==1)
        {
            std::cout << desc << "\n";
            return 1;
        }
        if (vm.count("plan"))
            planOneLevel(*ss, maxTime, plannerName);
        if (vm.count("planall"))
            planAllLevels(*ss, maxTime, plannerName);
        if (vm.count("benchmark"))
            benchmarkOneLevel(*ss, ot::Benchmark::Request(maxTime, 10000.0, numRuns));
    }
    catch(std::exception& e) {
        std::cerr << "error: " << e.what() << "\n";
        return 1;
    }
    catch(...) {
        std::cerr << "Exception of unknown type!\n";
    }

    return 0;
}
