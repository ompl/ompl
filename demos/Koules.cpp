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
        const ob::RealVectorBounds &bounds = space_->as<oc::RealVectorControlSpace>()->getBounds();
        oc::RealVectorControlSpace::ControlType *rcontrol =
            control->as<oc::RealVectorControlSpace::ControlType>();
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
        steer(control, state, rng_.uniformReal(0., sideLength), rng_.uniformReal(0., sideLength));
    }
    virtual void sampleNext(oc::Control *control, const oc::Control * /* previous */, const ob::State *state)
    {
        sample(control, state);
    }
    virtual void steer(oc::Control *control, const ob::State *state, double x, double y)
    {
        const KoulesStateSpace::StateType* s = state->as<KoulesStateSpace::StateType>();
        const double* r = s->as<ob::RealVectorStateSpace::StateType>(0)->values;
        unsigned int dim = space_->getStateSpace()->getDimension();
        double dx = x - r[dim - 5];
        double dy = y - r[dim - 4];
        double xNrm2 = dx * dx + dy * dy;
        if (xNrm2 > std::numeric_limits<float>::epsilon())
        {
            const ob::RealVectorBounds &bounds = space_->as<oc::RealVectorControlSpace>()->getBounds();
            double v = rng_.uniformReal(bounds.low[0], bounds.high[0]) / sqrt(xNrm2);
            oc::RealVectorControlSpace::ControlType *rcontrol =
                control->as<oc::RealVectorControlSpace::ControlType>();
            rcontrol->values[0] = v * dx;
            rcontrol->values[1] = v * dy;
        }
        else
            sample(control);
    }

protected:
    ompl::RNG rng_;
};

// Directed control sampler
class KoulesDirectedControlSampler : public oc::DirectedControlSampler
{
public:
    KoulesDirectedControlSampler(const oc::SpaceInformation *si)
        : DirectedControlSampler(si), cs_(si->getControlSpace().get())
    {
    }
    virtual unsigned int sampleTo(oc::Control *control, const ob::State *source, ob::State *dest)
    {
        const KoulesStateSpace::StateType* dst = dest->as<KoulesStateSpace::StateType>();
        const double* dstPos = dst->as<ob::RealVectorStateSpace::StateType>(0)->values;
        const double minDuration = si_->getMinControlDuration();
        const double maxDuration = si_->getMaxControlDuration();
        unsigned int dim = si_->getStateSpace()->getDimension();
        unsigned int steps = cs_.sampleStepCount(minDuration, maxDuration);

        cs_.steer(control, source, dstPos[dim - 5], dstPos[dim - 4]);
        return si_->propagateWhileValid(source, control, steps, dest);
    }
    virtual unsigned int sampleTo(oc::Control *control, const oc::Control * /* previous */,
        const ob::State *source, ob::State *dest)
    {
        return sampleTo(control, source, dest);
    }
protected:
    KoulesControlSampler cs_;
    ompl::RNG            rng_;
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
        const double* cval = control->as<oc::RealVectorControlSpace::ControlType>()->values;
        unsigned int numSteps = ceil(duration / timeStep_), offset = 4 * numKoules_;
        double dt = duration / (double)numSteps, u[3] = {0., 0., 0.};
        std::vector<double> qdot(numDimensions_), q(numDimensions_);
        std::vector<bool> hasCollision(numKoules_ + 1);

        si_->getStateSpace()->copyToReals(q, start);

        double v[2] = { cval[0] - q[offset + 2], cval[1] - q[offset + 3]};
        double deltaTheta = atan2(v[1], v[0]) - q[offset + 4];
        if (v[0]*v[0] + v[1]*v[1] > shipDelta * shipDelta)
        {
            if (std::abs(deltaTheta) < shipEps)
            {
                u[0] = shipAcceleration * cos(q[offset + 4]);
                u[1] = shipAcceleration * sin(q[offset + 4]);
                u[2] = 0.;
            }
            else if (deltaTheta > 0)
                u[2] = shipRotVel;
            else
                u[2] = -shipRotVel;
        }
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

    void ode(std::vector<double>& q, double* u, std::vector<double>& qdot) const
    {
        // koules: qdot[4*i, 4*i + 1] is xdot, qdot[4*i + 2, 4*i + 3] is vdot
        unsigned int offset = 4 * numKoules_;
        for (unsigned int i = 0; i < offset; i += 4)
        {
            qdot[i    ] = q[i + 2];
            qdot[i + 1] = q[i + 3];
            qdot[i + 2] = (.5 * sideLength - q[i    ]) * lambda_c - q[i + 2] * h;
            qdot[i + 3] = (.5 * sideLength - q[i + 1]) * lambda_c - q[i + 3] * h;
        }
        // ship: qdot[offset, offset + 1] is xdot
        // ship: qdot[offset + 4] + ] is thetadot, qdot[offset + 2, offset + 3] is vdot
        qdot[offset    ] = q[offset + 2];
        qdot[offset + 1] = q[offset + 3];
        qdot[offset + 2] = u[0];
        qdot[offset + 3] = u[1];
        qdot[offset + 4] = u[2];
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

    // check collision among object i and j
    // compute elastic collision response if i and j collide
    // see http://en.wikipedia.org/wiki/Elastic_collision
    bool checkCollision(std::vector<double>& q, unsigned int i, unsigned int j, double dt) const
    {
        static const float delta = 1e-5;
        double *a = &q[4 * i], *b = &q[4 * j];
        double dx = a[0] - b[0], dy = a[1] - b[1];
        double dist = dx * dx + dy * dy;
        double minDist = si_->getStateSpace()->as<KoulesStateSpace>()->getRadius(i) +
            si_->getStateSpace()->as<KoulesStateSpace>()->getRadius(j) + delta;
        if (dist < minDist*minDist && ((b[2] - a[2]) * dx + (b[3] - a[3]) * dy > 0))
        // close enough and moving closer; elastic collision happens
        {
            dist = sqrt(dist);
            // compute unit normal and tangent vectors
            double normal[2] = {dx / dist, dy / dist};
            double tangent[2] = {-normal[1], normal[0]};

            // compute scalar projections of velocities onto normal and tangent vectors
            double aNormal = normal[0] * a[2] + normal[1] * a[3];
            double aTangentPrime = tangent[0] * a[2] + tangent[1] * a[3];
            double bNormal = normal[0] * b[2] + normal[1] * b[3];
            double bTangentPrime = tangent[0] * b[2] + tangent[1] * b[3];

            // compute new velocities using one-dimensional elastic collision in the normal direction
            double massA = si_->getStateSpace()->as<KoulesStateSpace>()->getMass(i);
            double massB = si_->getStateSpace()->as<KoulesStateSpace>()->getMass(j);
            double aNormalPrime = (aNormal * (massA - massB) + 2. * massB * bNormal) / (massA + massB);
            double bNormalPrime = (bNormal * (massB - massA) + 2. * massA * aNormal) / (massA + massB);

            // compute new normal and tangential velocity vectors
            double aNewNormalVel[2] = {normal[0] * aNormalPrime, normal[1] * aNormalPrime};
            double aNewTangentVel[2] = {tangent[0] * aTangentPrime, tangent[1] * aTangentPrime};
            double bNewNormalVel[2] = {normal[0] * bNormalPrime, normal[1] * bNormalPrime};
            double bNewTangentVel[2] = {tangent[0] * bTangentPrime, tangent[1] * bTangentPrime};

            // compute new velocities
            double bNewVel[2] = { bNewNormalVel[0] + bNewTangentVel[0], bNewNormalVel[1] + bNewTangentVel[1] };
            double aNewVel[2] = { aNewNormalVel[0] + aNewTangentVel[0], aNewNormalVel[1] + aNewTangentVel[1] };

            // preservation of momemtum
            assert(std::abs(massA * (a[2]-aNewVel[0]) + massB * (b[2]-bNewVel[0])) < 1e-6);
            assert(std::abs(massA * (a[3]-aNewVel[1]) + massB * (b[3]-bNewVel[1])) < 1e-6);
            // preservation of kinetic energy
            assert(std::abs(massA * (a[2]*a[2] + a[3]*a[3] - aNewVel[0]*aNewVel[0] - aNewVel[1]*aNewVel[1])
                + massB * (b[2]*b[2] + b[3]*b[3] - bNewVel[0]*bNewVel[0] - bNewVel[1]*bNewVel[1])) < 1e-6);

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
        // Pick koule positions evenly radially distributed, but at a linearly
        // increasing distance from the center. The ship's initial position is
        // at the center. Initial velocities are 0.
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
oc::SimpleSetup* koulesSetup(unsigned int numKoules, const std::string& plannerName, double kouleVel)
{
    oc::SimpleSetup* ss = koulesSetup(numKoules, plannerName);
    double* state = ss->getProblemDefinition()->getStartState(0)->as<KoulesStateSpace::StateType>()
        ->as<ob::RealVectorStateSpace::StateType>(0)->values;
    double theta;
    ompl::RNG rng;
    for (unsigned int i = 0; i < numKoules; ++i)
    {
        theta = rng.uniformReal(0., 2. * boost::math::constants::pi<double>());
        state[4 * i + 2] = kouleVel * cos(theta);
        state[4 * i + 3] = kouleVel * sin(theta);
    }
    return ss;
}

void planOneLevel(oc::SimpleSetup& ss, double maxTime, const std::string& plannerName)
{
    if (ss.solve(maxTime))
    {
        oc::PathControl path(ss.getSolutionPath());
        path.interpolate();
        if (!path.check())
            OMPL_ERROR("Path is invalid");
        path.printAsMatrix(std::cout);
        if (!ss.haveExactSolutionPath())
            OMPL_INFORM("Solution is approximate. Distance to actual goal is %g",
                ss.getProblemDefinition()->getSolutionDifference());
    }
}

void planAllLevelsRecursive(oc::SimpleSetup* ss, double maxTime, const std::string& plannerName,
    std::vector<ob::PathPtr>& solution)
{
    double timeAttempt = maxTime / numAttempts;
    double tol = ss->getProblemDefinition()->getGoal()->as<KoulesGoal>()->getThreshold();
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
        const ob::State* goalState = cpath->getStates().back();
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
        // make sure the problem size decreases as we recurse
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
            static_cast<oc::PathControl*>(p->get())->printAsMatrix(std::cout);
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
        unsigned int numKoules, numRuns;
        double maxTime, kouleVel;
        std::string plannerName;
        po::options_description desc("Options");
        desc.add_options()
            ("help", "show help message")
            ("plan", "plan one level of koules")
            ("planall", "plan all levels of koules")
            ("benchmark", "benchmark one level")
            ("numkoules", po::value<unsigned int>(&numKoules)->default_value(2),
                "start from <numkoules> koules")
            ("maxtime", po::value<double>(&maxTime)->default_value(10.),
                "time limit in seconds")
            ("numruns", po::value<unsigned int>(&numRuns)->default_value(10),
                "number of runs for each planner in benchmarking mode")
            ("planner", po::value<std::string>(&plannerName)->default_value("kpiece"),
                "planning algorithm to use (pdst, kpiece, rrt, or est)")
            ("velocity", po::value<double>(&kouleVel)->default_value(0.),
                "initial velocity of each koule")
        ;

        po::variables_map vm;
        po::store(po::parse_command_line(argc, argv, desc,
            po::command_line_style::unix_style ^ po::command_line_style::allow_short), vm);
        po::notify(vm);

        oc::SimpleSetup* ss = koulesSetup(numKoules, plannerName, kouleVel);
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
        delete ss;
    }
    catch(std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }
    catch(...) {
        std::cerr << "Exception of unknown type!\n";
    }

    return 0;
}
