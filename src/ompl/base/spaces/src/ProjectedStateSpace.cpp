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

/* Author: Zachary Kingston */

#include "ompl/base/spaces/ProjectedStateSpace.h"

#include "ompl/base/PlannerDataGraph.h"
#include "ompl/base/SpaceInformation.h"
#include "ompl/util/Exception.h"

#include <boost/graph/iteration_macros.hpp>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

/// ProjectedStateSampler

/// Public

ompl::base::ProjectedStateSampler::ProjectedStateSampler(const SpaceInformation *si)
  : RealVectorStateSampler(si->getStateSpace().get())
  , ss_(*si->getStateSpace()->as<ProjectedStateSpace>())
{
    ProjectedStateSpace::checkSpace(si);
}

ompl::base::ProjectedStateSampler::ProjectedStateSampler(const ProjectedStateSpace &ss)
    : RealVectorStateSampler(&ss)
    , ss_(ss)
{
}

void ompl::base::ProjectedStateSampler::sampleUniform(State *state)
{
    RealVectorStateSampler::sampleUniform(state);
    ss_.getConstraint()->project(state);
}

void ompl::base::ProjectedStateSampler::sampleUniformNear(State *state, const State *near, const double distance)
{
    RealVectorStateSampler::sampleUniformNear(state, near, distance);
    ss_.getConstraint()->project(state);
}

void ompl::base::ProjectedStateSampler::sampleGaussian(State *state, const State *mean, const double stdDev)
{
    RealVectorStateSampler::sampleGaussian(state, mean, stdDev);
    ss_.getConstraint()->project(state);
}

/// ProjectedValidStateSampler

/// Public

ompl::base::ProjectedValidStateSampler::ProjectedValidStateSampler(const SpaceInformation *si)
  : ValidStateSampler(si), sampler_(si)
{
    ProjectedStateSpace::checkSpace(si);
}

bool ompl::base::ProjectedValidStateSampler::sample(State *state)
{
    // Rejection sample for at most attempts_ tries.
    unsigned int tries = 0;
    bool valid;
    do
        sampler_.sampleUniform(state);
    while (!(valid = si_->isValid(state)) && ++tries < attempts_);

    return valid;
}

bool ompl::base::ProjectedValidStateSampler::sampleNear(State *state, const State *near, const double distance)
{
    // Rejection sample for at most attempts_ tries.
    unsigned int tries = 0;
    bool valid;
    do
        sampler_.sampleUniformNear(state, near, distance);
    while (!(valid = si_->isValid(state)) && ++tries < attempts_);

    return valid;
}

/// ProjectedMotionValidator

/// Public

ompl::base::ProjectedMotionValidator::ProjectedMotionValidator(SpaceInformation *si)
  : MotionValidator(si), ss_(*si->getStateSpace()->as<ProjectedStateSpace>())
{
    ProjectedStateSpace::checkSpace(si);
}

ompl::base::ProjectedMotionValidator::ProjectedMotionValidator(const SpaceInformationPtr &si)
  : MotionValidator(si), ss_(*si->getStateSpace()->as<ProjectedStateSpace>())
{
    ProjectedStateSpace::checkSpace(si.get());
}

bool ompl::base::ProjectedMotionValidator::checkMotion(const State *s1, const State *s2) const
{
    return ss_.traverseManifold(s1->as<ProjectedStateSpace::StateType>(), s2->as<ProjectedStateSpace::StateType>());
}

bool ompl::base::ProjectedMotionValidator::checkMotion(const State *s1, const State *s2,
                                                       std::pair<State *, double> &lastValid) const
{
    // Invoke the manifold-traversing algorithm to save intermediate states
    std::vector<ProjectedStateSpace::StateType *> stateList;
    const ProjectedStateSpace::StateType *const as1 = s1->as<ProjectedStateSpace::StateType>();
    const ProjectedStateSpace::StateType *const as2 = s2->as<ProjectedStateSpace::StateType>();
    bool reached = ss_.traverseManifold(as1, as2, false, &stateList);

    // We are supposed to be able to assume that s1 is valid. However, it's not
    // on rare occasions, and I don't know why. This makes stateList empty.
    if (stateList.empty())
    {
        if (lastValid.first)
            ss_.copyState(lastValid.first, as1);
        lastValid.second = 0;
        return false;
    }

    double distanceTraveled = 0;
    for (std::size_t i = 0; i < stateList.size() - 1; i++)
    {
        if (!reached)
            distanceTraveled += ss_.distance(stateList[i], stateList[i + 1]);
        ss_.freeState(stateList[i]);
    }

    if (!reached && lastValid.first)
    {
        // Check if manifold traversal stopped early and set its final state as
        // lastValid.
        ss_.copyState(lastValid.first, stateList.back());
        // Compute the interpolation parameter of the last valid
        // state. (Although if you then interpolate, you probably won't get this
        // exact state back.)
        double approxDistanceRemaining = ss_.distance(lastValid.first, as2);
        lastValid.second = distanceTraveled / (distanceTraveled + approxDistanceRemaining);
    }

    ss_.freeState(stateList.back());
    return reached;
}

/// ProjectedStateSpace

/// Public

ompl::base::ProjectedStateSpace::ProjectedStateSpace(const StateSpace *ambientSpace, const Constraint *constraint)
  : RealVectorStateSpace(ambientSpace->getDimension())
  , si_(nullptr)
  , ss_(ambientSpace)
  , constraint_(constraint)
  , n_(ambientSpace->getDimension())
  , k_(constraint_->getManifoldDimension())
  , delta_(0.02)
  , setup_(false)
{
}

ompl::base::ProjectedStateSpace::~ProjectedStateSpace(void)
{
}

void ompl::base::ProjectedStateSpace::setup(void)
{
    if (setup_)
        return;

    if (!si_)
        throw ompl::Exception("ompl::base::ProjectedStateSpace::setup(): "
                             "Must associate a SpaceInformation object to the ProjectedStateSpace via "
                             "setStateInformation() before use.");

    setup_ = true;
    setDelta(delta_);  // This makes some setup-related calls

    RealVectorStateSpace::setup();
}

void ompl::base::ProjectedStateSpace::checkSpace(const SpaceInformation *si)
{
    if (!dynamic_cast<ProjectedStateSpace *>(si->getStateSpace().get()))
        throw ompl::Exception("ompl::base::ProjectedStateSpace(): "
                             "si needs to use an ProjectedStateSpace!");
}

void ompl::base::ProjectedStateSpace::clear(void)
{
}

void ompl::base::ProjectedStateSpace::setSpaceInformation(const SpaceInformationPtr &si)
{
    // Check that the object is valid
    if (!si.get())
        throw ompl::Exception("ompl::base::ProjectedStateSpace::setSpaceInformation(): "
                             "si is nullptr.");
    if (si->getStateSpace().get() != this)
        throw ompl::Exception("ompl::base::ProjectedStateSpace::setSpaceInformation(): "
                             "si for ProjectedStateSpace must be constructed from the same state space object.");

    // Save only a raw pointer to prevent a cycle
    si_ = si.get();
    si_->setStateValidityCheckingResolution(delta_);
}

bool ompl::base::ProjectedStateSpace::traverseManifold(const StateType *from, const StateType *to, const bool interpolate,
                      std::vector<StateType *> *stateList) const
{
    const State* previous = from;

    // number of discrete steps between a and b in the state space
    int n = validSegmentCount(from, to);

    if (n == 0) // don't divide by zero
        return true;

    const StateValidityCheckerPtr &svc = si_->getStateValidityChecker();

    double dist = distance(from, to);

    // Save a copy of the from state.
    if (stateList)
    {
        stateList->clear();
        stateList->push_back(si_->cloneState(from)->as<StateType>());
    }

    State* scratchState = allocState();
    while (true)
    {
        // The distance to travel is less than our step size.  Just declare victory
        if (dist < (delta_ + std::numeric_limits<double>::epsilon()))
        {
            State* scratchState = cloneState(to);
            if (constraint_->project(scratchState) && (interpolate || svc->isValid(scratchState)))
            {
                if (stateList)
                    stateList->push_back(si_->cloneState(scratchState)->as<StateType>());

                freeState(scratchState);
                return true;
            } else
                return false;
        }

        // Compute the parameterization for interpolation
        double t = delta_ / dist;
        RealVectorStateSpace::interpolate(previous, to, t, scratchState);

        // Project new state onto constraint manifold.  Make sure the new state is valid
        // and that it has not deviated too far from where we started
        if (!constraint_->project(scratchState) || (interpolate && !svc->isValid(scratchState)) ||
            distance(previous, scratchState) > 2.0 * delta_)
            break;

        // Check for divergence.  Divergence is declared if we are no closer to b
        // than before projection
        double newDist = distance(scratchState, to);
        if (newDist >= dist)
        {
            // Since we already collision checked this state, we might as well keep it
            if (stateList)
                stateList->push_back(si_->cloneState(scratchState)->as<StateType>());
            break;
        }

        dist = newDist;

        // No divergence; getting closer.  Store the new state
        if (stateList)
            stateList->push_back(si_->cloneState(scratchState)->as<StateType>());

        previous = scratchState;
    }

    freeState(scratchState);
    return false;
}

void ompl::base::ProjectedStateSpace::interpolate(const State *from, const State *to, const double t, State *state) const
{
    // Get the list of intermediate states along the manifold.
    std::vector<StateType *> stateList;
    bool succeeded = traverseManifold(from->as<StateType>(), to->as<StateType>(), true, &stateList);
    if (!succeeded)
        stateList.push_back(si_->cloneState(to)->as<StateType>());
    piecewiseInterpolate(stateList, t, state);
    for (StateType *state : stateList)
        freeState(state);
}

void ompl::base::ProjectedStateSpace::piecewiseInterpolate(const std::vector<StateType *> &stateList, const double t, State *state) const
{
    std::size_t n = stateList.size();
    auto d = new double[n];

    // Compute partial sums of distances between intermediate states.
    d[0] = 0;
    for (std::size_t i = 1; i < n; i++)
        d[i] = d[i - 1] + distance(stateList[i - 1], stateList[i]);

    // Find the two adjacent states that t lies between.
    std::size_t i = 0;
    double tt;
    if (d[n - 1] == 0)
    {
        // Corner case where total distance is 0.
        i = n - 1;
        tt = t;
    }
    else
    {
        while (i < n - 1 && d[i] / d[n - 1] <= t)
            i++;
        tt = t - d[i - 1] / d[n - 1];
    }

    // Linearly interpolate between these two states.
    RealVectorStateSpace::interpolate(stateList[i > 0 ? i - 1 : 0], stateList[i], tt, state);
    delete[] d;

}

ompl::base::StateSamplerPtr ompl::base::ProjectedStateSpace::allocDefaultStateSampler(void) const
{
    return StateSamplerPtr(new ProjectedStateSampler(*this));
}

ompl::base::State *ompl::base::ProjectedStateSpace::allocState() const
{
    return RealVectorStateSpace::allocState();
}

void ompl::base::ProjectedStateSpace::freeState(State *state) const
{
    RealVectorStateSpace::freeState(state);
}

void ompl::base::ProjectedStateSpace::dumpGraph(const PlannerData::Graph &graph, std::ostream &out, const bool asIs) const
{
    std::stringstream v, f;
    std::size_t vcount = 0;
    std::size_t fcount = 0;

    BGL_FORALL_EDGES(edge, graph, PlannerData::Graph)
    {
        std::vector<StateType *> stateList;
        const State *const source = boost::get(vertex_type, graph, boost::source(edge, graph))->getState();
        const State *const target = boost::get(vertex_type, graph, boost::target(edge, graph))->getState();

        if (!asIs)
            traverseManifold(source->as<StateType>(), target->as<StateType>(), true, &stateList);
        if (asIs || stateList.size() == 1)
        {
            v << constraint_->toVector(source).transpose() << "\n";
            v << constraint_->toVector(target).transpose() << "\n";
            v << constraint_->toVector(source).transpose() << "\n";
            vcount += 3;
            f << 3 << " " << vcount - 3 << " " << vcount - 2 << " " << vcount - 1 << "\n";
            fcount++;
            for (StateType *state : stateList)
                freeState(state);
            continue;
        }
        StateType *to, *from = stateList[0];
        v << constraint_->toVector(from).transpose() << "\n";
        vcount++;
        bool reset = true;
        for (std::size_t i = 1; i < stateList.size(); i++)
        {
            to = stateList[i];
            from = stateList[i - 1];
            v << constraint_->toVector(to).transpose() << "\n";
            v << constraint_->toVector(from).transpose() << "\n";
            vcount += 2;
            f << 3 << " " << (reset ? vcount - 3 : vcount - 4) << " " << vcount - 2 << " " << vcount - 1 << "\n";
            fcount++;
            freeState(stateList[i - 1]);
            reset = false;
        }
        freeState(stateList.back());
    }

    out << "ply\n";
    out << "format ascii 1.0\n";
    out << "element vertex " << vcount << "\n";
    out << "property float x\n";
    out << "property float y\n";
    out << "property float z\n";
    out << "element face " << fcount << "\n";
    out << "property list uint uint vertex_index\n";
    out << "end_header\n";
    out << v.str() << f.str();
}

void ompl::base::ProjectedStateSpace::dumpPath(ompl::geometric::PathGeometric &path, std::ostream &out,
                                            const bool asIs) const
{
    std::stringstream v, f;
    std::size_t vcount = 0;
    std::size_t fcount = 0;

    const std::vector<State *> &waypoints = path.getStates();
    for (std::size_t i = 0; i < waypoints.size() - 1; i++)
    {
        std::vector<StateType *> stateList;
        const State *const source = waypoints[i];
        const State *const target = waypoints[i + 1];

        if (!asIs)
            traverseManifold(source->as<StateType>(), target->as<StateType>(), true, &stateList);
        if (asIs || stateList.size() == 1)
        {
            v << constraint_->toVector(source).transpose() << "\n";
            v << constraint_->toVector(target).transpose() << "\n";
            v << constraint_->toVector(source).transpose() << "\n";
            vcount += 3;
            f << 3 << " " << vcount - 3 << " " << vcount - 2 << " " << vcount - 1 << "\n";
            fcount++;
            for (StateType *state : stateList)
                freeState(state);
            continue;
        }
        StateType *to, *from = stateList[0];
        v << constraint_->toVector(from).transpose() << "\n";
        vcount++;
        bool reset = true;
        for (std::size_t i = 1; i < stateList.size(); i++)
        {
            to = stateList[i];
            from = stateList[i - 1];
            v << constraint_->toVector(to).transpose() << "\n";
            v << constraint_->toVector(from).transpose() << "\n";
            vcount += 2;
            f << 3 << " " << (reset ? vcount - 3 : vcount - 4) << " " << vcount - 2 << " " << vcount - 1 << "\n";
            fcount++;
            freeState(stateList[i - 1]);
            reset = false;
        }
        freeState(stateList.back());
    }

    out << "ply\n";
    out << "format ascii 1.0\n";
    out << "element vertex " << vcount << "\n";
    out << "property float x\n";
    out << "property float y\n";
    out << "property float z\n";
    out << "element face " << fcount << "\n";
    out << "property list uint uint vertex_index\n";
    out << "end_header\n";
    out << v.str() << f.str();
}
