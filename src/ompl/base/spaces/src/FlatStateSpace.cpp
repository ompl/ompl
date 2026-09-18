/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Rice University
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

/* Author: Clayton Ramsey */

#include "ompl/base/spaces/FlatStateSpace.h"

#include "ompl/base/Planner.h"
#include "ompl/base/ProjectionEvaluator.h"
#include "ompl/base/StateSpaceTypes.h"
#include "ompl/base/objectives/FlatEffortObjective.h"
#include "ompl/util/Console.h"
#include "ompl/util/Exception.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>
#include <string>

namespace
{
    /** \brief The name each distance type answers to in a parameter set. */
    const char *const DISTANCE_TYPE_NAMES[] = {"flat_state_metric", "trajectory_cost"};

    /** \brief Planners that hand back a path with an edge on it they never checked in the direction the
        path runs through it.

        A bidirectional search grows one of its trees from the goal, and a roadmap search comes out of the
        graph whichever way round each edge happens to sit.
        Either way the planner checked a curve the path doesn't follow.
        Planners that pick their check direction from the tree they're growing stay off this list, so
        RRTConnect and BiTRRT aren't on it.
        Neither are the ones that already warn about asymmetric spaces themselves.
    */
    const std::set<std::string> REVERSING_PLANNERS = {"BFMT",        "BiEST",     "BiRLRT", "BKPIECE1", "LazyPRM",
                                                      "LazyPRMstar", "LBKPIECE1", "PDST",   "PRM",      "PRMstar",
                                                      "pSBL",        "SBL",       "SPARS",  "SPARStwo"};
}  // namespace

namespace ompl::base
{
    FlatStateSpace::FlatStateSpace(const StateSpacePtr &output, unsigned int order)
    {
        if (order != 2u)
            throw Exception("FlatStateSpace only supports order 2");

        checkComponent(output, 0u, "flat output");
        outputDimension_ = output->getDimension();

        setName("Flat" + output->getName());
        type_ = STATE_SPACE_FLAT;

        addSubspace(output, 1.);
        for (unsigned int level = 1; level < order; ++level)
            addSubspace(std::make_shared<RealVectorStateSpace>(outputDimension_), 1.);
        lock();

        setDefaultWeights();
        declareParams();
    }

    FlatStateSpace::FlatStateSpace(const StateSpacePtr &output, const std::vector<StateSpacePtr> &derivatives)
    {
        if (derivatives.size() != 1u)
            throw Exception("FlatStateSpace only supports order 2, so it needs exactly one derivative space");

        checkComponent(output, 0u, "flat output");
        outputDimension_ = output->getDimension();

        setName("Flat" + output->getName());
        type_ = STATE_SPACE_FLAT;

        addSubspace(output, 1.);
        for (const StateSpacePtr &derivative : derivatives)
        {
            checkComponent(derivative, outputDimension_, "derivative level");
            addSubspace(derivative, 1.);
        }
        lock();

        setDefaultWeights();
        declareParams();
    }

    void FlatStateSpace::declareParams()
    {
        params_.declareParam<double>("rho", [this](double value) { setRho(value); }, [this] { return getRho(); });

        params_.declareParam<std::string>(
            "distance_type",
            [this](const std::string &value)
            {
                if (value == DISTANCE_TYPE_NAMES[FLAT_STATE_METRIC])
                    setDistanceType(FLAT_STATE_METRIC);
                else if (value == DISTANCE_TYPE_NAMES[TRAJECTORY_COST])
                    setDistanceType(TRAJECTORY_COST);
                else
                    throw std::invalid_argument("a flat state space measures distance as " +
                                                std::string(DISTANCE_TYPE_NAMES[FLAT_STATE_METRIC]) + " or " +
                                                std::string(DISTANCE_TYPE_NAMES[TRAJECTORY_COST]));
            },
            [this] { return std::string(DISTANCE_TYPE_NAMES[distanceType_]); });
        params_["distance_type"].setRangeSuggestion(std::string(DISTANCE_TYPE_NAMES[FLAT_STATE_METRIC]) + "," +
                                                    DISTANCE_TYPE_NAMES[TRAJECTORY_COST]);
    }

    void FlatStateSpace::checkComponent(const StateSpacePtr &space, unsigned int dimension, const char *role)
    {
        if (space == nullptr)
            throw Exception(std::string("FlatStateSpace needs a ") + role + " space");
        if (dynamic_cast<const RealVectorStateSpace *>(space.get()) == nullptr)
            throw Exception(std::string("FlatStateSpace needs a ") + role +
                            " space deriving from RealVectorStateSpace");
        if (space->getDimension() < 1u)
            throw Exception(std::string("FlatStateSpace needs a ") + role + " space of at least one dimension");
        if (dimension > 0u && space->getDimension() != dimension)
            throw Exception(std::string("FlatStateSpace needs its ") + role +
                            " space to match the flat output dimension");
    }

    void FlatStateSpace::setDefaultWeights()
    {
        for (unsigned int i = 0; i < componentCount_; ++i)
        {
            const double extent = components_[i]->getMaximumExtent();
            setSubspaceWeight(i, extent > std::numeric_limits<double>::epsilon() ? 1. / extent : 1.);
        }
        defaultWeights_ = weights_;
    }

    void FlatStateSpace::setDerivativeBound(unsigned int level, double bound)
    {
        if (level < 1u || level >= componentCount_)
            throw Exception("FlatStateSpace derivative levels run from 1 through the order minus one");
        if (!(bound > 0.))
            throw Exception("FlatStateSpace needs a positive derivative bound");

        getDerivativeSpace(level)->setBounds(-bound, bound);
    }

    void FlatStateSpace::toFlatState(const State *state, Eigen::Ref<Eigen::MatrixXd> flatState) const
    {
        const auto *compound = state->as<CompoundState>();
        for (unsigned int level = 0; level < componentCount_; ++level)
        {
            const double *values = compound->components[level]->as<RealVectorStateSpace::StateType>()->values;
            for (unsigned int axis = 0; axis < outputDimension_; ++axis)
                flatState(level, axis) = values[axis];
        }
    }

    void FlatStateSpace::fromFlatState(const Eigen::Ref<const Eigen::MatrixXd> &flatState, State *state) const
    {
        auto *compound = state->as<CompoundState>();
        for (unsigned int level = 0; level < componentCount_; ++level)
        {
            double *values = compound->components[level]->as<RealVectorStateSpace::StateType>()->values;
            for (unsigned int axis = 0; axis < outputDimension_; ++axis)
                values[axis] = flatState(level, axis);
        }
    }

    std::optional<FlatMotion> FlatStateSpace::steer(const State *from, const State *to) const
    {
        Eigen::MatrixXd start(componentCount_, outputDimension_);
        Eigen::MatrixXd finish(componentCount_, outputDimension_);
        toFlatState(from, start);
        toFlatState(to, finish);
        return steering_.steer(start, finish);
    }

    double FlatStateSpace::steeringCost(const State *from, const State *to) const
    {
        // Nobody has to move to get from a flat state to itself, whatever velocity it carries.
        if (equalStates(from, to))
            return 0.;

        const std::optional<FlatMotion> motion = steer(from, to);
        return motion.has_value() ? steering_.cost(*motion) : std::numeric_limits<double>::infinity();
    }

    double FlatStateSpace::distance(const State *state1, const State *state2) const
    {
        if (distanceType_ == FLAT_STATE_METRIC)
            return CompoundStateSpace::distance(state1, state2);
        return steeringCost(state1, state2);
    }

    void FlatStateSpace::interpolate(const State *from, const State *to, double t, State *state) const
    {
        bool firstTime = true;
        std::optional<FlatMotion> motion;
        interpolate(from, to, t, firstTime, motion, state);
    }

    void FlatStateSpace::interpolate(const State *from, const State *to, double t, bool &firstTime,
                                     std::optional<FlatMotion> &motion, State *state) const
    {
        if (t <= 0.)
        {
            if (from != state)
                copyState(state, from);
            return;
        }
        if (t >= 1.)
        {
            if (to != state)
                copyState(state, to);
            return;
        }

        // Steering reads both flat states before anything is written, so state can alias one of them.
        if (firstTime)
        {
            motion = steer(from, to);
            firstTime = false;
        }

        if (!motion.has_value())
        {
            if (from != state)
                copyState(state, from);
            return;
        }

        interpolate(*motion, t, state);
    }

    void FlatStateSpace::interpolate(const FlatMotion &motion, double t, State *state) const
    {
        // Each component holds its values contiguously, so the motion writes its derivative levels
        // straight into the state and nothing is allocated per sample.
        const double time = t * motion.duration();
        auto *compound = state->as<CompoundState>();
        for (unsigned int level = 0; level < componentCount_; ++level)
        {
            double *values = compound->components[level]->as<RealVectorStateSpace::StateType>()->values;
            Eigen::Map<Eigen::VectorXd> output(values, outputDimension_);
            motion.evaluate(time, level, output);
        }
    }

    unsigned int FlatStateSpace::validSegmentCount(const State *state1, const State *state2) const
    {
        const std::optional<FlatMotion> motion = steer(state1, state2);
        return motion.has_value() ? validSegmentCount(*motion) : 1u;
    }

    unsigned int FlatStateSpace::validSegmentCount(const FlatMotion &motion) const
    {
        const double span = motion.duration() * motion.peakSpeed();
        const auto count = static_cast<unsigned int>(std::ceil(span / longestValidSegment_));
        return std::max(1u, longestValidSegmentCountFactor_ * count);
    }

    void FlatStateSpace::sanityChecks() const
    {
        unsigned int flags = ~0u;
        if (distanceType_ == TRAJECTORY_COST)
        {
            // A cost isn't a length.
            // It runs one way, it climbs with the effort the edge spends rather than staying inside the
            // extent of the space, and it jumps either side of coincidence, since a flat state carrying a
            // velocity has to fly a whole loop to get back to where it already is.
            // The checks that read distance as a length go with it.
            flags &= ~(STATESPACE_DISTANCE_SYMMETRIC | STATESPACE_TRIANGLE_INEQUALITY | STATESPACE_DISTANCE_BOUND |
                       STATESPACE_INTERPOLATION);
        }

        sanityChecks(std::numeric_limits<double>::epsilon(), std::numeric_limits<float>::epsilon(), flags);
    }

    void FlatStateSpace::checkPlanner(const Planner *planner) const
    {
        if (planner == nullptr)
            return;

        const std::string &name = planner->getName();
        if (REVERSING_PLANNERS.count(name) > 0u)
            OMPL_WARN("%s doesn't check every edge in the direction its path runs through it, and edges in "
                      "%s run forward in time. The paths it returns fail PathGeometric::check().",
                      name.c_str(), getName().c_str());

        if (!planner->getSpecs().optimizingPaths)
            return;

        const ProblemDefinitionPtr &definition = planner->getProblemDefinition();
        const OptimizationObjectivePtr objective =
            definition == nullptr ? nullptr : definition->getOptimizationObjective();
        if (dynamic_cast<const FlatEffortObjective *>(objective.get()) == nullptr)
            OMPL_WARN("%s optimizes paths through %s without a FlatEffortObjective, so it minimizes a sum of "
                      "flat state distances rather than what traversing the path costs.",
                      name.c_str(), getName().c_str());
    }

    void FlatStateSpace::registerProjections()
    {
        registerDefaultProjection(std::make_shared<SubspaceProjectionEvaluator>(this, 0u));
    }

    void FlatStateSpace::setup()
    {
        // Bounds usually arrive after construction, so the weights get another look here.
        // A caller who set weights of their own keeps them.
        if (weights_ == defaultWeights_)
            setDefaultWeights();

        CompoundStateSpace::setup();
    }

    State *FlatStateSpace::allocState() const
    {
        auto *state = new StateType();
        allocStateComponents(state);
        return state;
    }
}  // namespace ompl::base
