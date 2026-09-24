/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2026, Rice University
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

/* Author: Clayton W. Ramsey */

#include "ompl/base/spaces/FlatStateSpace.h"

#include "ompl/base/Planner.h"
#include "ompl/base/ProjectionEvaluator.h"
#include "ompl/base/StateSpaceTypes.h"
#include "ompl/base/objectives/FlatEffortObjective.h"
#include "ompl/util/Console.h"
#include "ompl/util/Exception.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>

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
    FlatStateSpace::FlatStateSpace(const StateSpacePtr &output, unsigned int order, FlatChartPtr chart)
    {
        if (order != 2u)
            throw Exception("FlatStateSpace only supports order 2");

        chart_ = chart == nullptr ? allocFlatChart(output) : std::move(chart);
        checkOutput(output, chart_);
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

    FlatStateSpace::FlatStateSpace(const StateSpacePtr &output, const std::vector<StateSpacePtr> &derivatives,
                                   FlatChartPtr chart)
    {
        if (derivatives.size() != 1u)
            throw Exception("FlatStateSpace only supports order 2, so it needs exactly one derivative space");

        chart_ = chart == nullptr ? allocFlatChart(output) : std::move(chart);
        checkOutput(output, chart_);
        outputDimension_ = output->getDimension();

        setName("Flat" + output->getName());
        type_ = STATE_SPACE_FLAT;

        addSubspace(output, 1.);
        for (const StateSpacePtr &derivative : derivatives)
        {
            checkDerivative(derivative, outputDimension_);
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

    void FlatStateSpace::checkOutput(const StateSpacePtr &output, const FlatChartPtr &chart)
    {
        if (output == nullptr)
            throw Exception("FlatStateSpace needs a flat output space");
        if (output->getDimension() < 1u)
            throw Exception("FlatStateSpace needs a flat output space of at least one dimension");
        if (chart->getDimension() != output->getDimension())
            throw Exception("FlatStateSpace needs a chart with one coordinate per flat output dimension");
    }

    void FlatStateSpace::checkDerivative(const StateSpacePtr &derivative, unsigned int dimension)
    {
        if (derivative == nullptr)
            throw Exception("FlatStateSpace needs a derivative level space");
        if (dynamic_cast<const RealVectorStateSpace *>(derivative.get()) == nullptr)
            throw Exception("FlatStateSpace needs a derivative level space deriving from RealVectorStateSpace");
        if (derivative->getDimension() != dimension)
            throw Exception("FlatStateSpace needs its derivative level space to match the flat output dimension");
    }

    double *FlatStateSpace::outputValue(State *state, unsigned int index) const
    {
        return const_cast<double *>(outputValue(static_cast<const State *>(state), index));
    }

    const double *FlatStateSpace::outputValue(const State *state, unsigned int index) const
    {
        const StateSpacePtr &output = components_[0];
        const State *flatOutput = state->as<CompoundState>()->components[0];
        const double *value = output->getValueAddressAtIndex(flatOutput, index);
        if (value == nullptr || output->getValueAddressAtIndex(flatOutput, outputDimension_) != nullptr)
            throw Exception("FlatStateSpace can only read a flat state into a matrix when the flat output space "
                            "carries one value per dimension");
        return value;
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
        for (unsigned int axis = 0; axis < outputDimension_; ++axis)
            flatState(0, axis) = *outputValue(state, axis);

        const auto *compound = state->as<CompoundState>();
        for (unsigned int level = 1; level < componentCount_; ++level)
        {
            const double *values = compound->components[level]->as<RealVectorStateSpace::StateType>()->values;
            for (unsigned int axis = 0; axis < outputDimension_; ++axis)
                flatState(level, axis) = values[axis];
        }
    }

    void FlatStateSpace::fromFlatState(const Eigen::Ref<const Eigen::MatrixXd> &flatState, State *state) const
    {
        for (unsigned int axis = 0; axis < outputDimension_; ++axis)
            *outputValue(state, axis) = flatState(0, axis);

        auto *compound = state->as<CompoundState>();
        for (unsigned int level = 1; level < componentCount_; ++level)
        {
            double *values = compound->components[level]->as<RealVectorStateSpace::StateType>()->values;
            for (unsigned int axis = 0; axis < outputDimension_; ++axis)
                values[axis] = flatState(level, axis);
        }
    }

    std::optional<FlatMotion> FlatStateSpace::steer(const State *from, const State *to) const
    {
        const auto *start = from->as<CompoundState>();
        const auto *finish = to->as<CompoundState>();

        Eigen::MatrixXd initial(componentCount_, outputDimension_);
        Eigen::MatrixXd terminal(componentCount_, outputDimension_);
        Eigen::VectorXd displacement(outputDimension_);
        chart_->difference(start->components[0], finish->components[0], displacement);

        // in all charts, we center about the start
        initial.row(0).setZero();
        terminal.row(0) = displacement.transpose();

        for (unsigned int level = 1; level < componentCount_; ++level)
        {
            initial.row(level) = Eigen::Map<const Eigen::RowVectorXd>(
                start->components[level]->as<RealVectorStateSpace::StateType>()->values, outputDimension_);
            terminal.row(level) = Eigen::Map<const Eigen::RowVectorXd>(
                finish->components[level]->as<RealVectorStateSpace::StateType>()->values, outputDimension_);
        }

        std::optional<FlatMotion> motion = steering_.steer(initial, terminal);
        if (!motion.has_value())
            return motion;

        // A coordinate that wraps reaches the target at every whole number of periods from the nearest
        // copy, and the nearest copy isn't the cheapest when the flat output is already moving fast.
        // Taking it anyway would break continued interpolation, since the tail of a motion swinging more
        // than a half period round has a different nearest copy from the motion as a whole.
        //
        // Over a fixed duration T, an axis with velocities v0 and vf and displacement o costs
        //     12 (o - (v0 + vf) T / 2)^2 / T^3 + (v0 - vf)^2 / T,
        // so the cheapest copy at T is the one nearest (v0 + vf) T / 2, and it changes only where that
        // point passes halfway between two copies.
        // The cheapest motion overall takes the cheapest copies, so walking those
        // crossings in order of time and steering to each new set of copies finds it.
        // A motion costs at least rho times its duration, so crossings later than the cheapest cost so
        // far over rho can't lead anywhere cheaper and the walk stops there.
        const std::vector<double> &periods = chart_->getPeriods();
        const auto moving = [&](unsigned int axis)
        { return periods[axis] > 0. && initial(1, axis) + terminal(1, axis) != 0.; };
        bool anyMoving = false;
        for (unsigned int axis = 0; axis < outputDimension_ && !anyMoving; ++axis)
            anyMoving = moving(axis);
        if (!anyMoving)
            return motion;

        std::vector<std::pair<double, unsigned int>> crossings;
        double cost = steering_.cost(*motion);
        const double horizon = cost / steering_.getRho();
        for (unsigned int axis = 0; axis < outputDimension_; ++axis)
        {
            if (!moving(axis))
                continue;
            const double period = periods[axis];
            const double sum = initial(1, axis) + terminal(1, axis);

            // The nearest copy starts out as the one the chart picked, which lies within half a period of
            // the origin, and moves a period in the direction of travel at each crossing.
            const double direction = sum > 0. ? 1. : -1.;
            for (double halfway = terminal(0, axis) + 0.5 * direction * period;; halfway += direction * period)
            {
                // Written this way round, a velocity of NaN ends the walk rather than spinning forever.
                const double time = 2. * halfway / sum;
                if (!(time <= horizon))
                    break;
                if (time > 0.)
                    crossings.emplace_back(time, axis);
            }
        }

        std::sort(crossings.begin(), crossings.end());
        for (const auto &[time, axis] : crossings)
        {
            if (time > cost / steering_.getRho())
                break;

            terminal(0, axis) += initial(1, axis) + terminal(1, axis) > 0. ? periods[axis] : -periods[axis];
            std::optional<FlatMotion> candidate = steering_.steer(initial, terminal);
            if (!candidate.has_value())
                continue;

            const double candidateCost = steering_.cost(*candidate);
            if (candidateCost < cost)
            {
                motion = std::move(candidate);
                cost = candidateCost;
            }
        }

        return motion;
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

        interpolate(from, *motion, t, state);
    }

    void FlatStateSpace::interpolate(const State *from, const FlatMotion &motion, double t, State *state) const
    {
        // Each derivative component holds its values contiguously, so the motion writes those straight
        // into the state.
        const double time = t * motion.duration();
        auto *compound = state->as<CompoundState>();
        for (unsigned int level = 1; level < componentCount_; ++level)
        {
            // derivatives are always real-valued, so this is sound
            double *values = compound->components[level]->as<RealVectorStateSpace::StateType>()->values;
            Eigen::Map<Eigen::VectorXd> derivative(values, outputDimension_);
            motion.evaluate(time, level, derivative);
        }

        // for states less than 16 dimensions, allocate on the stack to go fast :)
        constexpr unsigned int STACK_DIMENSION = 16u;
        std::array<double, STACK_DIMENSION> stack;
        std::vector<double> heap;
        double *buffer = stack.data();
        if (outputDimension_ > STACK_DIMENSION)
        {
            heap.resize(outputDimension_);
            buffer = heap.data();
        }

        Eigen::Map<Eigen::VectorXd> coordinates(buffer, outputDimension_);
        motion.evaluate(time, 0u, coordinates);
        chart_->advance(from->as<CompoundState>()->components[0], coordinates, compound->components[0]);
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
        if (components_[0]->hasDefaultProjection())
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
