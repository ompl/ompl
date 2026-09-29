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

#include "ompl/base/spaces/flat/FlatStateSpace.h"

#include "ompl/base/Planner.h"
#include "ompl/base/ProjectionEvaluator.h"
#include "ompl/base/StateSpaceTypes.h"
#include "ompl/base/objectives/FlatEffortObjective.h"
#include "ompl/util/Console.h"
#include "ompl/util/Exception.h"

#include <boost/container/small_vector.hpp>

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

    /** \brief The largest flat output dimension whose scratch space stays on the stack. */
    constexpr unsigned int STACK_DIMENSION = 16u;

    /** \brief Room for one flat output, held on the stack up to \ref STACK_DIMENSION dimensions. */
    using OutputBuffer = boost::container::small_vector<double, STACK_DIMENSION>;

    /** \brief Room for both endpoints of an edge, containing the start and end flat states.

        Two flat states of order \e k hold 2 \e k values per flat output dimension, so this stays on the
        stack up to \ref STACK_DIMENSION dimensions at order 2.
    */
    using EndpointsBuffer = boost::container::small_vector<double, 4u * STACK_DIMENSION>;
}  // namespace

namespace ompl::base
{
    FlatStateSpace::FlatStateSpace(const StateSpacePtr &output, unsigned int order, FlatChartPtr chart)
      : steering_(order)
    {
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
      : steering_(static_cast<unsigned int>(derivatives.size()) + 1u)
    {
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

    std::optional<MinimumEffortSteering::OptimalDuration>
    FlatStateSpace::optimalEdge(const State *from, const State *to, Eigen::Ref<Eigen::MatrixXd> initial,
                                Eigen::Ref<Eigen::MatrixXd> terminal) const
    {
        /** \brief A change in the cheapest winding of one axis, which the walk sorts by time. */
        struct Change
        {
            /** \brief The duration from which the new winding is cheapest. */
            double time;

            /** \brief The axis changing winding. */
            unsigned int axis;

            /** \brief The number of whole periods the new winding adds to the chart's displacement. */
            double winding;

            bool operator<(const Change &other) const
            {
                return time < other.time || (time == other.time && axis < other.axis);
            }
        };

        const auto *start = from->as<CompoundState>();
        const auto *finish = to->as<CompoundState>();

        // The chart writes into a contiguous vector and a row of a flat state isn't one.
        OutputBuffer displacement(outputDimension_);
        Eigen::Map<Eigen::VectorXd> displacementVector(displacement.data(), outputDimension_);
        chart_->difference(start->components[0], finish->components[0], displacementVector);

        // in all charts, we center about the start
        initial.row(0).setZero();
        terminal.row(0) = displacementVector.transpose();

        for (unsigned int level = 1; level < componentCount_; ++level)
        {
            initial.row(level) = Eigen::Map<const Eigen::RowVectorXd>(
                start->components[level]->as<RealVectorStateSpace::StateType>()->values, outputDimension_);
            terminal.row(level) = Eigen::Map<const Eigen::RowVectorXd>(
                finish->components[level]->as<RealVectorStateSpace::StateType>()->values, outputDimension_);
        }

        std::optional<MinimumEffortSteering::OptimalDuration> cheapest = steering_.optimalDuration(initial, terminal);
        if (!cheapest.has_value())
            return cheapest;

        // On a wrapping axis, the flat output can reach the goal after any whole number of extra windings.
        // The winding with the shortest displacement isn't always the cheapest, such as when the flat output
        // starts out moving fast.
        //
        // At a fixed duration, the cost splits into one quadratic per axis in that axis's displacement.
        // Each quadratic is least at the ideal winding, which is usually fractional, so the cheapest motion at
        // that duration picks the whole winding nearest the ideal winding on every axis.
        // As the duration grows, that combination of windings changes only at the times windingChanges reports.
        // The cheapest motion therefore uses one of the combinations between those times, and the loop prices
        // each one at its best duration.
        // Every motion costs at least rho times its duration, so a change later than the cheapest cost so far
        // divided by rho can't lead to anything cheaper, and the loop stops there.
        const std::vector<double> &periods = chart_->getPeriods();
        std::vector<Change> changes;
        const double horizon = cheapest->cost / steering_.getRho();
        for (unsigned int axis = 0; axis < outputDimension_; ++axis)
            if (periods[axis] > 0.)
                for (const MinimumEffortSteering::WindingChange &change :
                     steering_.windingChanges(initial, terminal, axis, periods[axis], horizon))
                    changes.push_back(Change{change.time, axis, change.winding});

        if (changes.empty())
            return cheapest;

        std::sort(changes.begin(), changes.end());
        const OutputBuffer nearest(terminal.row(0).begin(), terminal.row(0).end());
        OutputBuffer cheapestOutput = nearest;
        for (const Change &change : changes)
        {
            if (change.time > cheapest->cost / steering_.getRho())
                break;

            terminal(0, change.axis) = nearest[change.axis] + change.winding * periods[change.axis];
            const std::optional<MinimumEffortSteering::OptimalDuration> candidate =
                steering_.optimalDuration(initial, terminal);
            if (candidate.has_value() && candidate->cost < cheapest->cost)
            {
                cheapest = candidate;
                std::copy(terminal.row(0).begin(), terminal.row(0).end(), cheapestOutput.begin());
            }
        }

        terminal.row(0) = Eigen::Map<const Eigen::RowVectorXd>(cheapestOutput.data(), outputDimension_);
        return cheapest;
    }

    std::optional<FlatMotion> FlatStateSpace::steer(const State *from, const State *to) const
    {
        EndpointsBuffer buffer(2u * componentCount_ * outputDimension_);
        Eigen::Map<Eigen::MatrixXd> initial(buffer.data(), componentCount_, outputDimension_);
        Eigen::Map<Eigen::MatrixXd> terminal(buffer.data() + initial.size(), componentCount_, outputDimension_);
        const std::optional<MinimumEffortSteering::OptimalDuration> cheapest = optimalEdge(from, to, initial, terminal);
        if (!cheapest.has_value())
            return {};
        return steering_.motion(initial, terminal, cheapest->duration);
    }

    double FlatStateSpace::steeringCost(const State *from, const State *to) const
    {
        // Nobody has to move to get from a flat state to itself, whatever velocity it carries.
        if (equalStates(from, to))
            return 0.;

        EndpointsBuffer buffer(2u * componentCount_ * outputDimension_);
        Eigen::Map<Eigen::MatrixXd> initial(buffer.data(), componentCount_, outputDimension_);
        Eigen::Map<Eigen::MatrixXd> terminal(buffer.data() + initial.size(), componentCount_, outputDimension_);
        const std::optional<MinimumEffortSteering::OptimalDuration> cheapest = optimalEdge(from, to, initial, terminal);
        return cheapest.has_value() ? cheapest->cost : std::numeric_limits<double>::infinity();
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
