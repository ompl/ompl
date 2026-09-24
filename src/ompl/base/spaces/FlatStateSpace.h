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

#ifndef OMPL_BASE_SPACES_FLAT_STATE_SPACE_
#define OMPL_BASE_SPACES_FLAT_STATE_SPACE_

#include "ompl/base/StateSpace.h"
#include "ompl/base/spaces/FlatChart.h"
#include "ompl/base/spaces/FlatMotion.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"

#include <vector>

namespace ompl::base
{
    class Planner;

    /// @cond IGNORE
    OMPL_CLASS_FORWARD(FlatStateSpace);
    /// @endcond

    /** \brief A state space for planning with a differentially flat system.

        A state here is a flat state, meaning the flat output of the system stacked with its derivatives up
        to the order minus one.
        Planners treat that stack as one point, and this space joins two of them with the polynomial that
        spends the least control effort getting from one to the other, so every planner in
        ompl::geometric steers through flat motions without knowing it.

        The space is a compound of the flat output space the caller supplies at component 0 and one
        ompl::base::RealVectorStateSpace per derivative level from 1 through the order minus one.
        Bounds on those components are the velocity and acceleration limits of the system.

        Each edge between states in a flat state space is a polynomial, which is called a steering polynomial.
        To handle non-Euclidean state spaces, the polynomials live inside of a ompl::base::FlatChart, which provides
        a coordinate system for constructing these polynomials.
        Euclidean spaces, SO(2), and compounds of them, such as SE(2) and tori, get an exact chart without
        being asked.

        Distance defaults to the flat output distance plus a weighted Euclidean distance per derivative
        level, which is symmetric, satisfies the triangle inequality, and needs no steering solve, so
        nearest-neighbor structures that prune on those properties stay usable.
        \ref setDistanceType enables other distances, such as edge traversal cost.

        Edges run forward in time, so \ref hasSymmetricInterpolate reports false.
        Planners that check an edge one way and then walk it the other return paths this space rejects, and
        \ref checkPlanner warns by name when one of them gets set up here.

        @par External documentation
        T. Duong, C. W. Ramsey, Z. Kingston, W. Thomason, and L. E. Kavraki, Ultrafast sampling-based kinodynamic
        planning via differential flatness, in <em>IEEE Transactions on Robotics</em>, 2026.
        [[PDF]](https://arxiv.org/abs/2603.16059)
    */
    class FlatStateSpace : public CompoundStateSpace
    {
    public:
        /** \brief A flat output stacked with its derivatives. */
        class StateType : public CompoundStateSpace::StateType
        {
        public:
            /** \brief The flat output. */
            State *output()
            {
                return components[0];
            }

            /** \brief The flat output. */
            const State *output() const
            {
                return components[0];
            }

            /** \brief Derivative level \e level of the flat output, counting the flat output itself as
                level 0, so level 1 is the velocity.
                The level has to be between 1 and the order minus one. */
            RealVectorStateSpace::StateType *derivative(unsigned int level)
            {
                return components[level]->as<RealVectorStateSpace::StateType>();
            }

            /** \brief Derivative level \e level of the flat output, counting the flat output itself as
                level 0, so level 1 is the velocity.
                The level has to be between 1 and the order minus one. */
            const RealVectorStateSpace::StateType *derivative(unsigned int level) const
            {
                return components[level]->as<RealVectorStateSpace::StateType>();
            }
        };

        /** \brief The choice of measurement for \ref distance.
         *
         *  If you want to plan using a distance not available in this enumeration, you shoud subclass FlatStateSpace.
         */
        enum DistanceType
        {
            /** \brief The flat output distance plus a weighted Euclidean distance per derivative level.

                It's symmetric, it satisfies the triangle inequality, and it costs no steering solve, so
                nearest-neighbor structures that prune on those properties stay usable.
            */
            FLAT_STATE_METRIC,

            /** \brief The cost of traversing a steering motion, meaning its control effort plus rho
                times its duration.

                It costs a steering solve per query and it runs one way, so nearest-neighbor structures
                fall back to a linear scan.
                Reach for it when a planner keys its search on edge cost and you'd rather it did that
                directly than through an optimization objective.
            */
            TRAJECTORY_COST
        };

        /** \brief Build a flat state space over \e output holding \e order derivative levels, counting the
            flat output itself.

            The derivative components come out unbounded, so call \ref setDerivativeBound for each level
            before planning.
            The order has to be 2.
            The steering polynomials between states run in \e chart, which has to carry one coordinate per
            dimension of \e output.
            If \e chart is unspecified, ompl::base::allocFlatChart will generate a default chart based on \e output.
        */
        FlatStateSpace(const StateSpacePtr &output, unsigned int order, FlatChartPtr chart = nullptr);

        /** \brief Build a flat state space over \e output whose derivative levels are the spaces in
            \e derivatives, so the order is one more than their count.

            Using this constructor, client code can implement their own bounds, instead of relying on
            FlatStateSpace to build them.
            Each derivative space has to derive from ompl::base::RealVectorStateSpace and match the dimension of
            \e output, which keeps the value layout the steering math reads, and everything past that is
            the caller's to decide.
            Overriding enforceBounds, satisfiesBounds, and allocDefaultStateSampler on a derivative
            component reaches enforcement, checking, and sampling through compound delegation.
            The steering polynomials between states run in \e chart, which has to carry one coordinate per
            dimension of \e output.
            If \e chart is unspecified, ompl::base::allocFlatChart will generate a default chart based on \e output.
        */
        FlatStateSpace(const StateSpacePtr &output, const std::vector<StateSpacePtr> &derivatives,
                       FlatChartPtr chart = nullptr);

        ~FlatStateSpace() override = default;

        /** \brief The number of derivative levels a state carries, counting the flat output itself. */
        unsigned int getOrder() const
        {
            return componentCount_;
        }

        /** \brief The number of flat output dimensions. */
        unsigned int getOutputDimension() const
        {
            return outputDimension_;
        }

        /** \brief The flat output space supplied at construction. */
        const StateSpacePtr &getOutputSpace() const
        {
            return components_[0];
        }

        /** \brief The chart containing steering polynomials. */
        const FlatChartPtr &getChart() const
        {
            return chart_;
        }

        /** \brief The component holding derivative level \e level, which has to be between 1 and the order
            minus one. */
        RealVectorStateSpace *getDerivativeSpace(unsigned int level) const
        {
            return components_[level]->as<RealVectorStateSpace>();
        }

        /** \brief Bound every dimension of derivative level \e level to plus and minus \e bound.

            The level has to be between 1 and the order minus one and the bound has to be positive.
        */
        void setDerivativeBound(unsigned int level, double bound);

        /** \brief Set the time penalty charged per unit duration, which has to be positive.
            It defaults to 5.
            Raising it buys shorter motions that spend more effort. */
        void setRho(double rho)
        {
            steering_.setRho(rho);
        }

        /** \brief The time penalty charged per unit duration. */
        double getRho() const
        {
            return steering_.getRho();
        }

        /** \brief The steering that joins two flat states. */
        const MinimumEffortSteering &getSteering() const
        {
            return steering_;
        }

        /** \brief Pick the distance measurement returned from \ref distance.

            This also moves \ref isMetricSpace and \ref hasSymmetricDistance, so set it before a planner
            builds its nearest-neighbor structure.
        */
        void setDistanceType(DistanceType type)
        {
            distanceType_ = type;
        }

        /** \brief Get the distance measurement type. */
        DistanceType getDistanceType() const
        {
            return distanceType_;
        }

        /** \brief The cost of traversing the steering polynomial from \e from to \e to, meaning its
            control effort plus rho times its duration.

            Getting from a flat state to itself costs nothing, and a pair no polynomial reaches costs
            infinity.
        */
        double steeringCost(const State *from, const State *to) const;

        /** \brief The motion from \e from to \e to, or nothing when the two coincide and there's nothing
            to steer through.
        */
        virtual std::optional<FlatMotion> steer(const State *from, const State *to) const;

        /** \brief Write the flat state \e state carries into \e flatState, which comes out with one row per
            derivative level and one column per flat output dimension.

            Row 0 holds the values of the flat output in the order getValueAddressAtIndex lists them, so
            this throws unless the flat output space carries one value per dimension.
        */
        void toFlatState(const State *state, Eigen::Ref<Eigen::MatrixXd> flatState) const;

        /** \brief Read the flat state in \e flatState into \e state, which needs one row per derivative
            level and one column per flat output dimension.

            Row 0 holds the values of the flat output in the order getValueAddressAtIndex lists them, so
            this throws unless the flat output space carries one value per dimension.
        */
        void fromFlatState(const Eigen::Ref<const Eigen::MatrixXd> &flatState, State *state) const;

        /** \brief The state reached by following the steering polynomial from \e from toward \e to for
            \e t of its duration.

            Steering between coinciding flat states finds nothing to follow, and \e state then comes out
            equal to \e from, which planners already read as no progress.
            \e state is allowed to alias \e from or \e to.

            Every call solves the steering afresh, so a caller walking one edge sample by sample should
            hold the motion and use the overload below.
        */
        void interpolate(const State *from, const State *to, double t, State *state) const override;

        /** \brief The state reached by following the steering polynomial from \e from toward \e to for
            \e t of its duration, solving the steering only when \e firstTime says to.

            Hand in \e firstTime set, and the first call solves the steering and clears it.
            Pass the same flag and the same \e motion back on every later call for the same pair of
            states, and the whole edge costs one steering solve.
            \e motion comes out empty when the two flat states coincide.
            \e state is allowed to alias \e from or \e to.
        */
        void interpolate(const State *from, const State *to, double t, bool &firstTime,
                         std::optional<FlatMotion> &motion, State *state) const;

        /** \brief The state reached by following \e motion from \e from for \e t of its duration.

            \e motion has to run in the chart at the flat output of \e from, the way \ref steer builds it.
            \e state is allowed to alias \e from.
        */
        void interpolate(const State *from, const FlatMotion &motion, double t, State *state) const;

        /** \brief Calculate the number of segments of maximal valid length on the motion from \e state1 to
            \e state2.

            Every call solves the steering afresh, so a caller holding a motion should use the overload
            below.
        */
        unsigned int validSegmentCount(const State *state1, const State *state2) const override;

        /** \brief Calculate the number of segments of maximal valid length on \e motion. */
        unsigned int validSegmentCount(const FlatMotion &motion) const;

        /** \brief The distance from \e state1 to \e state2 under the current distance type. */
        double distance(const State *state1, const State *state2) const override;

        /** \brief Only \ref FLAT_STATE_METRIC is a metric. */
        bool isMetricSpace() const override
        {
            return distanceType_ == FLAT_STATE_METRIC && CompoundStateSpace::isMetricSpace();
        }

        /** \brief Only \ref FLAT_STATE_METRIC measures the same both ways. */
        bool hasSymmetricDistance() const override
        {
            return distanceType_ == FLAT_STATE_METRIC && CompoundStateSpace::hasSymmetricDistance();
        }

        /** \brief Motions run forward in time, so a motion and its reverse are different motions. */
        bool hasSymmetricInterpolate() const override
        {
            return false;
        }

        /** \brief Run the checks that apply under the current distance type. */
        void sanityChecks() const override;

        /** \brief Run the checks \e flags asks for, to the tolerances \e zero and \e eps. */
        using CompoundStateSpace::sanityChecks;

        /** \brief Warn about anything in \e planner's setup this space can't deliver on.

            Edges here run forward in time, so a planner that checks an edge one way and then walks it the
            other hands back a path that ompl::geometric::PathGeometric::check rejects.
            An optimizing planner with no ompl::base::FlatEffortObjective optimizes a sum of flat state
            distances, which isn't what traversing a path costs.
            Both come out as warnings, so planning runs either way.

            ompl::base::Planner::setup calls this.
        */
        void checkPlanner(const Planner *planner) const;

        void registerProjections() override;

        void setup() override;

        State *allocState() const override;

    protected:
        /** \brief Put \e rho in the parameter set, so a benchmark configuration can sweep it without
            touching code. */
        void declareParams();

        /** \brief Throw unless \e output carries at least one dimension and \e chart one coordinate per
            dimension of it. */
        static void checkOutput(const StateSpacePtr &output, const FlatChartPtr &chart);

        /** \brief Throw unless \e derivative derives from ompl::base::RealVectorStateSpace and carries
            \e dimension dimensions. */
        static void checkDerivative(const StateSpacePtr &derivative, unsigned int dimension);

        /** \brief The value of the flat output in \e state at \e index, throwing unless the flat output
            space carries exactly one value per dimension. */
        double *outputValue(State *state, unsigned int index) const;

        /** \brief The value of the flat output in \e state at \e index, throwing unless the flat output
            space carries exactly one value per dimension. */
        const double *outputValue(const State *state, unsigned int index) const;

        /** \brief Set every component weight to the reciprocal of its maximum extent, so the flat output
            and every derivative level contribute comparably whatever their units. */
        void setDefaultWeights();

        /** \brief The steering that joins two flat states. */
        MinimumEffortSteering steering_{2};

        /** \brief The chart that maintains coordinates for steering polynomials. */
        FlatChartPtr chart_;

        /** \brief What \ref distance measures. */
        DistanceType distanceType_{FLAT_STATE_METRIC};

        /** \brief The number of flat output dimensions. */
        unsigned int outputDimension_;

        /** \brief The weights \ref setDefaultWeights last wrote.
            \ref setup compares against these to tell weights a caller supplied apart from the defaults. */
        std::vector<double> defaultWeights_;
    };
}  // namespace ompl::base

#endif
