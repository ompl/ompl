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

#ifndef OMPL_BASE_SPACES_FLAT_STATE_SPACE_
#define OMPL_BASE_SPACES_FLAT_STATE_SPACE_

#include "ompl/base/StateSpace.h"
#include "ompl/base/spaces/FlatMotion.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"

#include <vector>

namespace ompl::base
{
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
        The flat output space has to derive from ompl::base::RealVectorStateSpace.

        Distance is the flat output distance plus a weighted Euclidean distance per derivative level.
        It's symmetric and it satisfies the triangle inequality, so nearest-neighbor structures that prune
        on those properties stay usable.

        Edges run forward in time, so \ref hasSymmetricInterpolate reports false.
        Planners that traverse an edge backwards return paths this space rejects, and \ref spaces names
         the ones that only ever extend forward.
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

        /** \brief Build a flat state space over \e output holding \e order derivative levels, counting the
            flat output itself.

            The derivative components come out unbounded, so call \ref setDerivativeBound for each level
            before planning.
            The order has to be 2 and \e output has to derive from ompl::base::RealVectorStateSpace.
        */
        FlatStateSpace(const StateSpacePtr &output, unsigned int order);

        /** \brief Build a flat state space over \e output whose derivative levels are the spaces in
            \e derivatives, so the order is one more than their count.

            Using this constructor, client code can implement their own bounds, instead of relying on
            FlatStateSpace to build them.
            Each derivative space has to derive from ompl::base::RealVectorStateSpace and match the dimension of
            \e output, which keeps the value layout the steering math reads, and everything past that is
            the caller's to decide.
            Overriding enforceBounds, satisfiesBounds, and allocDefaultStateSampler on a derivative
            component reaches enforcement, checking, and sampling through compound delegation.
        */
        FlatStateSpace(const StateSpacePtr &output, const std::vector<StateSpacePtr> &derivatives);

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

        /** \brief The motion from \e from to \e to, or nothing when the two coincide and there's nothing
            to steer through. */
        virtual std::optional<FlatMotion> steer(const State *from, const State *to) const;

        /** \brief Write the flat state \e state carries into \e flatState, which comes out with one row per
            derivative level and one column per flat output dimension. */
        void toFlatState(const State *state, Eigen::Ref<Eigen::MatrixXd> flatState) const;

        /** \brief Read the flat state in \e flatState into \e state, which needs one row per derivative
            level and one column per flat output dimension. */
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

        /** \brief The state reached by following \e motion for \e t of its duration. */
        void interpolate(const FlatMotion &motion, double t, State *state) const;

        /** \brief Calculate the number of segments of maximal valid length on the motion from \e state1 to
            \e state2.

            Every call solves the steering afresh, so a caller holding a motion should use the overload
            below.
        */
        unsigned int validSegmentCount(const State *state1, const State *state2) const override;

        /** \brief Calculate the number of segments of maximal valid length on \e motion. */
        unsigned int validSegmentCount(const FlatMotion &motion) const;

        /** \brief Motions run forward in time, so a motion and its reverse are different motions. */
        bool hasSymmetricInterpolate() const override
        {
            return false;
        }

        void registerProjections() override;

        void setup() override;

        State *allocState() const override;

    protected:
        /** \brief Put \e rho in the parameter set, so a benchmark configuration can sweep it without
            touching code. */
        void declareParams();

        /** \brief Throw unless \e space derives from ompl::base::RealVectorStateSpace and carries
            \e dimension dimensions. */
        static void checkComponent(const StateSpacePtr &space, unsigned int dimension, const char *role);

        /** \brief Set every component weight to the reciprocal of its maximum extent, so the flat output
            and every derivative level contribute comparably whatever their units. */
        void setDefaultWeights();

        /** \brief The steering that joins two flat states. */
        MinimumEffortSteering steering_{2};

        /** \brief The number of flat output dimensions. */
        unsigned int outputDimension_;

        /** \brief The weights \ref setDefaultWeights last wrote.
            \ref setup compares against these to tell weights a caller supplied apart from the defaults. */
        std::vector<double> defaultWeights_;
    };
}  // namespace ompl::base

#endif
