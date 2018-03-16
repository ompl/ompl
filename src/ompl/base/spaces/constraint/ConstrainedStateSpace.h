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

/* Author: Zachary Kingston */

#ifndef OMPL_BASE_SPACES_CONSTRAINED_STATE_SPACE_
#define OMPL_BASE_SPACES_CONSTRAINED_STATE_SPACE_

#include "ompl/base/Constraint.h"
#include "ompl/base/StateSampler.h"
#include "ompl/base/ValidStateSampler.h"
#include "ompl/base/MotionValidator.h"
#include "ompl/base/spaces/WrapperStateSpace.h"
#include "ompl/base/SpaceInformation.h"

#include <eigen3/Eigen/Core>

namespace ompl
{
    namespace magic
    {
        /** \brief Default resolution for which to evaluate underlying
         * constraint manifold. */
        static const double CONSTRAINED_STATE_SPACE_DELTA = 0.05;
    }

    namespace base
    {
        /**
           @anchor gConstrained

           @par Short Description

           \ref gConstrained ConstrainedStateSpace encapsulates the idea of decoupled constrained planning, where the
           planner and constraint satisfaction methodology are separated. Core to this idea is the augmentation of a
           state space, rather than the augmentation of a planner. In OMPL, this is implemented as the
           ConstrainedStateSpace and its concrete implementations: ProjectedStateSpace for projection-based constraint
           satisfaction, AtlasStateSpace for atlas-based manifold approximation, and TangentBundleStateSpace for a lazy
           atlas-based approach.

           The core benefit of this method is that there is no additional work to make a sampling-based planner plan
           with constraints (in this case, a differentiable function of a state, implemented in Constraint), enabling
           mix-and-matching of planners with constraint methodologies, e.g., KPIECE1 with TangentBundleStateSpace, or
           RRT* with ProjectedStateSpace, and so on.

           @par External Documentation

           The decoupling approach is described in the following paper.

           Z. Kingston, M. Moll, L. E. Kavraki, "Decoupling Constraints from Sampling-Based Planners," in International
           Symposium of Robotics Research, Puerto Varas, Chile, 2017. PrePrint: <a
           href="http://kavrakilab.org/publications/kingston2017decoupling-constraints.pdf"></a>

           For more information on constrained sampling-based planning in general, see the following.

           Z. Kingston, M. Moll, and L. E. Kavraki, “Sampling-Based Methods for Motion Planning with Constraints,”
           Annual Review of Control, Robotics, and Autonomous Systems, 2018. PrePrint: <a
           href="http://kavrakilab.org/publications/kingston2018sampling-based-methods-for-motion-planning.pdf"></a>
        */

        /// @cond IGNORE
        /** \brief Forward declaration of ompl::base::ConstrainedStateSpace */
        OMPL_CLASS_FORWARD(ConstrainedStateSpace);
        /// @endcond

        /** \brief Constrained configuration space specific implementation of
         * checkMotion() that uses traverseManifold(). */
        class ConstrainedMotionValidator : public MotionValidator
        {
        public:
            /** \brief Constructor. */
            ConstrainedMotionValidator(SpaceInformation *si);

            /** \brief Constructor. */
            ConstrainedMotionValidator(const SpaceInformationPtr &si);

            /** \brief Return whether we can step from \a s1 to \a s2 along the
             * manifold without collision. */
            bool checkMotion(const State *s1, const State *s2) const override;

            /** \brief Return whether we can step from \a s1 to \a s2 along the
             * manifold without collision. If not, return the last valid state
             * and its interpolation parameter in \a lastValid. \note The
             * interpolation parameter will not likely reproduce the last valid
             * state if used in interpolation since the distance between the
             * last valid state and \a s2 is estimated using the ambient
             * metric. */
            bool checkMotion(const State *s1, const State *s2, std::pair<State *, double> &lastValid) const override;

        protected:
            /** \brief Space in which we check motion. */
            const ConstrainedStateSpace &ss_;
        };

        /** \brief A state space that has a \a Constraint imposed upon it.
         * Underlying space functions are passed to the ambient space, and the
         * constraint is used to inform any manifold related operations.
         * setSpaceInformation() must be called in order for collision checking
         * to be done in tandem with manifold traversal, a significant time
         * saver. */
        class ConstrainedStateSpace : public WrapperStateSpace
        {
        public:
            /** \brief A state in a constrained configuration space that can be
             * represented as a dense real vector of values. */
            class StateType : public WrapperStateSpace::StateType, public Eigen::Map<Eigen::VectorXd>
            {
            public:
                /** \brief Constructor. Requires \a space to setup information about underlying state. */
                StateType(const ConstrainedStateSpace *space)
                  : WrapperStateSpace::StateType(space->getSpace()->allocState())
                  , Eigen::Map<Eigen::VectorXd>(space->getValueAddressAtIndex(this, 0), space->getDimension())
                {
                }

                void copy(const Eigen::Ref<const Eigen::VectorXd> &other)
                {
                    // Explicitly cast and call `=` on the state as an
                    // Eigen::Map<...>. This copies the other.
                    static_cast<Eigen::Map<Eigen::VectorXd> *>(this)->operator=(other);
                }
            };

            /** \brief Construct a constrained space from an \a ambientSpace and
             * a \a constraint. */
            ConstrainedStateSpace(const StateSpacePtr &ambientSpace, const ConstraintPtr &constraint);

            /** \brief Returns false as the implicit constrained configuration
             * space defined by the constraint is not metric with respect to the
             * ambient configuration space's metric. */
            bool isMetricSpace() const override
            {
                return false;
            }

            /** \brief Check that the space referred to by the space information
             * \a si is, in fact, an ConstrainedStateSpace. */
            static void checkSpace(const SpaceInformation *si);

            /** \brief Sets the space information for this state space. Required
             * for collision checking in manifold traversal. */
            void setSpaceInformation(SpaceInformation *si);

            /** \brief Final setup for the space. */
            void setup() override;

            /** \brief Clear any allocated memory from the state space. */
            virtual void clear();

            /** \brief Allocate a new state in this space. */
            State *allocState() const override;

            /** @name Constrained Planning
                @{ */

            /** \brief Find a state between \a from and \a to around time \a t,
             * where \a t = 0 is \a from, and \a t = 1 is the final state
             * reached by traverseManifold(\a from, \a to, true, ...), which may
             * not be \a to. State returned in \a state. */
            void interpolate(const State *from, const State *to, double t, State *state) const override;

            /** \brief Traverse the manifold from \a from toward \a to. Returns
             * true if we reached \a to, and false if we stopped early for any
             * reason, such as a collision or traveling too far. No collision
             * checking is performed if \a interpolate is true. If \a stateList
             * is not nullptr, the sequence of intermediates is saved to it,
             * including a copy of \a from, as well as the final state, which is
             * a copy of \a to if we reached \a to. Caller is responsible for
             * freeing states returned in \a stateList. if \a endpoints is true,
             * then \a from and \a to are included in stateList. Needs to be
             * implemented by any constrained state space. */
            virtual bool traverseManifold(const State *from, const State *to, bool interpolate = false,
                                          std::vector<State *> *stateList = nullptr, bool endpoints = true) const = 0;

            /** \brief Like interpolate(...), but interpolates between
             * intermediate states already supplied in \a stateList from a
             * previous call to traverseManifold(..., true, \a stateList). The
             * \a from and \a to states are the first and last elements \a
             * stateList. Returns a pointer to a state in \a stateList. */
            virtual State *piecewiseInterpolate(const std::vector<State *> &stateList, double t) const;

            /** @} */

            /** @name Setters and Getters
                @{ */

            /** \brief Set \a delta, the step size for traversing the manifold
             * and collision checking. Default defined by
             * ompl::magic::CONSTRAINED_STATE_SPACE_DELTA. */
            void setDelta(const double delta);

            /** \brief Get delta, the step size across the manifold. */
            double getDelta() const
            {
                return delta_;
            }

            /** \brief Returns the dimension of the ambient space. */
            unsigned int getAmbientDimension() const
            {
                return n_;
            }

            /** \brief Returns the dimension of the manifold. */
            unsigned int getManifoldDimension() const
            {
                return k_;
            }

            /** \brief Returns the constraint that defines the underlying manifold. */
            const ConstraintPtr getConstraint() const
            {
                return constraint_;
            }

            /** @} */

        protected:
            /** \brief SpaceInformation associated with this space. Required
             * for early collision checking in manifold traversal. */
            SpaceInformation *si_{nullptr};

            /** \brief Constraint function that defines the manifold. */
            const ConstraintPtr constraint_;

            /** \brief Ambient space dimension. */
            const unsigned int n_;

            /** \brief Manifold dimension. */
            const unsigned int k_;

            /** \brief Step size when traversing the manifold and collision checking. */
            double delta_;

            /** \brief Whether setup() has been called. */
            bool setup_{false};
        };
    }
}

#endif
