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

#include <Eigen/Core>

namespace ompl
{
    namespace magic
    {
        static const double CONSTRAINED_STATE_SPACE_DELTA = 0.05;
        static const double CONSTRAINED_STATE_SPACE_LAMBDA = 2.0;
    }

    namespace base
    {
        /// @cond IGNORE
        /** \brief Forward declaration of ompl::base::ConstrainedStateSpace */
        OMPL_CLASS_FORWARD(ConstrainedStateSpace);
        /// @endcond

        /** \brief Constrained configuration space specific implementation of
         * checkMotion() that uses discreteGeodesic(). */
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

        /**
           @anchor gConstrained
           @par Short Description
           ConstrainedStateSpace encapsulates the idea of decoupled constrained planning, where the planner and
           constraint satisfaction methodology are separated. Core to this idea is the augmentation of a state space,
           rather than the augmentation of a planner. In OMPL, this is implemented as the ConstrainedStateSpace and its
           concrete implementations: ProjectedStateSpace for projection-based constraint satisfaction, AtlasStateSpace
           for atlas-based manifold approximation, and TangentBundleStateSpace for a lazy atlas-based approach.

           The core benefit of this method is that there is no additional work to make a sampling-based planner plan
           with constraints (in this case, a differentiable function of a state, implemented in Constraint), enabling
           mix-and-matching of planners with constraint methodologies, e.g., KPIECE1 with TangentBundleStateSpace, or
           RRT* with ProjectedStateSpace, and so on.

           See \ref constrainedPlanning for more details.

           @par External Documentation
           The following paper describes the idea of decoupled constrained planning, as implemented in OMPL.

           Zachary Kingston, Mark Moll, and Lydia E. Kavraki, Exploring Implicit Spaces for Constrained Sampling-Based
           Planning, _International Journal of Robotics Research,_ 38(10–11):1151–1178, September 2019.
           DOI: <a href="https://dx.doi.org/10.1177/0278364919868530">10.1177/0278364919868530</a>
           PrePrint:
           <a href="http://www.kavrakilab.org/publications/kingston2019exploring-implicit-spaces-for-constrained.pdf">[PDF]</a>

           For more information on constrained sampling-based planning in general, see the following review paper. The
           sections on projection- and atlas-based planning describe the methods used in the ProjectedStateSpace,
           AtlasStateSpace, and TangentBundleStateSpace.

           Z. Kingston, M. Moll, and L. E. Kavraki, “Sampling-Based Methods for Motion Planning with Constraints,”
           Annual Review of Control, Robotics, and Autonomous Systems, 2018. DOI:
           <a href="https://dx.doi.org/10.1146/annurev-control-060117-105226">10.1146/annurev-control-060117-105226</a>
           <a href="http://kavrakilab.org/publications/kingston2018sampling-based-methods-for-motion-planning.pdf">[PDF]</a>.
        */

        /** \brief A StateSpace that has a \a Constraint imposed upon it.
         * Underlying space functions are passed to the ambient space, and the
         * constraint is used to inform any manifold related operations.
         * setSpaceInformation() must be called in order for collision checking
         * to be done in tandem with manifold traversal. */
        class ConstrainedStateSpace : public WrapperStateSpace
        {
        public:
            /** \brief Flags used in a bit mask for constrained state space
             * sanity checks, constraintSanityChecks(). */
            enum SanityChecks
            {
                /** \brief Check whether state samplers return constraint
                 * satisfying samples. */
                CONSTRAINED_STATESPACE_SAMPLERS = (1 << 1),

                /** \brief Check whether discrete geodesics satisfy the
                 * constraint at all points. */
                CONSTRAINED_STATESPACE_GEODESIC_SATISFY = (1 << 2),

                /** \brief Check whether discrete geodesics keep lambda_ *
                 * delta_ continuity. */
                CONSTRAINED_STATESPACE_GEODESIC_CONTINUITY = (1 << 3),

                /** \brief Check whether geodesicInterpolate(...) returns
                 * constraint satisfying states. */
                CONSTRAINED_STATESPACE_GEODESIC_INTERPOLATE = (1 << 4),

                /** \brief Check if the constraint's numerical Jacobian
                 * approximates its provided Jacobian. */
                CONSTRAINED_STATESPACE_JACOBIAN = (1 << 5)
            };

            /** \brief A State in a ConstrainedStateSpace, represented as a
             * dense real vector of values. For convenience and efficiency of
             * various Constraint related operations, this State inherits from
             * Eigen::Map<Eigen::VectorXd>, mapping the underlying dense double
             * vector into an Eigen::VectorXd. Note that this state type
             * inherits from WrapperStateSpace::StateType, and as such the
             * underlying state can be accessed by getState(). */
            class StateType : public WrapperStateSpace::StateType, public Eigen::Map<Eigen::VectorXd>
            {
            public:
                /** \brief Constructor. Requires \a space to setup information about underlying state. */
                StateType(const ConstrainedStateSpace *space)
                  : WrapperStateSpace::StateType(space->getSpace()->allocState())
                  , Eigen::Map<Eigen::VectorXd>(space->getValueAddressAtIndex(this, 0), space->getDimension())
                {
                }

                /** \brief Copy the contents from a vector into this state. Uses
                 * the underlying copy operator used by Eigen for dense
                 * vectors. */
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

            /** @name State Space Related Operations
                @{ */

            /** \brief Returns false as the implicit constrained configuration
             * space defined by the constraint is not metric with respect to the
             * ambient configuration space's metric. */
            bool isMetricSpace() const override
            {
                return false;
            }

            /** \brief Sets the space information for this state space. Required
             * for collision checking in manifold traversal. */
            void setSpaceInformation(SpaceInformation *si);

            /** \brief Final setup for the space. */
            void setup() override;

            /** \brief Clear any allocated memory from the state space. */
            virtual void clear();

            /** \brief Allocate a new state in this space. */
            State *allocState() const override;

            /** \brief Do some sanity checks relating to discrete geodesic
            computation and constraint satisfaction. See SanityChecks flags. */
            void constrainedSanityChecks(unsigned int flags) const;

            /** \brief Perform both constrained and regular sanity checks. */
            void sanityChecks() const override;

            /** \brief Return the valid segment count on the manifold, as valid
             * segment count is determined by \e delta_ and \e lambda_. */
            unsigned int validSegmentCount(const State* s1, const State* s2) const override
            {
                return distance(s1, s2) * (1. / delta_) * lambda_;
            }

            /** @} */

            /** @name Constrained Planning
                @{ */

            /** \brief Find a state between \a from and \a to around time \a t,
             * where \a t = 0 is \a from, and \a t = 1 is the final state
             * reached by discreteGeodesic(\a from, \a to, true, ...), which may
             * not be \a to. State returned in \a state. */
            void interpolate(const State *from, const State *to, double t, State *state) const override;

            /** \brief Traverse the manifold from \a from toward \a to. Returns
             * true if we reached \a to, and false if we stopped early for any
             * reason, such as a collision or traveling too far. No collision
             * checking is performed if \a interpolate is true. If \a geodesic
             * is not nullptr, the sequence of intermediates is saved to it,
             * including a copy of \a from, as well as the final state, which is
             * a copy of \a to if we reached \a to. Caller is responsible for
             * freeing states returned in \a geodesic. Needs to be implemented
             * by any constrained state space. */
            virtual bool discreteGeodesic(const State *from, const State *to, bool interpolate = false,
                                          std::vector<State *> *geodesic = nullptr) const = 0;

            /** \brief Like interpolate(...), but interpolates between
             * intermediate states already supplied in \a stateList from a
             * previous call to discreteGeodesic(..., \a geodesic). The
             * \a from and \a to states are the first and last elements \a
             * stateList. Returns a pointer to a state in \a geodesic. */
            virtual State *geodesicInterpolate(const std::vector<State *> &geodesic, double t) const;

            /** @} */

            /** @name Setters and Getters
                @{ */

            /** \brief Set \a delta, the step size for traversing the manifold
             * and collision checking. Default defined by
             * ompl::magic::CONSTRAINED_STATE_SPACE_DELTA. */
            void setDelta(double delta);

            /** \brief Set \a lambda, where lambda * distance(x, y) is the
             * maximum length of the geodesic x to y. Additionally, lambda *
             * delta is the greatest distance a point can diverge from its
             * previous step, to preserve continuity. Must be greater than 1. */
            void setLambda(double lambda)
            {
                if (lambda <= 1)
                    throw ompl::Exception("ompl::base::AtlasStateSpace::setLambda(): "
                                          "lambda must be > 1.");
                lambda_ = lambda;
            }

            /** \brief Get delta, the step size across the manifold. */
            double getDelta() const
            {
                return delta_;
            }

            /** \brief Get lambda (see setLambda()). */
            double getLambda() const
            {
                return lambda_;
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

            /** \brief Manifold traversal from x to y is stopped if accumulated
             * distance is greater than d(x,y) times this. Additionally, if d(x,
             * y) is greater than lambda * delta between two points, search is
             * discontinued. */
            double lambda_{ompl::magic::CONSTRAINED_STATE_SPACE_LAMBDA};

            /** \brief Whether setup() has been called. */
            bool setup_{false};
        };
    }
}

#endif
