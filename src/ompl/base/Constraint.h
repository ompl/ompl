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

/* Author: Zachary Kingston, Ryan Luna */

#ifndef OMPL_BASE_CONSTRAINTS_CONSTRAINT_
#define OMPL_BASE_CONSTRAINTS_CONSTRAINT_

#include "ompl/base/StateSpace.h"
#include "ompl/base/OptimizationObjective.h"
#include "ompl/util/ClassForward.h"
#include "ompl/util/Exception.h"

#include <Eigen/Core>
#include <Eigen/Dense>
#include <utility>

namespace ompl
{
    namespace magic
    {
        /** \brief Default projection tolerance of a constraint unless otherwise
         * specified. */
        static const double CONSTRAINT_PROJECTION_TOLERANCE = 1e-4;

        /** \brief Maximum number of iterations in projection routine until
         * giving up. */
        static const unsigned int CONSTRAINT_PROJECTION_MAX_ITERATIONS = 50;
    }  // namespace magic

    namespace base
    {
        /// @cond IGNORE
        /** \brief Forward declaration of ompl::base::Constraint */
        OMPL_CLASS_FORWARD(Constraint);
        /// @endcond

        /** \class ompl::base::ConstraintPtr
            \brief A shared pointer wrapper for ompl::base::Constraint */

        /** \brief Definition of a differentiable holonomic constraint on a
         * configuration space. See \ref constrainedPlanning for more details.
         */
        class Constraint
        {
        public:
            /** \brief Constructor. The dimension of the ambient configuration
             space as well as the dimension of the function's output need to be
             specified (the co-dimension of the constraint manifold). I.E., for
             a sphere constraint function in \f$\mathbb{R}^3$,
            \f[
                F(q) = \left\lVert q \right\rVert - 1 \qquad F(q) : \f\mathbb{R}^3 \rightarrow \mathbb{R}
            \f]
            \a ambientDim will be 3, and \a coDim will be 1.
            */
            Constraint(const unsigned int ambientDim, const unsigned int coDim,
                       double tolerance = magic::CONSTRAINT_PROJECTION_TOLERANCE)
              : n_(ambientDim)
              , k_(ambientDim - coDim)
              , tolerance_(tolerance)
              , maxIterations_(magic::CONSTRAINT_PROJECTION_MAX_ITERATIONS)
            {
                if (n_ <= 0 || k_ <= 0)
                    throw ompl::Exception("ompl::base::Constraint(): "
                                          "Ambient and manifold dimensions must be positive.");
            }

            virtual ~Constraint() = default;

            /** @name Constraint Function Operations
                @{ */

            /** \brief Compute the constraint function at \a state. Result is
             returned in \a out, which should be allocated to size \a coDim. */
            virtual void function(const State *state, Eigen::Ref<Eigen::VectorXd> out) const;

            /** \brief Compute the constraint function at \a x. Result is
                returned in \a out, which should be allocated to size \a coDim. */
            virtual void function(const Eigen::Ref<const Eigen::VectorXd> &x,
                                  Eigen::Ref<Eigen::VectorXd> out) const = 0;

            /** \brief Compute the Jacobian of the constraint function at \a
             state. Result is returned in \a out, which should be allocated to
             size \a coDim by \a ambientDim. Default implementation performs the
             differentiation numerically with a seven-point central difference
             stencil. It is best to provide an analytic formulation. */
            virtual void jacobian(const State *state, Eigen::Ref<Eigen::MatrixXd> out) const;

            /** \brief Compute the Jacobian of the constraint function at \a x.
              Result is returned in \a out, which should be allocated to size \a
              coDim by \a ambientDim. Default implementation performs the
              differentiation numerically with a seven-point central difference
              stencil. It is best to provide an analytic formulation. */
            virtual void jacobian(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::MatrixXd> out) const;

            /** @} */

            /** @name Other Function Operations
                @{ */

            /** \brief Project a state \a state given the constraints. If a
             valid projection cannot be found, this method will return false.
             Even if this method fails, \a state will be modified. */
            virtual bool project(State *state) const;

            /** \brief Project a state \a x given the constraints. If a valid
                projection cannot be found, this method will return false. */
            virtual bool project(Eigen::Ref<Eigen::VectorXd> x) const;

            /** \brief Returns the distance of \a state to the constraint
             * manifold. */
            virtual double distance(const State *state) const;

            /** \brief Returns the distance of \a x to the constraint manifold. */
            virtual double distance(const Eigen::Ref<const Eigen::VectorXd> &x) const;

            /** \brief Check whether a state \a state satisfies the
             * constraints */
            virtual bool isSatisfied(const State *state) const;

            /** \brief Check whether a state \a x satisfies the constraints */
            virtual bool isSatisfied(const Eigen::Ref<const Eigen::VectorXd> &x) const;

            /** @} */

            /** @name Getters and Setters
                @{ */

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

            /** \brief Returns the dimension of the manifold. */
            unsigned int getCoDimension() const
            {
                return n_ - k_;
            }

            /** \brief Sets the underlying manifold dimension. */
            void setManifoldDimension(unsigned int k)
            {
                if (k <= 0)
                    throw ompl::Exception("ompl::base::Constraint(): "
                                          "Space is over constrained!");
                k_ = k;
            }

            /** \brief Returns the tolerance of the projection routine. */
            double getTolerance() const
            {
                return tolerance_;
            }

            /** \brief Returns the maximum number of allowed iterations in the
             * projection routine. */
            unsigned int getMaxIterations() const
            {
                return maxIterations_;
            }

            /** \brief Sets the projection tolerance. */
            void setTolerance(const double tolerance)
            {
                if (tolerance <= 0)
                    throw ompl::Exception("ompl::base::Constraint::setProjectionTolerance(): "
                                          "tolerance must be positive.");
                tolerance_ = tolerance;
            }

            /** \brief Sets the maximum number of iterations in the projection
             * routine. */
            void setMaxIterations(const unsigned int iterations)
            {
                if (iterations == 0)
                    throw ompl::Exception("ompl::base::Constraint::setProjectionMaxIterations(): "
                                          "iterations must be positive.");
                maxIterations_ = iterations;
            }

            /** @} */

        protected:
            /** \brief Ambient space dimension. */
            const unsigned int n_;

            /** \brief Manifold dimension. */
            unsigned int k_;

            /** \brief Tolerance for Newton method used in projection onto
             * manifold. */
            double tolerance_;

            /** \brief Maximum number of iterations for Newton method used in
             * projection onto manifold. */
            unsigned int maxIterations_;
        };

        /// @cond IGNORE
        OMPL_CLASS_FORWARD(ConstraintIntersection);
        /// @endcond

        /** \brief Definition of a constraint composed of multiple constraints
         that all must be satisfied simultaneously. This class `stacks' the
         constraint functions together. */
        class ConstraintIntersection : public Constraint
        {
        public:
            /** \brief Constructor. If constraints is empty assume it will be
             * filled later. */
            ConstraintIntersection(const unsigned int ambientDim, std::vector<ConstraintPtr> constraints)
              : Constraint(ambientDim, 0)
            {
                for (const auto &constraint : constraints)
                    addConstraint(constraint);
            }

            /** \brief Constructor. If constraints is empty assume it will be
             * filled later. */
            ConstraintIntersection(const unsigned int ambientDim, std::initializer_list<ConstraintPtr> constraints)
              : Constraint(ambientDim, 0)
            {
                for (const auto &constraint : constraints)
                    addConstraint(constraint);
            }

            void function(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::VectorXd> out) const override
            {
                unsigned int i = 0;
                for (const auto &constraint : constraints_)
                {
                    constraint->function(x, out.segment(i, constraint->getCoDimension()));
                    i += constraint->getCoDimension();
                }
            }

            void jacobian(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::MatrixXd> out) const override
            {
                unsigned int i = 0;
                for (const auto &constraint : constraints_)
                {
                    constraint->jacobian(x, out.block(i, 0, constraint->getCoDimension(), n_));
                    i += constraint->getCoDimension();
                }
            }

        protected:
            void addConstraint(const ConstraintPtr &constraint)
            {
                setManifoldDimension(k_ - constraint->getCoDimension());
                constraints_.push_back(constraint);
            }

            /** \brief Constituent constraints. */
            std::vector<ConstraintPtr> constraints_;
        };

        /// @cond IGNORE
        OMPL_CLASS_FORWARD(ConstraintObjective);
        /// @endcond

        /** \brief Wrapper around ompl::base::Constraint to use as an
         * optimization objective. */
        class ConstraintObjective : public OptimizationObjective
        {
        public:
            /** \brief Constructor. */
            ConstraintObjective(ConstraintPtr constraint, SpaceInformationPtr si)
              : OptimizationObjective(std::move(si)), constraint_(std::move(constraint))
            {
            }

            /** \brief Evaluate a cost map defined on the state space at a state
             * \e s. Cost map is defined as the distance from the constraint. */
            Cost stateCost(const State *s) const override
            {
                return Cost(constraint_->distance(s));
            }

        protected:
            /** \brief Optimizing constraint */
            ConstraintPtr constraint_;
        };
    }  // namespace base
}  // namespace ompl

#endif
