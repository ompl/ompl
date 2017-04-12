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
#include "ompl/util/ClassForward.h"
#include "ompl/util/Exception.h"

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

namespace ompl
{
    namespace magic
    {
        /** \brief Default projection tolerance of a constraint unless otherwise specified. */
        static const double CONSTRAINT_PROJECTION_TOLERANCE = 1e-3;

        /** \brief Maximum number of iterations in projection routine until giving up. */
        static const unsigned int CONSTRAINT_PROJECTION_MAX_ITERATIONS = 50;
    }

    namespace base
    {
        /// @cond IGNORE
        OMPL_CLASS_FORWARD(Constraint);
        /// @endcond

        /** \brief Definition of a constraint on (a portion of) the state space. */
        class Constraint
        {
        public:
            /** \brief Constructor. */
            Constraint(const unsigned int ambientDim, const unsigned int manifoldDim)
              : n_(ambientDim)
              , k_(manifoldDim)
              , tolerance_(magic::CONSTRAINT_PROJECTION_TOLERANCE)
              , maxIterations_(magic::CONSTRAINT_PROJECTION_MAX_ITERATIONS)
            {
                if (n_ <= 0 || k_ <= 0)
                    throw ompl::Exception("ompl::base::Constraint(): "
                                          "Ambient and manifold dimensions must be positive.");
            }

            virtual ~Constraint() {};

            /** \brief Compute the constraint function at \a state. Result is
             * returned in \a out, which should be allocated to size n_. */
            void function(const State *state, const Eigen::Ref<Eigen::VectorXd> &out) const;

            /** \brief Compute the Jacobian of the constraint function at \a
             * state. Result is returned in \a out, which should be allocated to
             * size (n_ - k_) by n_. Default implementation performs the
             * differentiation numerically, which may be slower and/or more
             * inaccurate than an explicit formula. */
            void jacobian(const State *state, const Eigen::Ref<Eigen::MatrixXd> &out) const;

            /** \brief Project a state \a state given the constraints. If a valid
             * projection cannot be found, this method will return false. */
            bool project(State *state) const;

            /** \brief Returns the distance of \a state to the constraint manifold. */
            double distance(const State *state) const;

            /** \brief Check whether a state \a state satisfies the constraints */
            bool isSatisfied(const State *state) const;

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

            /** \brief Returns the maximum number of allowed iterations in the projection routine. */
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

            /** \brief Sets the maximum number of iterations in the projection routine. */
            void setMaxIterations(const unsigned int iterations)
            {
                if (iterations == 0)
                    throw ompl::Exception("ompl::base::Constraint::setProjectionMaxIterations(): "
                                          "iterations must be positive.");
                maxIterations_ = iterations;
            }

            // /** \brief Translates a state from the ambient space into an Eigen vector. */
            // Eigen::Ref<Eigen::VectorXd> toVector(const State *state) const;

            // /** \brief Translates an Eigen vector into a generic state from the ambient space. */
            // void fromVector(State *state, const Eigen::VectorXd &x) const;

            /** \brief Compute the constraint function at \a x. Result is returned
             * in \a out, which should be allocated to size n_. */
            virtual void function(const Eigen::VectorXd &x, Eigen::Ref<Eigen::VectorXd> out) const = 0;

            /** \brief Compute the Jacobian of the constraint function at \a x.
             * Result is returned in \a out, which should be allocated to size
             * (n_ - k_) by n_. Default implementation performs the
             * differentiation numerically, which may be slower and/or more
             * inaccurate than an explicit formula. */
            virtual void jacobian(const Eigen::VectorXd &x, Eigen::Ref<Eigen::MatrixXd> out) const;

            /** \brief Project a state \a x given the constraints. If a valid
             * projection cannot be found, this method will return false. */
            virtual bool project(Eigen::Ref<Eigen::VectorXd> x) const;

            /** \brief Returns the distance of \a x to the constraint manifold. */
            virtual double distance(const Eigen::VectorXd &x) const;

            /** \brief Check whether a state \a x satisfies the constraints */
            virtual bool isSatisfied(const Eigen::VectorXd &x) const;

        protected:
            /** \brief Ambient space dimension. */
            const unsigned int n_;

            /** \brief Manifold dimension. */
            unsigned int k_;

            /** \brief Tolerance for Newton method used in projection onto manifold. */
            double tolerance_;

            /** \brief Maximum number of iterations for Newton method used in projection onto manifold. */
            unsigned int maxIterations_;
        };

        /// @cond IGNORE
        OMPL_CLASS_FORWARD(ConstraintIntersection);
        /// @endcond

        /** \brief Definition of a constraint on (a portion of) the state space. */
        class ConstraintIntersection : public Constraint
        {
        public:
            /** \brief Constructor. If constraints is empty assume it will be filled later. */
            ConstraintIntersection(const unsigned int ambientDim, std::initializer_list<Constraint *> constraints)
              : Constraint(ambientDim, ambientDim)
            {
                for (auto constraint : constraints)
                    addConstraint(constraint);
            }

            ~ConstraintIntersection()
            {
                for (auto constraint : constraints_)
                    delete constraint;
            }

            void function(const Eigen::VectorXd &x, Eigen::Ref<Eigen::VectorXd> out) const
            {
                unsigned int i = 0;
                for (auto constraint : constraints_)
                {
                    constraint->function(x, out.segment(i, constraint->getCoDimension()));
                    i += constraint->getCoDimension();
                }
            }

            void jacobian(const Eigen::VectorXd &x, Eigen::Ref<Eigen::MatrixXd> out) const
            {
                unsigned int i = 0;
                for (auto constraint : constraints_)
                {
                    constraint->jacobian(x, out.block(i, 0, constraint->getCoDimension(), n_));
                    i += constraint->getCoDimension();
                }
            }

        protected:
            void addConstraint(Constraint *constraint)
            {
                setManifoldDimension(k_ - constraint->getCoDimension());
                constraints_.push_back(constraint);
            }

            std::vector<Constraint *> constraints_;
        };

        /** \brief Definition of a constraint on (a portion of) the state space. */
        class ConstraintUnion : public Constraint
        {
        public:
            /** \brief Constructor. If constraints is empty assume it will be filled later. */
            ConstraintUnion(const unsigned int ambientDim, std::initializer_list<Constraint *> constraints)
              : Constraint(ambientDim, ambientDim)
            {
                for (auto constraint : constraints)
                    addConstraint(constraint);
            }

            ~ConstraintUnion()
            {
                for (auto constraint : constraints_)
                    delete constraint;
            }

            const Constraint *closest(const Eigen::VectorXd &x) const
            {
                Eigen::VectorXd f(n_ - k_);

                Constraint *c = nullptr;
                double min = std::numeric_limits<double>::max();

                for (auto constraint : constraints_)
                {
                    double v = constraint->distance(x);
                    if (v < min)
                    {
                        c = constraint;
                        min = v;
                    }
                }

                return c;
            }

            void function(const Eigen::VectorXd &x, Eigen::Ref<Eigen::VectorXd> out) const
            {
                closest(x)->function(x, out);
            }

            void jacobian(const Eigen::VectorXd &x, Eigen::Ref<Eigen::MatrixXd> out) const
            {
                closest(x)->jacobian(x, out);
            }

        protected:
            void addConstraint(Constraint *constraint)
            {
                if (k_ == n_)
                    setManifoldDimension(constraint->getManifoldDimension());
                else if (k_ != constraint->getManifoldDimension())
                    throw ompl::Exception("ompl::base::ConstraintUnion(): "
                                          "Manifold Dimensions must be the same.");

                constraints_.push_back(constraint);
            }

            std::vector<Constraint *> constraints_;
        };
    }
}

#endif
