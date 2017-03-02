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

#ifndef OMPL_GEOMETRIC_CONSTRAINTS_CONSTRAINT_
#define OMPL_GEOMETRIC_CONSTRAINTS_CONSTRAINT_

#include "ompl/base/StateSpace.h"
#include "ompl/util/ClassForward.h"

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

namespace ompl
{
    namespace base
    {
        /// @cond IGNORE
        OMPL_CLASS_FORWARD(Constraint);
        /// @endcond

        /** \brief Definition of a constraint on (a portion of) the state space. */
        class Constraint
        {
        public:
            /** \brief Constructor.  Takes a pointer to the StateSpace being
             * constrained. */
            Constraint(const unsigned int ambientDimension, const unsigned int manifoldDimension)
              : n_(ambientDimension)
              , k_(manifoldDimension)
              , projectionTolerance_(1e-6)
              , projectionMaxIterations_(50)
            {
            }
            virtual ~Constraint()
            {
            }

            /** \brief Check whether this state satisfies the constraints */
            virtual bool isSatisfied(const Eigen::VectorXd &x) const;
            bool isSatisfied(const State *state) const;

            virtual double distance(const Eigen::VectorXd &x) const;
            double distance(const State *state) const;

            /** \brief Compute the constraint function at \a state. Result is returned
             * in \a out, which should be allocated to size n_. */
            virtual void function(const Eigen::VectorXd &x, Eigen::Ref<Eigen::VectorXd> out) const = 0;
            void function(const State *state, Eigen::Ref<Eigen::VectorXd> out) const;

            /** \brief Compute the Jacobian of the constraint function at \a
             * state. Result is returned in \a out, which should be allocated to
             * size (n_-k_) by n_. Default implementation performs the
             * differentiation numerically, which may be slower and/or more
             * inaccurate than an explicit formula. */
            virtual void jacobian(const Eigen::VectorXd &x, Eigen::Ref<Eigen::MatrixXd> out) const;
            void jacobian(const State *state, Eigen::Ref<Eigen::MatrixXd> out) const;

            /** \brief Project a state given the constraints.  If a valid
             * projection cannot be found, this method will return false. */
            virtual bool project(Eigen::Ref<Eigen::VectorXd> x) const;
            bool project(State *state) const;

            Eigen::Map<Eigen::VectorXd> toVector(const State *state) const;

            /** \brief Returns the dimension of the ambient space. */
            unsigned int getAmbientDimension() const;

            /** \brief Returns the dimension of the manifold. */
            unsigned int getManifoldDimension() const;

            /** \brief Returns the dimension of the ambient space. */
            double getProjectionTolerance() const;

            /** \brief Returns the dimension of the manifold. */
            unsigned int getProjectionMaxIterations() const;

        protected:
            /** \brief Ambient space dimension. */
            const unsigned int n_;

            /** \brief Manifold dimension. */
            const unsigned int k_;

            /** \brief Tolerance for Newton method used in projection onto manifold. */
            double projectionTolerance_;

            /** \brief Maximum number of iterations for Newton method used in projection onto manifold. */
            unsigned int projectionMaxIterations_;
        };
    }
}

#endif
