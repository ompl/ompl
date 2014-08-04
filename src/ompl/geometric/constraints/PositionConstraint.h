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

/* Author: Ryan Luna */

#ifndef OMPL_GEOMETRIC_CONSTRAINTS_POSITION_CONSTRAINT_
#define OMPL_GEOMETRIC_CONSTRAINTS_POSITION_CONSTRAINT_

#include "ompl/base/Constraint.h"

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/util/RandomNumbers.h>

#include <limits>

namespace ompl
{
    namespace geometric
    {
        /// @cond IGNORE
        OMPL_CLASS_FORWARD(PositionConstraint);
        /// @endcond

        /// Representation of a position constraint.  It is assumed that the
        /// portion of the state space constrained by this constraint is of
        /// type base::RealVectorStateSpace at the given SubstateLocation.  The
        /// dimensionality of the subspace must be equal to the given
        /// RealVectorBounds that represents the constraint.
        class PositionConstraint : public base::Constraint
        {
        public:
            PositionConstraint(const base::StateSpacePtr& space,
                               const base::StateSpace::SubstateLocation& loc,
                               const base::RealVectorBounds& bounds) : base::Constraint(space), loc_(loc), bounds_(bounds)
            {
                dim_ = bounds_.low.size();

                // TODO: Sanity check loc.space to be RealVectorStateSpace with
                // dimensionality == dim_
            }

            virtual ~PositionConstraint()
            {
            }

            /// \brief Check whether this state satisfies the constraints
            virtual bool isSatisfied(const base::State* state) const
            {
                const base::RealVectorStateSpace::StateType* substate = space_->getSubstateAtLocation(state, loc_)->as<base::RealVectorStateSpace::StateType>();

                for (unsigned int i = 0; i < dim_; ++i)
                {
                    if(substate->values[i] < bounds_.low[i] || substate->values[i] > bounds_.high[i])
                    {
                        /// Make sure that if the value is REALLY close to the bounds, we say "good enough"
                        double tol = 1e-6; // within tol of the bound is acceptable
                        if (fabs(bounds_.low[i] - substate->values[i]) > tol && fabs(bounds_.high[i] - substate->values[i]) > tol)
                        {
                            //std::cerr << "POSITION FAILED.  " << substate->values[i] << " should be between [" << bounds_.low[i] << " and " << bounds_.high[i] << "]" << std::endl;
                            return false;
                        }
                    }
                }

                return true;
            }

            virtual bool isSatisfied(const double* values) const
            {
                for (unsigned int i = 0; i < dim_; ++i)
                {
                    if(values[i] < bounds_.low[i] || values[i] > bounds_.high[i])
                    {
                        /// Make sure that if the value is REALLY close to the bounds, we say "good enough"
                        double tol = 1e-6; // within tol of the bound is acceptable
                        if (fabs(bounds_.low[i] - values[i]) > tol && fabs(bounds_.high[i] - values[i]) > tol)
                        {
                            //std::cerr << "POSITION FAILED.  " << substate->values[i] << " should be between [" << bounds_.low[i] << " and " << bounds_.high[i] << "]" << std::endl;
                            return false;
                        }
                    }
                }

                return true;
            }

            /// \brief Return the distance from satisfaction of a state
            /// A state that satisfies the constraint should have distance 0.
            /// Returns the Euclidean norm from constraint boundary
            virtual double distance(const base::State* state) const
            {
                double dist = 0.0;
                const base::RealVectorStateSpace::StateType* substate = space_->getSubstateAtLocation(state, loc_)->as<base::RealVectorStateSpace::StateType>();
                for (unsigned int i = 0; i < dim_; ++i)
                {
                    if(substate->values[i] < bounds_.low[i])
                    {
                        double d = substate->values[i] - bounds_.low[i];
                        dist += (d*d);
                    }

                    if(substate->values[i] > bounds_.high[i])
                    {
                        double d = substate->values[i] - bounds_.high[i];
                        dist += (d*d);
                    }
                }

                return sqrt(dist);
            }

            /// \brief Sample a state given the constraints.  If a state cannot
            /// be sampled, this method will return false.
            virtual bool sample(base::State* state)
            {
                base::RealVectorStateSpace::StateType* substate = space_->getSubstateAtLocation(state, loc_)->as<base::RealVectorStateSpace::StateType>();

                for (unsigned int i = 0 ; i < dim_; ++i)
                    substate->values[i] = rng_.uniformReal(bounds_.low[i], bounds_.high[i]);

                return true;
            }

            /// \brief Project a state given the constraints.  If a valid
            /// projection cannot be found, this method will return false.
            virtual bool project(base::State* state)
            {
                const base::RealVectorStateSpace::StateType* substate = space_->getSubstateAtLocation(state, loc_)->as<base::RealVectorStateSpace::StateType>();

                for (unsigned int i = 0; i < dim_; ++i)
                {
                    if(substate->values[i] < bounds_.low[i])
                        substate->values[i] = bounds_.low[i];
                    if(substate->values[i] > bounds_.high[i])
                        substate->values[i] = bounds_.high[i];
                }

                return true;
            }

            const base::RealVectorBounds& getConstraintBounds() const
            {
                return bounds_;
            }

            void setConstraintBounds(const base::RealVectorBounds& bounds)
            {
                bounds_ = bounds;
            }

        protected:
            base::StateSpace::SubstateLocation loc_;
            base::RealVectorBounds             bounds_;
            unsigned int                       dim_;
            RNG                                rng_;
        };
    }
}

#endif