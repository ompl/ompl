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

#ifndef OMPL_GEOMETRIC_CONSTRAINTS_CONSTRAINT_
#define OMPL_GEOMETRIC_CONSTRAINTS_CONSTRAINT_

#include "ompl/base/StateSpace.h"
#include "ompl/util/ClassForward.h"

namespace ompl
{
    namespace base
    {
        /// @cond IGNORE
        OMPL_CLASS_FORWARD(StateSpace);
        /// @endcond

        /// @cond IGNORE
        OMPL_CLASS_FORWARD(Constraint);
        /// @endcond

        /// \brief Definition of a constraint on (a portion of) the state
        /// space.  Derived classes must implement the isSatisfied, sample,
        /// and project methods.  Distance is an optionally implemented value
        /// that some methods may take advantage of.
        class Constraint
        {
        public:

            /// \brief Constructor.  Takes a pointer to the StateSpace being
            /// constrained.
            Constraint(const StateSpacePtr& space) : space_(space) {}
            virtual ~Constraint() {}

            /// \brief Check whether this state satisfies the constraints
            virtual bool isSatisfied(const State* state) const = 0;

            /// \brief Return the distance from satisfaction of a state
            /// A state that satisfies the constraint should have distance 0.
            virtual double distance(const State* state) const = 0;

            /// \brief Sample a state given the constraints.  If a state cannot
            /// be sampled, this method will return false.
            virtual bool sample(State* state) = 0;

            /// \brief Project a state given the constraints.  If a valid
            /// projection cannot be found, this method will return false.
            virtual bool project(State* state) = 0;

        protected:
            StateSpacePtr space_;
        };
    }
}

#endif