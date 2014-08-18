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

/* Author: Caleb Voss */

#ifndef OMPL_BASE_SPACES_ATLAS_CONSTRAINT_
#define OMPL_BASE_SPACES_ATLAS_CONSTRAINT_

#include "ompl/base/Constraint.h"
#include "ompl/base/spaces/AtlasStateSpace.h"

namespace ompl
{
    namespace base
    {
        /** \brief Class which applies the same constraints as an atlas' bigF. */
        class AtlasConstraint : public Constraint
        {
        public:
            
            /** \brief Constructor. */
            AtlasConstraint (AtlasStateSpacePtr &atlas);
            
            /** \brief Does \a state lie within a tolerance of the constraints? */
            bool isSatisfied (const State *state) const;
            
            /** \brief How far is \a state from satisfying the constraints? */
            double distance (const State *state) const;
            
            /** \brief This just projects a uniform sample. Returns false if
             * the projection failed. */
            bool sample (State *state);
            
            /** \brief Project state to nearest point on the manifold. Return false
             * if projection fails. */
            bool project (State *state);
            
            /** \brief Convert RealVectorState to Eigen vector. */
            Eigen::VectorXd toVector(const State *state) const;
            
            /** \brief Convert Eigen vector to RealVectorState. */
            void fromVector(State *state, const Eigen::VectorXd &x) const;
            
        private:
            
            /** \brief Atlas the constraints belong to. */
            const AtlasStateSpace &atlas_;
            
            /** \brief Internal sampler. */
            mutable RealVectorStateSampler sampler_;
        };
    }
}

#endif
