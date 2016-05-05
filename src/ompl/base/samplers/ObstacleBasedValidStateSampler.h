/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, Rice University
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

/* Author: Ioan Sucan */

#ifndef OMPL_BASE_SAMPLERS_OBSTACLE_BASED_VALID_STATE_SAMPLER_
#define OMPL_BASE_SAMPLERS_OBSTACLE_BASED_VALID_STATE_SAMPLER_

#include "ompl/base/ValidStateSampler.h"
#include "ompl/base/StateSampler.h"

namespace ompl
{
    namespace base
    {
        /** \brief Generate valid samples using obstacle based sampling.  First
            sample an invalid state, then sample a valid state.  Then, interpolate from
            the invalid state to the valid state, returning the first valid state
            encountered.

            @par External documentation
            N. M. Amato, O. B. Bayazit, L. K. Dale, C. Jones, and D. Vallejo, OBPRM: an obstacle-based PRM for 3D
           workspaces, in <em> Third Workshop on the Algorithmic Foundations of Robotics</em>, pp. 155-168, 1998.
           [[URL]](https://parasol.tamu.edu/groups/amatogroup/research/OBPRM/)
        */
        class ObstacleBasedValidStateSampler : public ValidStateSampler
        {
        public:
            /** \brief Constructor */
            ObstacleBasedValidStateSampler(const SpaceInformation *si);

            virtual ~ObstacleBasedValidStateSampler()
            {
            }

            virtual bool sample(State *state);
            virtual bool sampleNear(State *state, const State *near, const double distance);

        protected:
            /** \brief The sampler to build upon */
            StateSamplerPtr sampler_;
        };
    }
}

#endif
