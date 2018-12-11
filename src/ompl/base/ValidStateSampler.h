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

#ifndef OMPL_BASE_VALID_STATE_SAMPLER_
#define OMPL_BASE_VALID_STATE_SAMPLER_

#include "ompl/base/State.h"
#include "ompl/util/ClassForward.h"
#include "ompl/base/GenericParam.h"
#include <string>

namespace ompl
{
    namespace base
    {
        /// @cond IGNORE
        OMPL_CLASS_FORWARD(SpaceInformation);
        /// @endcond

        /// @cond IGNORE
        /** \brief Forward declaration of ompl::base::ValidStateSampler */
        OMPL_CLASS_FORWARD(ValidStateSampler);
        /// @endcond

        /** \class ompl::base::ValidStateSamplerPtr
            \brief A shared pointer wrapper for ompl::base::ValidStateSampler */

        /** \brief Abstract definition of a state sampler. */
        class ValidStateSampler
        {
        public:
            // non-copyable
            ValidStateSampler(const ValidStateSampler &) = delete;
            ValidStateSampler &operator=(const ValidStateSampler &) = delete;

            /** \brief Constructor */
            ValidStateSampler(const SpaceInformation *si);

            virtual ~ValidStateSampler();

            /** \brief Get the name of the sampler */
            const std::string &getName() const
            {
                return name_;
            }

            /** \brief Set the name of the sampler */
            void setName(const std::string &name)
            {
                name_ = name;
            }

            /** \brief Sample a state. Return false in case of failure */
            virtual bool sample(State *state) = 0;

            /** \brief Sample a state near another, within specified distance. Return false, in case of failure.
                \note The memory for \e near must be disjoint from the memory for \e state */
            virtual bool sampleNear(State *state, const State *near, double distance) = 0;

            /** \brief Finding a valid sample usually requires
                performing multiple attempts. This call allows setting
                the number of such attempts. */
            void setNrAttempts(unsigned int attempts)
            {
                attempts_ = attempts;
            }

            /** \brief Get the number of attempts to be performed by the sampling routine */
            unsigned int getNrAttempts() const
            {
                return attempts_;
            }

            /** \brief Get the parameters for the valid state sampler */
            ParamSet &params()
            {
                return params_;
            }

            /** \brief Get the parameters for the valid state sampler */
            const ParamSet &params() const
            {
                return params_;
            }

        protected:
            /** \brief The state space this sampler samples */
            const SpaceInformation *si_;

            /** \brief Number of attempts to find a valid sample */
            unsigned int attempts_;

            /** \brief The name of the sampler */
            std::string name_;

            /** \brief The parameters for this instance of the valid state sampler */
            ParamSet params_;
        };

        /** \brief Definition of a function that can allocate a valid state sampler */
        using ValidStateSamplerAllocator = std::function<ValidStateSamplerPtr(const SpaceInformation *)>;
    }
}

#endif
