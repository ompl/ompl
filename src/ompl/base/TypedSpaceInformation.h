/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2015, Rice University
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

/* Author: Neil T. Dantam */

#ifndef OMPL_BASE_TYPED_SPACE_INFORMATION_
#define OMPL_BASE_TYPED_SPACE_INFORMATION_

#include "ompl/base/SpaceInformation.h"
#include "ompl/base/ScopedState.h"

namespace ompl
{
    namespace base
    {
        template <typename SpaceType_>
        class TypedSpaceInformation : public SpaceInformation
        {
        public:
            /*--- Type Definitions ---*/

            /** The actual type of the state space */
            using SpaceType = SpaceType_;

            /** The actual type of states in the space. */
            using StateType = typename SpaceType::StateType;

            /** The actual type for a Scoped State. */
            using ScopedStateType = ScopedState<SpaceType>;

            /** Shared Pointer to the actual type of the space. */
            using SpacePtr = std::shared_ptr<SpaceType>;

            /** Shared pointer to the typed space. */
            using Ptr = std::shared_ptr<TypedSpaceInformation<SpaceType>>;

            /*--- Constructor ---*/

            /** Construct from shared pointer to the actual space. */
            TypedSpaceInformation(const SpacePtr &space) : SpaceInformation(space)
            {
            }

            /*--- Space Accessors ---*/

            /** Get space pointer of the proper type, const. */
            const SpaceType *getTypedStateSpace() const
            {
                return getStateSpace()->template as<SpaceType>();
            }

            /** Get space pointer of the proper type. */
            SpaceType *getTypedStateSpace()
            {
                return getStateSpace()->template as<SpaceType>();
            }

            /*--- State Memory Management ---*/

            /** Allocate a state of the proper type. */
            StateType *allocTypedState() const
            {
                return this->allocState()->template as<StateType>();
            }

            /** Allocate memory for typed states in array */
            void allocTypedStates(std::vector<StateType *> &states) const
            {
                allocStates(states);
            }

            /** Free a state of the proper type. */
            void freeTypedState(StateType *state) const
            {
                freeState(state);
            }

            /** Free typed states in array */
            void freeTypedStates(std::vector<StateType *> &states) const
            {
                freeStates(states);
            }

            /** Copy a state of the proper type. */
            void copyTypedState(StateType *destination, const StateType *source) const
            {
                copyState(destination, source);
            }

            /** Clone a state of the proper type. */
            StateType *cloneTypedState(const StateType *source) const
            {
                return this->cloneState(source)->template as<StateType>();
            }

            static StateType *state_as(ompl::base::State *s)
            {
                return s->template as<StateType>();
            }

            static const StateType *state_as(const ompl::base::State *s)
            {
                return s->template as<StateType>();
            }
        };
    }
}

#endif
