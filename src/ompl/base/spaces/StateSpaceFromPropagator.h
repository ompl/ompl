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

/* Author: Javier V. Gomez */

#ifndef OMPL_BASE_SPACES_STATE_SPACE_FROM_PROPAGATOR_
#define OMPL_BASE_SPACES_STATE_SPACE_FROM_PROPAGATOR_

#include "ompl/base/StateSpace.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "ompl/base/spaces/SE2StateSpace.h"

namespace ompl
{
    namespace base
    {
/*T is a base State Space (namely SE2)
template <typename T>
class Bridge : public T {
public:
    Bridge(const StatePropagatorPtr &s) : T(), s_(s) {
                // check if the stateprop state space is the same as T (signature)}
    virtual void interpolate()...
}
*/
        /** \brief  */
        template <typename T>
        class StateSpaceFromPropagator : public T
        {
        public:

            /** \brief */
            /*class StateType : public T::StateType
            {
            public:
                //StateType() : T::StateType()
                StateType()
                {
                    std::cout << "State created." << std::endl;
                }
            };*/


            //StateSpaceFromPropagator() : T()
            StateSpaceFromPropagator<T>()
            {
                //setName("StateSpaceFromPropagator" + getName());
                //type_ = STATE_SPACE_FROM_PROPAGATOR;
                std::cout << "State space created" << std::endl;
            }

            virtual ~StateSpaceFromPropagator<T>()
            {
            }
        };
    }
}

#endif
