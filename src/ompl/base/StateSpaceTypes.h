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

#ifndef OMPL_BASE_STATE_SPACE_TYPES_
#define OMPL_BASE_STATE_SPACE_TYPES_

namespace ompl
{
    namespace base
    {
        /** \brief The type of a state space */
        enum StateSpaceType
        {

            /** \brief Unset type; this is the default type */
            STATE_SPACE_UNKNOWN = 0,

            /** \brief ompl::base::RealVectorStateSpace */
            STATE_SPACE_REAL_VECTOR = 1,

            /** \brief ompl::base::SO2StateSpace */
            STATE_SPACE_SO2 = 2,

            /** \brief ompl::base::SO3StateSpace */
            STATE_SPACE_SO3 = 3,

            /** \brief ompl::base::SE2StateSpace */
            STATE_SPACE_SE2 = 4,

            /** \brief ompl::base::SE3StateSpace */
            STATE_SPACE_SE3 = 5,

            /** \brief ompl::base::TimeStateSpace */
            STATE_SPACE_TIME = 6,

            /** \brief ompl::base::DiscreteStateSpace */
            STATE_SPACE_DISCRETE = 7,

            /** \brief Number of state space types; To add new types,
                use values that are larger than the count*/
            STATE_SPACE_TYPE_COUNT
        };
    }
}

#endif
