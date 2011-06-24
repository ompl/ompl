/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2011, Rice University
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

/* Author: Mark Moll */

#ifndef OMPL_CONTROL_PLANNER_DATA_
#define OMPL_CONTROL_PLANNER_DATA_

#include "ompl/base/PlannerData.h"
#include "ompl/control/Control.h"

namespace ompl
{
    namespace control
    {

        /** \brief Datatype holding data a planner can expose for debug purposes. */
        class PlannerData : public base::PlannerData
        {
        public:
            PlannerData(void) : base::PlannerData()
            {
            }

            virtual ~PlannerData(void)
            {
            }

            /** \brief Record an edge between two states. This
                function is called by planners to fill \e states, \e
                stateIndex and \e edges. If the same state/edge is
                seen multiple times, it is added only once. */
            int recordEdge(const base::State *s1, const base::State *s2, const Control* c, double duration);

            /** \brief Clear any stored data */
            virtual void clear(void);

            /** \brief For each i, controls[i] contains the controls[i][j]
                that are needed to take the system from state[i] to state[j] */
            std::vector< std::vector< const Control* > > controls;

            /** \brief controlDurations[i][j] contains the duration that
                controls[i][j] needs to be applied to take the system
                from state[i] to state[j] */
            std::vector< std::vector< double > >         controlDurations;
        };
    }
}

#endif
