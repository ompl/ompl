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

/* Author: Ioan Sucan */

#ifndef OMPL_BASE_PLANNER_DATA_
#define OMPL_BASE_PLANNER_DATA_

#include "ompl/base/SpaceInformation.h"
#include <iostream>
#include <vector>
#include <string>
#include <map>

namespace ompl
{
    namespace base
    {

        /** \brief Datatype holding data a planner can expose for debug purposes. */
        class PlannerData
        {
        public:
            PlannerData(void)
            {
            }

            virtual ~PlannerData(void)
            {
            }

            /** \brief Record an edge between two states. This
                function is called by planners to fill \e states, \e
                stateIndex and \e edges. If the same state/edge is
                seen multiple times, it is added only once.
                \return index of s1 in state array when an edge is added,
                -1 otherwise. */
            int recordEdge(const State *s1, const State *s2);

            /** \brief Assign a tag to a state */
            void tagState(const State *s, int tag);

            /** \brief Clear any stored data */
            void clear(void);

            /** \brief Print this data to a stream */
            virtual void print(std::ostream &out = std::cout) const;

            /** \brief The space information containing the states of the exploration datastructure */
            SpaceInformationPtr                      si;

            /** \brief The list of states in the current exploration datastructure */
            std::vector< const State* >              states;

            /** \brief For every state, a tag may be associated by the planner. For example, a bi-directional planner
                may assign one tag for states in the start tree and another for states in the goal tree. By default the tag has value 0. */
            std::vector< int >                       tags;

            /** \brief The same list of states as above, provided for convenience, in a manner that allows finding out a
                state's index from its pointer value */
            std::map< const State *, unsigned int >  stateIndex;

            /** \brief For each i, edges[i] contains the values edges[i][j] such that states[i] connects to every states[edges[i][j]] */
            std::vector< std::vector<unsigned int> > edges;

            /** \brief Any extra properties (key-value pairs) the planner can set. */
            std::map<std::string, std::string>       properties;
        };
    }
}

#endif
