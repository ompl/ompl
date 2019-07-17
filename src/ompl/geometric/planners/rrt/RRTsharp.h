/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2016, Georgia Institute of Technology
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

/* Author: Florian Hauer */

#ifndef OMPL_GEOMETRIC_PLANNERS_RRT_RRTsharp_
#define OMPL_GEOMETRIC_PLANNERS_RRT_RRTsharp_

#include "ompl/geometric/planners/rrt/RRTXstatic.h"

namespace ompl
{
    namespace geometric
    {
        /**
           @anchor gRRTsharp
           @par Short description
           \ref gRRTsharp "RRT#" is an asymptotically-optimal incremental
           sampling-based motion planning algorithm. It is similar from  \ref gRRTXstatic "RRTXstatic"
           but maintains an optimal tree, same as \ref gRRTXstatic "RRTXstatic" with a treshold 0.\n
           The parameters are the same as \ref gRRTXstatic "RRTXstatic" except for the parameter epsilon.
           @par External documentation
           -# M. Otte & E. Frazzoli - RRTX : Real-Time Motion Planning/Replanning for Environments with Unpredictable
           Obstacles,
           Algorithmic Foundations of Robotics XI,
           Volume 107 of the series Springer Tracts in Advanced Robotics pp 461-478
           -# O. Arslan, P. Tsiotras - The role of vertex consistency in sampling-based algorithms for optimal motion
           planning,
           https://arxiv.org/pdf/1204.6453
           -# O. Arslan, P. Tsiotras - Dynamic programming guided exploration for sampling-based motion planning
           algorithms,
           2015 IEEE International Conference on Robotics and Automation (ICRA), pp 4819-4826
        */

        /** \brief Optimal Rapidly-exploring Random Trees Maintaining An Optimal Tree*/
        class RRTsharp : public RRTXstatic
        {
        public:
            RRTsharp(const base::SpaceInformationPtr &si);

            /** \brief Overwrite of RRTXstatic setEpsilon. It does nothing but warn the user that this parameter cannot
             * be
             * changed */
            void setEpsilon(double /*epsilon*/) override
            {
                OMPL_WARN("The parameter epsilon is 0 for the %s algorithm, it cannot be changed.", getName().c_str());
            }
        };
    }
}

#endif
