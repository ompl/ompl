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
    
        class PlannerDataEdgeControl : public base::PlannerDataEdge
        {
        public:
            PlannerDataEdgeControl (const Control *c, double duration) : PlannerDataEdge(), c_(c), duration_(duration) {}
            PlannerDataEdgeControl (const PlannerDataEdgeControl &rhs) : PlannerDataEdge(), c_(rhs.c_), duration_(rhs.duration_) {}
            virtual ~PlannerDataEdgeControl (void) {}
            virtual base::PlannerDataEdge* clone () const
            { 
                return static_cast<base::PlannerDataEdge*>(new PlannerDataEdgeControl(*this)); 
            }
            
            const Control* getControl (void) const { return c_; }
            double getDuration (void) const { return duration_; }

        protected:
            const Control *c_;
            double duration_;
        };
    }
}

#endif
