/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, University of Toronto
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
*   * Neither the name of the University of Toronto nor the names of its
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

/* Authors: Jonathan Gammell */

#ifndef OMPL_GEOMETRIC_PLANNERS_RRT_SORRTSTAR_
#define OMPL_GEOMETRIC_PLANNERS_RRT_SORRTSTAR_

#include "ompl/geometric/planners/rrt/InformedRRTstar.h"

namespace ompl
{
    namespace geometric
    {
        /**
            @anchor gSORRTstar

            Run \ref gRRTstar "RRT*" as SORRT* using an ordered informed search strategy that considers states in the
           subproblem that could provide a better solution in order of their potential solution cost.
            A sorted version \ref gInformedRRTstar "Informed RRT*".

            @par Associated publication:

            J. D. Gammell, T. D. Barfoot, S. S. Srinivasa,  "Batch Informed Trees (BIT*): Informed asymptotically optimal
            anytime search." The International Journal of Robotics Research (IJRR), 39(5): 543-567, Apr. 2020.
            DOI: <a href="https://doi.org/10.1177/0278364919890396">10.1177/0278364919890396</a>.
            arXiv: <a href="https://arxiv.org/pdf/1707.01888">1707.01888 [cs.RO]</a>.
            <a href="https://www.youtube.com/watch?v=MRzSfLpNBmA">Illustration video</a>.
        */

        /** \brief SORRT* */
        class SORRTstar : public InformedRRTstar
        {
        public:
            /** \brief Constructor */
            SORRTstar(const base::SpaceInformationPtr &si);
        };
    }
}

#endif
