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

#ifndef OMPL_GEOMETRIC_PLANNERS_RRT_INFORMED_RRTSTAR_
#define OMPL_GEOMETRIC_PLANNERS_RRT_INFORMED_RRTSTAR_


#include "ompl/geometric/planners/rrt/RRTstar.h"

namespace ompl
{

    namespace geometric
    {

        /**
            @anchor gInformedRRTstar

            Run \ref gRRTstar "RRT*" with an informed search strategy that uses heuristics to only consider subproblem that could provide a better solution.
            The search is limited to this subproblem by pruning the graph, generating samples only in this subproblem (directly if available, e.g., \ref gPathLengthDirectInfSampler "path length")
            and, when available, using the measure of this subproblem to calculate the connection terms (e.g., path length)

            @par Associated publication:

            J D. Gammell, S. S. Srinivasa, T. D. Barfoot, "Informed RRT*: Optimal Sampling-based
            Path Planning Focused via Direct Sampling of an Admissible Ellipsoidal Heuristic." In Proceedings
            of the IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS). Chicago, IL, USA,
            14-18 Sept. 2014.
            DOI: <a href="http://dx.doi.org/10.1109/IROS.2014.6942976">10.1109/IROS.2014.6942976</a>.
            <a href="http://www.youtube.com/watch?v=d7dX5MvDYTc">Illustration video</a>.
            <a href="http://www.youtube.com/watch?v=nsl-5MZfwu4">Short description video</a>.
        */

        /** \brief Informed RRT* */
        class InformedRRTstar : public RRTstar
        {
        public:

            /** \brief Constructor */
            InformedRRTstar(const base::SpaceInformationPtr &si);
        };
    }
}

#endif
