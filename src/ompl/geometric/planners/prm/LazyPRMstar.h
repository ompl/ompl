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

/* Author: Ioan Sucan, James D. Marble, Henning Kayser */

#ifndef OMPL_GEOMETRIC_PLANNERS_PRM_LAZYPRM_STAR_
#define OMPL_GEOMETRIC_PLANNERS_PRM_LAZYPRM_STAR_

#include "ompl/geometric/planners/prm/LazyPRM.h"

namespace ompl
{
    namespace geometric
    {
        /**
           @anchor gLazyPRMstar
           Run LazyPRM with the "star strategy". Instead of setting the
           value "k" for how many neighbors to connect, automatically
           compute it based on the coverage of the space, guaranteeing
           optimality of solutions.
           @par Short description
           @par External documentation
           R. Bohlin and L.E. Kavraki
           Path Planning Using Lazy PRM
           <em>IEEE International Conference on Robotics and Automation</em>, San Francisco, pp. 521–528, 2000.
           DOI: [10.1109/ROBOT.2000.844107](http://dx.doi.org/10.1109/ROBOT.2000.844107)<br>
           S. Karaman and E. Frazzoli, Sampling-based
           Algorithms for Optimal Motion Planning, International Journal of Robotics
           Research, vol. 30, no.7, pp. 846-894, 2011.
           DOI: [10.1177/0278364911406761](http://dx.doi.org/10.1177/0278364911406761)<br>
           [[more]](http://www.kavrakilab.org/robotics/lazyprm.html)
        */

        /** \brief PRM* planner */
        class LazyPRMstar : public LazyPRM
        {
        public:
            /** \brief Constructor */
            LazyPRMstar(const base::SpaceInformationPtr &si);

            /** \brief Constructor */
            LazyPRMstar(const base::PlannerData &data);
        };
    }
}

#endif
