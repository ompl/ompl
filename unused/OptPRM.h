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

#ifndef OMPL_GEOMETRIC_PLANNERS_PRM_OPTPRM_
#define OMPL_GEOMETRIC_PLANNERS_PRM_OPTPRM_

#include "ompl/geometric/planners/prm/PRM.h"
#include <limits>

namespace ompl
{

    namespace geometric
    {

        /**
           @anchor gOptPRM

           @par Short description

           Construct a roadmap of milestones that approximate the
           connectivity of the state space.

           @par External documentation

           Kavraki, L. E.; Svestka, P.; Latombe, J.-C.; Overmars,
           M. H. (1996), "Probabilistic roadmaps for path planning in
           high-dimensional configuration spaces", IEEE Transactions
           on Robotics and Automation 12 (4): 566–580.
           @htmlonly
           <a href="http://en.wikipedia.org/wiki/Probabilistic_roadmap">http://en.wikipedia.org/wiki/Probabilistic_roadmap</a>
           <br>
           <a href="http://www.kavrakilab.org/robotics/prm.html">http://www.kavrakilab.org/robotics/prm.html</a>
           @endhtmlonly

        */

        /** \brief Rapidly-exploring Random Trees */
        class OptPRM : public PRM
        {
        public:

            OptPRM(const base::SpaceInformationPtr &si) : PRM(si)
            {
                name_ = "OptPRM";
                msg_.setPrefix(name_);
                ballRadiusMax_ = 0.0;
                ballRadiusConst_ = 1.0;
                maxNearestNeighbors_ = std::numeric_limits<unsigned int>::max();
            }

            virtual bool solve(double solveTime);

            virtual void setup(void);

            /** \brief When the planner computes nearest neighbors, it
                does so by looking at some of the neighbors within a
                computed radius. The computation of that radius
                depends on the multiplicative factor set here.*/
            void setBallRadiusConstant(double ballRadiusConstant)
            {
                ballRadiusConst_ = ballRadiusConstant;
            }

            /** \brief Get the multiplicative factor used in the
                computation of the radius whithin which tree rewiring
                is done. */
            double getBallRadiusConstant(void) const
            {
                return ballRadiusConst_;
            }

            /** \brief When the planner computes nearest neighbors, it
                does so by looking at some of the neighbors within a
                computed radius. That radius is bounded by the value
                set here.*/
            void setMaxBallRadius(double maxBallRadius)
            {
                ballRadiusMax_ = maxBallRadius;
            }

            /** \brief Get the maximum radius the planner uses in the
                tree rewiring step */
            double getMaxBallRadius(void) const
            {
                return ballRadiusMax_;
            }

        protected:

            virtual void nearestNeighbors(Milestone *milestone, std::vector<Milestone*> &nbh);

            double ballRadiusConst_;
            double ballRadiusMax_;
        };

    }
}

#endif
