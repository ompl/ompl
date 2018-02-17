/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2018, Rice University
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

/* Authors: Bryce Willey */

#ifndef OMPL_TEST_2DARTICULATED_SETUP_
#define OMPL_TEST_2DARTICULATED_SETUP_

#include "ompl/base/SpaceInformation.h"
#include "ompl/base/spaces/SO2StateSpace.h"

#include "../../resources/config.h"
#include "../../resources/articulated2D.h"

namespace ompl
{
    namespace geometric
    {
        class StateValidityChecker2DArticulated : public base::StateValidityChecker
        {
        public:
            StateValidityChecker2DArticulated(const base::SpaceInformationPtr &si, const Articulated2D &articulated) :
                base::StateValidityChecker(si),
                articulated_(articulated)
            {}

            virtual bool isValid(const base::State *state) const
            {
                std::vector<double> angles(2);
                si_->getStateSpace()->copyToReals(angles, state);
                return !articulated_.isInCollision(angles[0], angles[1]);
            }

            virtual double clearance(const base::State *state) const
            {
                std::vector<double> angles(2);
                si_->getStateSpace()->copyToReals(angles, state);
                int link;
                Eigen::Vector2d point, global_point;
                return articulated_.signedDistance(angles[0], angles[1], link, point, global_point);
            }

            virtual double clearanceWithClosestGradient(const base::State *state, Eigen::MatrixXd &grad, bool &gradient_avaliable) const override
            {
                std::vector<double> angles(2);
                si_->getStateSpace()->copyToReals(angles, state);
                gradient_avaliable = true;
                return articulated_.signedDistanceGradient(angles[0], angles[1], grad);
            }
        
        private:
            Articulated2D articulated_;
        };

        static base::SpaceInformationPtr spaceInformation2DArticulated(const Articulated2D &articulated)
        {
            auto s1(std::make_shared<base::SO2StateSpace>());
            auto s2(std::make_shared<base::SO2StateSpace>());

            base::StateSpacePtr space = s1 + s2;
            auto si(std::make_shared<base::SpaceInformation>(space));
            si->setStateValidityChecker(std::make_shared<StateValidityChecker2DArticulated>(si, articulated));
            si->setStateValidityCheckingResolution(0.002);

            si->setup();
            return si;
        }
    }
}


#endif