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

/* Author: Bryce Willey */

#ifndef OMPL_TEST_2DSDF_SETUP_
#define OMPL_TEST_2DSDF_SETUP_

#include <boost/filesystem.hpp>

#include "ompl/base/SpaceInformation.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"

#include "../../resources/config.h"
#include "../../resources/sdf2D.h"

namespace ompl
{
    namespace geometric
    {

        class StateValidityChecker2DSdf : public base::StateValidityChecker
        {
        public:

            StateValidityChecker2DSdf(const base::SpaceInformationPtr &si, const SignedDistanceField2D &sdf) :
                base::StateValidityChecker(si),
                sdf_(sdf)
            {
            }

            virtual bool isValid(const base::State *state) const
            {
                const double *xy = state->as<base::RealVectorStateSpace::StateType>()->values;
                return sdf_.noOverlap(xy[0], xy[1]);
            }

            virtual double clearance(const base::State *state) const
            {
                const double *xy = state->as<base::RealVectorStateSpace::StateType>()->values;
                return sdf_.signedDistance(xy[0], xy[1]);
            }

            virtual double clearanceWithClosestGradient(const base::State * state, Eigen::MatrixXd &grad,
                bool &gradient_avaliable) const override
            {
                const double *xy = state->as<base::RealVectorStateSpace::StateType>()->values;
                gradient_avaliable = true;
                double dist = sdf_.obstacleDistanceGradient(xy[0], xy[1], grad);
                return dist;
            }

            virtual double clearanceWithMedialGradient(const base::State *state, Eigen::MatrixXd &grad, bool &gradient_avaliable) const override
            {
                const double *xy = state->as<base::RealVectorStateSpace::StateType>()->values;
                gradient_avaliable = true;
                return sdf_.medialAxisGradient(xy[0], xy[1], grad);
            }

        private:
            const SignedDistanceField2D sdf_;
        };

        static base::SpaceInformationPtr spaceInformation2DSdf(const SignedDistanceField2D &sdf)
        {
            auto space(std::make_shared<base::RealVectorStateSpace>());
            space->addDimension(sdf.minX_, sdf.maxX_);
            space->addDimension(sdf.minY_, sdf.maxY_);
            auto si(std::make_shared<base::SpaceInformation>(space));
            si->setStateValidityChecker(
                std::make_shared<StateValidityChecker2DSdf>(si, sdf));
            si->setStateValidityCheckingResolution(0.002);
            si->setup();
            return si;
        }
    }
}

#endif
