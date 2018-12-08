/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2013, Willow Garage
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
*   * Neither the name of Willow Garage nor the names of its
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

#ifndef OMPL_TEST_2DCIRCLES_SETUP_
#define OMPL_TEST_2DCIRCLES_SETUP_

#include <boost/filesystem.hpp>

#include "ompl/base/SpaceInformation.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "ompl/geometric/PathGeometric.h"

#include "../../resources/circles2D.h"

namespace ompl
{
    namespace geometric
    {

        class StateValidityChecker2DCircles : public base::StateValidityChecker
        {
        public:

            StateValidityChecker2DCircles(const base::SpaceInformationPtr &si, const Circles2D &circles) :
                base::StateValidityChecker(si),
                circles_(circles)
            {
            }

            bool isValid(const base::State *state) const override
            {
                const double *xy = state->as<base::RealVectorStateSpace::StateType>()->values;
                return circles_.noOverlap(xy[0], xy[1]);
            }

            double clearance(const base::State *state) const override
            {
                const double *xy = state->as<base::RealVectorStateSpace::StateType>()->values;
                return circles_.signedDistance(xy[0], xy[1]);
            }

        private:
            const Circles2D circles_;
        };

        static base::SpaceInformationPtr spaceInformation2DCircles(const Circles2D &circles)
        {
            auto space(std::make_shared<base::RealVectorStateSpace>());
            space->addDimension(circles.minX_, circles.maxX_);
            space->addDimension(circles.minY_, circles.maxY_);
            auto si(std::make_shared<base::SpaceInformation>(space));
            si->setStateValidityChecker(
                std::make_shared<StateValidityChecker2DCircles>(si, circles));
            si->setStateValidityCheckingResolution(0.002);
            si->setup();
            return si;
        }

        static std::vector<geometric::PathGeometric *> readPathsFromFile(const base::SpaceInformationPtr si, std::string filename)
        {
            std::vector<geometric::PathGeometric *> toReturn;
            geometric::PathGeometric *path = new geometric::PathGeometric(si);
            std::ifstream fin(filename.c_str());
            // ignore the first two lines.
            char dummy[4096];
            fin.getline(dummy, sizeof(dummy));
            fin.getline(dummy, sizeof(dummy));
            std::string line;
            while (getline(fin, line))
            {
                if (line.empty())
                {
                    // Add states to a path, add path, clear states.
                    toReturn.push_back(path);
                    path = new geometric::PathGeometric(si);
                }
                else
                {
                    std::istringstream tmp(line);
                    double x, y;
                    tmp >> x >> y;
                    base::State* state = si->allocState();
                    double* xy = state->as<base::RealVectorStateSpace::StateType>()->values;
                    xy[0] = x;
                    xy[1] = y;
                    path->append(state);
                }
            }
            if (path->getStateCount() != 0)
            {
                toReturn.push_back(path);
            }
            return toReturn;
        }

    }
}

#endif
