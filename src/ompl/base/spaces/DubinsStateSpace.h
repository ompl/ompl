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

/* Author: Mark Moll */

#ifndef OMPL_BASE_SPACES_DUBINS_STATE_SPACE_
#define OMPL_BASE_SPACES_DUBINS_STATE_SPACE_

#include "ompl/base/spaces/SE2StateSpace.h"
#include <boost/math/constants/constants.hpp>

namespace ompl
{
    namespace base
    {

        /** \brief An SE(2) state space where distance is measured by the
            length of Dubins curves.

            The notation and solutions are taken from:
            A.M. Shkel and V. Lumelsky, “Classification of the Dubins set,”
            Robotics and Autonomous Systems, 34(4):179-202, 2001.
            DOI: <a href="http://dx.doi.org/10.1016/S0921-8890(00)00127-5">10.1016/S0921-8890(00)00127-5</a>
            */
        class DubinsStateSpace : public SE2StateSpace
        {
        public:

            /** \brief The Dubins path segment type */
            enum DubinsPathSegmentType { DUBINS_LEFT, DUBINS_RIGHT, DUBINS_STRAIGHT };
            /** \brief Dubins path types */
            static const DubinsPathSegmentType dubinsPathType[6][3];
            /** \brief Complete description of a Dubins path */
            class DubinsPath
            {
            public:
                DubinsPath(const DubinsPathSegmentType* type = dubinsPathType[0],
                    double t=0., double p=std::numeric_limits<double>::max(), double q=0.)
                {
                    memcpy(type_, type, 3*sizeof(DubinsPathSegmentType));
                    length_[0] = t;
                    length_[1] = p;
                    length_[2] = q;
                    assert(t >= 0.);
                    assert(p >= 0.);
                    assert(q >= 0.);
                }
                double length()
                {
                    return length_[0] + length_[1] + length_[2];
                }
                DubinsPath reverse()
                {
                    DubinsPath path(*this);
                    std::swap(path.type_[0],   path.type_[2]);
                    std::swap(path.length_[0], path.length_[2]);
                    return path;
                }

                /** Path segment types */
                DubinsPathSegmentType type_[3];
                /** Path segment lengths */
                double length_[3];
            };

            DubinsStateSpace(double turningRadius = 1.0) : SE2StateSpace(), rho_(turningRadius)
            {
            }

            virtual double distance(const State *state1, const State *state2) const;

            virtual void interpolate(const State *from, const State *to, const double t, State *state) const;

            /** \brief Return the shortest Dubins path from SE(2) state state1 to SE(2) state state2 */
            DubinsPath dubins(const State *state1, const State *state2) const;

        protected:
            static DubinsPath dubinsLSL(double d, double alpha, double beta);
            static DubinsPath dubinsRSR(double d, double alpha, double beta);
            static DubinsPath dubinsRSL(double d, double alpha, double beta);
            static DubinsPath dubinsLSR(double d, double alpha, double beta);
            static DubinsPath dubinsRLR(double d, double alpha, double beta);
            static DubinsPath dubinsLRL(double d, double alpha, double beta);

            /** \brief Turning radius */
            double rho_;
        };
    }
}

#endif
