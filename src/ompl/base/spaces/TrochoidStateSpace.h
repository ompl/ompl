/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2025, Autonomous Systems Laboratory, ETH Zurich
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
*   * Neither the name of the ETH Zurich nor the names of its
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

/* Author: Jaeyoung Lim */

#ifndef OMPL_BASE_SPACES_TROCHOID_STATE_SPACE_
#define OMPL_BASE_SPACES_TROCHOID_STATE_SPACE_

#include "ompl/base/spaces/SE2StateSpace.h"
#include "ompl/base/MotionValidator.h"
#include <boost/math/constants/constants.hpp>
#include <optional>

namespace ompl
{
    namespace base
    {
        /** \brief An SE(2) state space where distance is measured by the
            length of Trochoid shortest paths curves.

            The notation and solutions in the code are taken from:<br>
            Techy, Laszlo, and Craig A. Woolsey. "Minimum-time path planning for unmanned aerial vehicles
             in steady uniform winds." Journal of guidance, control, and dynamics 32.6 (2009): 1736-1746.

            The implementation is from adapted from https://github.com/robotics-uncc/ConvectedDubins
            written by Artur Wolek.
            */
        class TrochoidStateSpace : public SE2StateSpace
        {
        public:
            /** \brief The Dubins path segment type */
            enum TrochoidPathSegmentType
            {
                TROCHOID_LEFT = 0,
                TROCHOID_STRAIGHT = 1,
                TROCHOID_RIGHT = 2
            };

            /** \brief Dubins path types */
            static const std::vector<std::vector<TrochoidPathSegmentType>>& dubinsPathType();
            /** \brief Complete description of a Dubins path */
            class PathType
            {
            public:
                PathType(const std::vector<TrochoidPathSegmentType>& type = dubinsPathType()[0], double tA = 0.,
                           double p = std::numeric_limits<double>::max(), double tB = 0., double t_2pi = 0.)
                  : type_(&type)
                {
                    length_[0] = tA;
                    length_[1] = p;
                    length_[2] = t_2pi - tB;
                    assert(tA >= 0.);
                    assert(p >= 0.);
                    assert(t_2pi - tB >= 0.);
                }
                double length() const
                {
                    return length_[0] + length_[1] + length_[2];
                }

                friend std::ostream& operator<<(std::ostream& os, const PathType& path);
                /** Path segment types */
                const std::vector<TrochoidPathSegmentType> *type_;
                /** Path segment lengths */
                double length_[3];
                /** Whether the path should be followed "in reverse" */
                bool reverse_{false};
            };

            TrochoidStateSpace(double turningRadius = 1.0, double windRatio = 0.0, double windDirection = 0.0,
             bool isSymmetric = false)
              : rho_(turningRadius), eta_(windRatio), psi_w_(windDirection), isSymmetric_(isSymmetric)
            {
                setName("Trochoid" + getName());
                type_ = STATE_SPACE_TROCHOID;
            }

            bool isMetricSpace() const override
            {
                return false;
            }

            double distance(const State *state1, const State *state2) const override;
            static double distance(const State *state1, const State *state2, double radius, double wind_ratio, double wind_heading);
            static double symmetricDistance(const State *state1, const State *state2, double radius, double wind_ratio, double wind_heading);

            void interpolate(const State *from, const State *to, double t, State *state) const override;
            virtual void interpolate(const State *from, const State *to, double t, bool &firstTime,
                                     PathType &path, State *state) const;
            virtual void interpolate(const State *from, const PathType &path, double t, State *state, double radius, double wind_ratio, double wind_heading) const;

            bool hasSymmetricDistance() const override
            {
                return isSymmetric_;
            }

            bool hasSymmetricInterpolate() const override
            {
                return isSymmetric_;
            }

            unsigned int validSegmentCount(const State *state1, const State *state2) const override;

            void sanityChecks() const override
            {
                double zero = std::numeric_limits<double>::epsilon();
                double eps = std::numeric_limits<float>::epsilon();
                int flags = ~(STATESPACE_INTERPOLATION | STATESPACE_TRIANGLE_INEQUALITY | STATESPACE_DISTANCE_BOUND);
                if (!isSymmetric_)
                    flags &= ~STATESPACE_DISTANCE_SYMMETRIC;
                StateSpace::sanityChecks(zero, eps, flags);
            }

            /** \brief Return a shortest Dubins path from SE(2) state state1 to SE(2) state state2 */
            PathType getPath(const State *state1, const State *state2, bool periodic=false) const;
            /** \brief Return a shortest Dubins path for a vehicle with given turning radius */
            static PathType getPath(const State *state1, const State *state2, double radius, double wind_ratio, double wind_heading, bool periodic=false);

        protected:
            /** \brief Turning radius */
            double rho_;
            
            /** \brief Wind ratio */
            double eta_;

            /** \brief Wind direction [0, 2pi] */
            double psi_w_;

            /** \brief Whether the distance is "symmetrized"

                If true the distance from state s1 to state s2 is the same as the
                distance from s2 to s1. This is done by taking the \b minimum
                length of the Dubins curves that connect s1 to s2 and s2 to s1. If
                isSymmetric_ is true, then the distance no longer satisfies the
                triangle inequality. */
            bool isSymmetric_;
        };
    }
}

#endif
