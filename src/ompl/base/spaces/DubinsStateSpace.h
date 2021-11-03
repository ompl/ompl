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
#include "ompl/base/MotionValidator.h"
#include <boost/math/constants/constants.hpp>

namespace ompl
{
    namespace base
    {
        /** \brief An SE(2) state space where distance is measured by the
            length of Dubins curves.

            Note that this Dubins distance is \b not a proper distance metric,
            so nearest neighbor methods that rely on distance() being a metric
            (such as ompl::NearestNeighborsGNAT) will not always return the
            true nearest neighbors or get stuck in an infinite loop.

            The notation and solutions in the code are taken from:<br>
            A.M. Shkel and V. Lumelsky, “Classification of the Dubins set,”
            Robotics and Autonomous Systems, 34(4):179-202, 2001.
            DOI: <a href="http://dx.doi.org/10.1016/S0921-8890(00)00127-5">10.1016/S0921-8890(00)00127-5</a>

            The classification scheme described there is not actually used,
            since it only applies to “long” paths.
            */
        class DubinsStateSpace : public SE2StateSpace
        {
        public:
            /** \brief The Dubins path segment type */
            enum DubinsPathSegmentType
            {
                DUBINS_LEFT = 0,
                DUBINS_STRAIGHT = 1,
                DUBINS_RIGHT = 2
            };
            /** \brief Dubins path types */
            static const DubinsPathSegmentType dubinsPathType[6][3];
            /** \brief Complete description of a Dubins path */
            class DubinsPath
            {
            public:
                DubinsPath(const DubinsPathSegmentType *type = dubinsPathType[0], double t = 0.,
                           double p = std::numeric_limits<double>::max(), double q = 0.)
                  : type_(type)
                {
                    length_[0] = t;
                    length_[1] = p;
                    length_[2] = q;
                    assert(t >= 0.);
                    assert(p >= 0.);
                    assert(q >= 0.);
                }
                double length() const
                {
                    return length_[0] + length_[1] + length_[2];
                }

                /** Path segment types */
                const DubinsPathSegmentType *type_;
                /** Path segment lengths */
                double length_[3];
                /** Whether the path should be followed "in reverse" */
                bool reverse_{false};
            };

            DubinsStateSpace(double turningRadius = 1.0, bool isSymmetric = false)
              : rho_(turningRadius), isSymmetric_(isSymmetric)
            {
                type_ = STATE_SPACE_DUBINS;
            }

            bool isMetricSpace() const override
            {
                return false;
            }

            double distance(const State *state1, const State *state2) const override;

            void interpolate(const State *from, const State *to, double t, State *state) const override;
            virtual void interpolate(const State *from, const State *to, double t, bool &firstTime,
                                     DubinsPath &path, State *state) const;

            bool hasSymmetricDistance() const override
            {
                return isSymmetric_;
            }

            bool hasSymmetricInterpolate() const override
            {
                return isSymmetric_;
            }

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
            DubinsPath dubins(const State *state1, const State *state2) const;

        protected:
            virtual void interpolate(const State *from, const DubinsPath &path, double t, State *state) const;

            /** \brief Turning radius */
            double rho_;

            /** \brief Whether the distance is "symmetrized"

                If true the distance from state s1 to state s2 is the same as the
                distance from s2 to s1. This is done by taking the \b minimum
                length of the Dubins curves that connect s1 to s2 and s2 to s1. If
                isSymmetric_ is true, then the distance no longer satisfies the
                triangle inequality. */
            bool isSymmetric_;
        };

        /** \brief A Dubins motion validator that only uses the state validity checker.
            Motions are checked for validity at a specified resolution.

            This motion validator is almost identical to the DiscreteMotionValidator
            except that it remembers the optimal DubinsPath between different calls to
            interpolate. */
        class DubinsMotionValidator : public MotionValidator
        {
        public:
            DubinsMotionValidator(SpaceInformation *si) : MotionValidator(si)
            {
                defaultSettings();
            }
            DubinsMotionValidator(const SpaceInformationPtr &si) : MotionValidator(si)
            {
                defaultSettings();
            }
            ~DubinsMotionValidator() override = default;
            bool checkMotion(const State *s1, const State *s2) const override;
            bool checkMotion(const State *s1, const State *s2, std::pair<State *, double> &lastValid) const override;

        private:
            DubinsStateSpace *stateSpace_;
            void defaultSettings();
        };
    }
}

#endif
