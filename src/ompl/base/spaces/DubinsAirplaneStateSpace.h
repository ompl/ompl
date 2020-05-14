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

#ifndef OMPL_BASE_SPACES_DUBINS_AIRPLANE_STATE_SPACE_
#define OMPL_BASE_SPACES_DUBINS_AIRPLANE_STATE_SPACE_

#include <boost/math/constants/constants.hpp>
#include <boost/optional/optional.hpp>
#include <ompl/base/MotionValidator.h>
#include <ompl/base/spaces/SimpleSE3StateSpace.h>

namespace ompl
{
    namespace base
    {
        //Copy from github repo by https://github.com/bendike
        class DubinsAirplaneStateSpace : public SimpleSE3StateSpace
        {
        public:
            /** \brief The DubinsAirplane path segment type */
            enum DubinsAirplanePathSegmentType
            {
                DUBINS_LEFT = 0,
                DUBINS_STRAIGHT = 1,
                DUBINS_RIGHT = 2
            };

            class Helix
            {
            public:
                double n = 0;
                double rho_ = 0.0;
                double climbAngle_ = 0.0;
                DubinsAirplanePathSegmentType type;  // used to determine the turn direction of the helix

                Helix(DubinsAirplanePathSegmentType type, double r, double a, double n)
                  : n(n), rho_(r), climbAngle_(a), type(type)
                {
                }

                double projectedLength() const
                {
                    return n * 2 * M_PI * rho_;
                }

                double length() const
                {
                    return n * 2 * M_PI * rho_ / cos(climbAngle_);
                }
            };

            /** \brief DubinsAirplane path types */
            static const DubinsAirplanePathSegmentType dubinsPathType[6][3];

            /** \brief Complete description of a DubinsAirplane path */
            class DubinsAirplanePath
            {
            public:
                DubinsAirplanePath(const DubinsAirplanePathSegmentType *type = dubinsPathType[0], double t = 0.,
                                   double p = std::numeric_limits<double>::max(), double q = 0.)
                  : type_(type), helix_(nullptr)
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
                const DubinsAirplanePathSegmentType *type_;
                /** Path segment lengths */
                double length_[3];
                /** Path climb angle */
                double climbAngle_;
                /** Helix*/
                Helix *helix_;
                // boost::optional<Helix> helix_;
                /** Whether the path should be followed "in reverse" */
                bool reverse_{false};
            };

            DubinsAirplaneStateSpace(double turningRadius = 1.0, double climbAngleLimit = 1.0, bool isSymmetric = false)
              : rho_(turningRadius), climbAngleLimit_(climbAngleLimit), isSymmetric_(isSymmetric)
            {
                type_ = STATE_SPACE_DUBINS_AIRPLANE;
            }

            bool isMetricSpace() const override
            {
                return false;
            }

            double distance(const State *state1, const State *state2) const override;

            void interpolate(const State *from, const State *to, double t, State *state) const override;
            virtual void interpolate(const State *from, const State *to, double t, bool &firstTime,
                                     DubinsAirplanePath &path, State *state) const;

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

            /** \brief Return the shortest DubinsAirplane path from SE(2) state state1 to SE(2) state state2 */
            DubinsAirplanePath dubins(const State *state1, const State *state2) const;

        protected:
            virtual void interpolate(const State *from, const DubinsAirplanePath &path, double t, State *state) const;

            /** \brief Turning radius */
            double rho_;

            /** \brief Climb angle limit */
            double climbAngleLimit_;

            /** \brief Whether the distance is "symmetrized"

                If true the distance from state s1 to state s2 is the same as the
                distance from s2 to s1. This is done by taking the \b minimum
                length of the DubinsAirplane curves that connect s1 to s2 and s2 to s1. If
                isSymmetric_ is true, then the distance no longer satisfies the
                triangle inequality. */
            bool isSymmetric_;
        };

        /** \brief A DubinsAirplane motion validator that only uses the state validity checker.
            Motions are checked for validity at a specified resolution.

            This motion validator is almost identical to the DiscreteMotionValidator
            except that it remembers the optimal DubinsAirplanePath between different calls to
            interpolate. */
        class DubinsAirplaneMotionValidator : public MotionValidator
        {
        public:
            DubinsAirplaneMotionValidator(SpaceInformation *si) : MotionValidator(si)
            {
                defaultSettings();
            }
            DubinsAirplaneMotionValidator(const SpaceInformationPtr &si) : MotionValidator(si)
            {
                defaultSettings();
            }
            ~DubinsAirplaneMotionValidator() override = default;
            bool checkMotion(const State *s1, const State *s2) const override;
            bool checkMotion(const State *s1, const State *s2, std::pair<State *, double> &lastValid) const override;

        private:
            DubinsAirplaneStateSpace *stateSpace_;
            void defaultSettings();
        };
    }  // namespace base
}  // namespace ompl

#endif
