/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2024, Metron, Inc.
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
 *   * Neither the name of the Metron, Inc. nor the names of its
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

#ifndef OMPL_BASE_SPACES_OWEN_STATE_SPACE_
#define OMPL_BASE_SPACES_OWEN_STATE_SPACE_

#include "ompl/base/spaces/DubinsStateSpace.h"
#include <optional>

namespace ompl::base
{
    /** \brief An R^3 x SO(2) state space where distance is measured by the
        length of a type Dubins airplane curves.

        Note that distance in this space is \b not a proper distance metric,
        so nearest neighbor methods that rely on distance() being a metric
        (such as ompl::NearestNeighborsGNAT) will not always return the
        true nearest neighbors or get stuck in an infinite loop.

        See the following reference for details:

       [1] M. Owen, R. W. Beard, and T. W. McLain, “Implementing Dubins
       airplane paths on fixed-wing UAVs,” in Handbook of Unmanned Aerial
       Vehicles, Springer, 2014, pp. 1677–1701.
       doi: 10.1007/978-90-481-9707-1 120
    */
    class OwenStateSpace : public CompoundStateSpace
    {
    public:
        enum PathCategory : char
        {
            LOW_ALTITUDE = 'L',
            MEDIUM_ALTITUDE = 'M',
            HIGH_ALTITUDE = 'H',
            UNKNOWN = '?'
        };

        class PathType
        {
        public:
            PathType(DubinsStateSpace::DubinsPath const &path, double turnRadius, double deltaZ,
                     unsigned int numTurns = 0)
              : path_(path), turnRadius_(turnRadius), deltaZ_(deltaZ), numTurns_(numTurns)
            {
            }
            PathType(DubinsStateSpace::DubinsPath const &path, double turnRadius, double deltaZ, double phi)
              : path_(path), turnRadius_(turnRadius), deltaZ_(deltaZ), phi_(phi)
            {
            }
            double length() const;

            PathCategory category() const;

            friend std::ostream &operator<<(std::ostream &os, const PathType &path);

            DubinsStateSpace::DubinsPath path_;
            double turnRadius_{1.};
            double deltaZ_;
            double phi_{0.};
            unsigned int numTurns_{0};
        };

        /** A state in R^3 x SO(2): (x, y, z, yaw) */
        class StateType : public CompoundStateSpace::StateType
        {
        public:
            StateType() = default;
            double operator[](unsigned index) const
            {
                return as<RealVectorStateSpace::StateType>(0)->values[index];
            }
            double &operator[](unsigned index)
            {
                return as<RealVectorStateSpace::StateType>(0)->values[index];
            }
            double yaw() const
            {
                return as<SO2StateSpace::StateType>(1)->value;
            }
            double &yaw()
            {
                return as<SO2StateSpace::StateType>(1)->value;
            }
        };

        OwenStateSpace(double turningRadius = 1.0, double maxPitch = boost::math::double_constants::sixth_pi);
        ~OwenStateSpace() override = default;

        bool isMetricSpace() const override
        {
            return false;
        }
        bool hasSymmetricDistance() const override
        {
            return false;
        }

        bool hasSymmetricInterpolate() const override
        {
            return false;
        }

        void sanityChecks() const override
        {
            double zero = std::numeric_limits<double>::epsilon();
            double eps = std::numeric_limits<float>::epsilon();
            int flags = ~(STATESPACE_INTERPOLATION | STATESPACE_TRIANGLE_INEQUALITY | STATESPACE_DISTANCE_BOUND |
                          STATESPACE_DISTANCE_SYMMETRIC);
            StateSpace::sanityChecks(zero, eps, flags);
        }

        /** \copydoc RealVectorStateSpace::setBounds() */
        void setBounds(const RealVectorBounds &bounds)
        {
            as<RealVectorStateSpace>(0)->setBounds(bounds);
        }

        /** \copydoc RealVectorStateSpace::getBounds() */
        const RealVectorBounds &getBounds() const
        {
            return as<RealVectorStateSpace>(0)->getBounds();
        }

        /** Set tolerance for convergence on optimal turning radius */
        void setTolerance(double tolerance)
        {
            tolerance_ = tolerance;
        }
        /** Get tolerance for convergence on optimal turning radius */
        double getTolerance()
        {
            return tolerance_;
        }

        State *allocState() const override;
        void registerProjections() override;

        double distance(const State *state1, const State *state2) const override;

        unsigned int validSegmentCount(const State *state1, const State *state2) const override;

        void interpolate(const State *from, const State *to, double t, State *state) const override;
        /**
         * \brief Compute the state that lies at time @e t in [0,1] on the segment that connects @e from state to @e to
         * state.
         *
         * \param from start state
         * \param to end state
         * \param t fraction of distance along segment between @e from and @e to. Has to be between
         * 0 and 1 (boundaries included).
         * \param path the segment connecting @from and @to.
         * \param state the state that lies on the segment at fraction @e t of the distance between
         * @e from and @e to.
         */
        virtual void interpolate(const State *from, const State *to, double t, PathType &path, State *state) const;

        /**
         * \brief Compute a 3D Dubins path using the model and algorithm proposed by Owen et al.
         *
         * \param state1 start state
         * \param state2 end state
         * \return a 3D Dubins path if one was found, std::nullopt_t otherwise
         */
        std::optional<PathType> getPath(const State *state1, const State *state2) const;

    protected:
        /**
         * \brief Compute the SE(2) state after making a turn
         */
        void turn(const State *from, double turnRadius, double angle, State *state) const;

        /** Turning radius */
        double rho_;
        /** Tan(pitch), where pitch is the maximum pitch in radians */
        double tanMaxPitch_;
        /** Tolerance used to determine convergence when searching for optimal turning radius */
        double tolerance_{1e-8};
        /** Auxiliary space to compute paths in SE(2) slices of state space */
        DubinsStateSpace dubinsSpace_;
    };
}  // namespace ompl::base

#endif
