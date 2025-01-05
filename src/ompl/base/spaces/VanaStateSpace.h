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

#ifndef OMPL_BASE_SPACES_VANA_STATE_SPACE_
#define OMPL_BASE_SPACES_VANA_STATE_SPACE_

#include "ompl/base/spaces/DubinsStateSpace.h"
#include <optional>

namespace ompl::base
{
    /** \brief An R^4 x SO(2) state space where distance is measured by the
        length of a type Dubins airplane curves.

        Note that distance in this space is \b not a proper distance metric,
        so nearest neighbor methods that rely on distance() being a metric
        (such as ompl::NearestNeighborsGNAT) will not always return the
        true nearest neighbors or get stuck in an infinite loop.

        See the following reference for details:

        [1] P. Váňa, A. Alves Neto, J. Faigl, and D. G. Macharet, “Minimal 3D Dubins
        Path with Bounded Curvature and Pitch Angle,” in 2020 IEEE International
        Conference on Robotics and Automation (ICRA), May 2020, pp. 8497–8503.
        doi: 10.1109/ICRA40945.2020.9197084.
    */
    class VanaStateSpace : public CompoundStateSpace
    {
    public:
        class PathType
        {
        public:
            PathType();
            PathType(const PathType &path);
            ~PathType();
            double length() const;
            PathType &operator=(const PathType &path);

            friend std::ostream &operator<<(std::ostream &os, const PathType &path);

            double horizontalRadius_{1.};
            double verticalRadius_{1.};

            DubinsStateSpace::DubinsPath pathXY_;
            DubinsStateSpace::DubinsPath pathSZ_;
            DubinsStateSpace::StateType *startSZ_{nullptr};
        };

        /** A state in R^4 x SO(2): (x, y, z, pitch, yaw) */
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
            double pitch() const
            {
                return as<RealVectorStateSpace::StateType>(0)->values[3];
            }
            double &pitch()
            {
                return as<RealVectorStateSpace::StateType>(0)->values[3];
            }
        };

        VanaStateSpace(double turningRadius = 1.0, double maxPitch = boost::math::double_constants::sixth_pi);
        VanaStateSpace(double turningRadius, std::pair<double, double> pitchRange);
        ~VanaStateSpace() override = default;

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
            RealVectorBounds b(bounds);
            if (bounds.low.size() < 4)
            {
                b.resize(4);
                b.low[3] = minPitch_;
                b.high[3] = maxPitch_;
            }
            as<RealVectorStateSpace>(0)->setBounds(b);
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
         * \brief Compute the state that lies at time @e t in [0,1] on the segment @e path starting from state @e from.
         *
         * \param from start state
         * \param path a segment
         * \param t fraction of distance along segment @path. Has to be between 0 and 1 (boundaries included).
         * \param state the state that lies on the segment at fraction @e t of the distance along @e path,
         * starting from state @e from.
         */
        virtual void interpolate(const State *from, const PathType &path, double t, State *state) const;

        /**
         * \brief Compute a 3D Dubins path using the model and algorithm proposed by Vana et al.
         *
         * \param state1 start state
         * \param state2 end state
         * \return a 3D Dubins path if one was found, std::nullopt_t otherwise
         */
        std::optional<PathType> getPath(const State *state1, const State *state2) const;

        /**
         * \brief Helper function used by getPath
         *
         * \param state1 start state
         * \param state2 end state
         * \param radius turning radius for the motion in the XY plane
         * \param path resulting path
         * \param endSZ working memory
         * \return true iff a feasible solution was found
         */
        bool decoupled(const State *state1, const State *state2, double radius, PathType &path,
                       DubinsStateSpace::StateType *endSZ) const;

    protected:
        /**
         * \brief Allocate a DubinsSpace state and initialize it
         *
         * \param x initial X coordinate of allocated state
         * \param y initial Y coordinate of allocated state
         * \param yaw initial yaw coordinate of allocated state
         * \return new allocated and initialized state
         */
        DubinsStateSpace::StateType *get2DPose(double x, double y, double yaw) const;

        /** Turning radius */
        double rho_;
        /** Minimum pitch in radians */
        double minPitch_;
        /** Maximum pitch in radians */
        double maxPitch_;
        /** Tolerance used to determine convergence when searching for optimal turning radius */
        double tolerance_{1e-8};
        /** Auxiliary space to compute paths in SE(2) slices of state space */
        DubinsStateSpace dubinsSpace_;
    };
}  // namespace ompl::base

#endif
