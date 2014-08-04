/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, Rice University
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

/* Author: Ryan Luna */

#ifndef OMPL_GEOMETRIC_CONSTRAINTS_POSE_CONSTRAINT_
#define OMPL_GEOMETRIC_CONSTRAINTS_POSE_CONSTRAINT_

#include <ompl/util/RandomNumbers.h>
#include <ompl/util/Exception.h>

#include "ompl/base/Constraint.h"
#include "ompl/geometric/constraints/PositionConstraint.h"
#include "ompl/geometric/constraints/OrientationConstraint.h"

namespace ompl
{
    namespace geometric
    {
        /// @cond IGNORE
        OMPL_CLASS_FORWARD(PoseConstraint);
        /// @endcond

        /// \brief Definition of a pose function.  Given a state, return the pose that is to
        /// be constrained (position and orientation - Euler angles).
        typedef boost::function<void(const base::State*, Eigen::Vector3d& position, Eigen::Vector3d& rpy)> PoseFn;

        class PoseConstraint : public base::Constraint
        {
        public:

            /// \brief Definition of a 3D pose constraint on the system.  PoseFn
            /// is used to compute the pose for this constraint in order to
            /// evaluate whether or not the pose is achieved.  This constraint
            /// is abstract, and it is not possible to sample or project a state
            /// onto this constraint directly.  Derive a constraint from this class
            /// to perform sampling or projection.
            PoseConstraint(const base::StateSpacePtr& space,
                           PoseFn poseFn) : base::Constraint(space)
            {
                // Identity pose, with no tolerances
                pos_ = Eigen::Vector3d(0, 0, 0);
                rpy_ = Eigen::Vector3d(0, 0, 0);
                posTol_ = Eigen::Vector3d(1e-6, 1e-6, 1e-6);
                rpyTol_ = Eigen::Vector3d(1e-6, 1e-6, 1e-6);
                poseFn_ = poseFn;

                assert(poseFn_);
            }

            virtual ~PoseConstraint()
            {
            }

            /// \brief Set the desired position of the end effector
            virtual void setPosition(double x, double y, double z)
            {
                pos_[0] = x;
                pos_[1] = y;
                pos_[2] = z;
            }

            /// \brief Set the tolerance around the desired position of the
            /// end effector to the given value, for each axis
            virtual void setPositionTolerance(double tol)
            {
                setPositionTolerance(tol, tol, tol);
            }

            /// \brief Set the tolerance around the desired position of the
            /// end effector to the given values for each axis
            virtual void setPositionTolerance(double tolX, double tolY, double tolZ)
            {
                posTol_(0) = (tolX < 1e-6 ? 1e-6 : tolX);
                posTol_(1) = (tolY < 1e-6 ? 1e-6 : tolY);
                posTol_(2) = (tolZ < 1e-6 ? 1e-6 : tolZ);
            }

            /// \brief Set the desired orientation of the end effector to the
            /// given Euler angles
            virtual void setOrientation(double roll, double pitch, double yaw)
            {
                rpy_(0) = roll;
                rpy_(1) = pitch;
                rpy_(2) = yaw;
            }

            /// \brief Set the tolerances allowed for the orientation about
            /// each axis to the given value (in radians)
            virtual void setOrientationTolerance(double tol)
            {
                setOrientationTolerance(tol, tol, tol);
            }

            /// \brief Set the tolerances allowed for the orientation about
            /// each axis to the given values (in radians)
            virtual void setOrientationTolerance(double tolR, double tolP, double tolY)
            {
                rpyTol_(0) = (tolR < 1e-6 ? 1e-6 : tolR);
                rpyTol_(1) = (tolP < 1e-6 ? 1e-6 : tolP);
                rpyTol_(2) = (tolY < 1e-6 ? 1e-6 : tolY);
            }

            /// \brief Check whether this state satisfies the constraints
            virtual bool isSatisfied(const base::State* state) const
            {
                //return position_->isSatisfied(state) && orientation_->isSatisfied(state);
                Eigen::Vector3d position, orientation;
                poseFn_(state, position, orientation);
                return isPositionSatisfied(&position[0]) && isOrientationSatisfied(&orientation[0]);
            }

            /// \brief Check whether this pose satisfies the constraints
            virtual bool isSatisfied(const Eigen::Affine3d& pose) const
            {
                Eigen::Vector3d pos = pose.translation();

                Eigen::Vector3d rpy;
                rpy = pose.rotation().eulerAngles(0,1,2);
                return isPositionSatisfied(&pos[0]) && isOrientationSatisfied(&rpy[0]);
            }

            /// \brief Return the distance from satisfaction of a state
            virtual double distance(const base::State* state) const
            {
                //eturn position_->distance(state) + orientation_->distance(state);
                return std::numeric_limits<double>::max();
            }

            /// \brief Sample a state given the constraints.  If a state cannot
            /// be sampled, this method will return false.
            virtual bool sample(base::State* state)
            {
                OMPL_WARN("PoseConstraint::sample is NOT implemented");
                return false;
            }

            /// \brief Sample a pose given the constraints
            virtual bool sample(Eigen::Affine3d& pose)
            {
                // Due to numerical precision errors, only sampling within 90% of tolerance range
                Eigen::Affine3d translation = Eigen::Affine3d::Identity();
                for (unsigned int i = 0 ; i < 3; ++i)
                    translation.translation()[i] = rng_.uniformReal(pos_[i] - (posTol_[i]*0.90), pos_[i] + (posTol_[i]*0.90));

                Eigen::Matrix3d rotation(Eigen::AngleAxisd(rpy_[0], Eigen::Vector3d::UnitX()) *
                                         Eigen::AngleAxisd(rpy_[1], Eigen::Vector3d::UnitY()) *
                                         Eigen::AngleAxisd(rpy_[2], Eigen::Vector3d::UnitZ()));

                double ax = 2.0 * (rng_.uniform01() - 0.5) * (rpyTol_[0] * 0.10);
                double ay = 2.0 * (rng_.uniform01() - 0.5) * (rpyTol_[1] * 0.10);
                double az = 2.0 * (rng_.uniform01() - 0.5) * (rpyTol_[2] * 0.10);
                Eigen::Matrix3d delta(Eigen::AngleAxisd(ax, Eigen::Vector3d::UnitX()) *
                                      Eigen::AngleAxisd(ay, Eigen::Vector3d::UnitY()) *
                                      Eigen::AngleAxisd(az, Eigen::Vector3d::UnitZ()));

                pose = Eigen::Affine3d(rotation * delta);
                pose = pose * Eigen::Affine3d(translation);
                return true;
            }

            /// \brief Project a state given the constraints.  If a valid
            /// projection cannot be found, this method will return false.
            virtual bool project(base::State* state)
            {
                OMPL_WARN("PoseConstraint::project is NOT implemented");
                return false;
            }

        protected:
            bool isPositionSatisfied(const double* values) const
            {
                for (unsigned int i = 0; i < 3; ++i)
                {
                    if (fabs(values[i] - pos_[i]) > posTol_[i])
                        return false;
                }
                return true;
            }

            bool isOrientationSatisfied(const double* rpy) const
            {
                bool valid = true;
                for (size_t i = 0; i < 3 && valid; ++i)
                    valid = (fabs(rpy_(i) - rpy[i]) < rpyTol_(i));
                return valid;
            }


            RNG             rng_;
             // The nominal pose
            Eigen::Vector3d pos_;
            Eigen::Vector3d rpy_;

            // The tolerances allowed on the pose
            Eigen::Vector3d posTol_;
            Eigen::Vector3d rpyTol_;

            PoseFn          poseFn_;

        };
    }
}

#endif