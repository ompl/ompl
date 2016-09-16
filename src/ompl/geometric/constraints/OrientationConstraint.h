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

#ifndef OMPL_GEOMETRIC_CONSTRAINTS_ORIENTATION_CONSTRAINT_
#define OMPL_GEOMETRIC_CONSTRAINTS_ORIENTATION_CONSTRAINT_

#include "ompl/base/Constraint.h"
#include "ompl/base/spaces/SO3StateSpace.h"
#include "ompl/util/RandomNumbers.h"

#include <boost/math/constants/constants.hpp>
#include <eigen3/Eigen/Dense>

#include <cmath>
#include <limits>

namespace ompl
{
    namespace geometric
    {
        /// @cond IGNORE
        OMPL_CLASS_FORWARD(OrientationConstraint);
        /// @endcond

        /// Representation of a constrained orientation in 3D space.  It is
        /// assumed that the portion of the state space constrained is of
        /// type base::SO3StateSpace at the given SubstateLocation \e loc.  This
        /// constraint is defined by a nominal orientation and tolerances about
        /// the x, y, and z axes.  The tolerance specifies a +/- value about
        /// each axis in radians.
        class OrientationConstraint : public base::Constraint
        {
        public:
            /// \brief Definition of an orientation constraint defined by
            /// a quaternion (orientation) and tolerances about each coordinate
            /// axis.
            OrientationConstraint(const base::StateSpacePtr &space, const base::StateSpace::SubstateLocation &loc,
                                  const base::State *orientation, double tolX = 0, double tolY = 0, double tolZ = 0)
              : base::Constraint(space), loc_(loc)
            {
                const base::SO3StateSpace::StateType *quat =
                    space_->getSubstateAtLocation(orientation, loc_)->as<base::SO3StateSpace::StateType>();

                quat_ = Eigen::Quaterniond(quat->w, quat->x, quat->y, quat->z);
                quat_.normalize();
                desired_matrix_ = quat_.toRotationMatrix();
                rpy_ = desired_matrix_.eulerAngles(0, 1, 2);  // rpy, in that order

                // Adding a numerical wiggle factor to tolerances
                double tol = 1e-6;

                tolX_ = tolX + tol;
                tolY_ = tolY + tol;
                tolZ_ = tolZ + tol;
            }

            /// \brief Definition of an orientation constraint defined by
            /// Euler angles and tolerances about each coordinate axis.
            OrientationConstraint(const base::StateSpacePtr &space, const base::StateSpace::SubstateLocation &loc,
                                  double roll, double pitch, double yaw, double tolX = 0, double tolY = 0,
                                  double tolZ = 0)
              : base::Constraint(space), loc_(loc)
            {
                desired_matrix_ = Eigen::Affine3d(Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) *
                                                  Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                                                  Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()))
                                      .rotation();

                quat_ = Eigen::Quaterniond(desired_matrix_);
                quat_.normalize();
                rpy_ = desired_matrix_.eulerAngles(0, 1, 2);  // rpy, in that order

                // Adding a numerical wiggle factor to tolerances
                double tol = 1e-6;

                tolX_ = tolX + tol;
                tolY_ = tolY + tol;
                tolZ_ = tolZ + tol;
            }

            virtual ~OrientationConstraint()
            {
            }

            /// \brief Check whether this state satisfies the constraints
            /// Very important to ensure that the input quaternion is normalized.
            virtual bool isSatisfied(const base::State *state) const
            {
                const base::SO3StateSpace::StateType *substate =
                    space_->getSubstateAtLocation(state, loc_)->as<base::SO3StateSpace::StateType>();
                Eigen::Quaterniond q = Eigen::Quaterniond(substate->w, substate->x, substate->y, substate->z);

                Eigen::Vector3d rpy;
                rpy = q.toRotationMatrix().eulerAngles(0, 1, 2);  // assuming input state is normalized quaternion

                bool result = true;
                result &= fabs(rpy_(0) - rpy(0)) < tolX_ + std::numeric_limits<double>::epsilon();
                result &= fabs(rpy_(1) - rpy(1)) < tolY_ + std::numeric_limits<double>::epsilon();
                result &= fabs(rpy_(2) - rpy(2)) < tolZ_ + std::numeric_limits<double>::epsilon();

                return result;
            }

            virtual bool isSatisfied(const double *rpy) const
            {
                bool result = true;
                result &= fabs(rpy_(0) - rpy[0]) < tolX_ + std::numeric_limits<double>::epsilon();
                result &= fabs(rpy_(1) - rpy[1]) < tolY_ + std::numeric_limits<double>::epsilon();
                result &= fabs(rpy_(2) - rpy[2]) < tolZ_ + std::numeric_limits<double>::epsilon();

                return result;
            }

            /// \brief Return the distance from satisfaction of a state
            /// A state that satisfies the constraint should have distance 0.
            /// This is not implemented
            virtual double distance(const base::State * /*state*/) const
            {
                return std::numeric_limits<double>::max();
            }

            /// \brief Sample a state given the constraints.  If a state cannot
            /// be sampled, this method will return false.
            virtual bool sample(base::State *state)
            {
                // Sample a rotation matrix within the tolerances
                double ax = 2.0 * (rng_.uniform01() - 0.5) * tolX_;
                double ay = 2.0 * (rng_.uniform01() - 0.5) * tolY_;
                double az = 2.0 * (rng_.uniform01() - 0.5) * tolZ_;

                Eigen::Affine3d delta(Eigen::AngleAxisd(ax, Eigen::Vector3d::UnitX()) *
                                      Eigen::AngleAxisd(ay, Eigen::Vector3d::UnitY()) *
                                      Eigen::AngleAxisd(az, Eigen::Vector3d::UnitZ()));

                // Sampled orientation is the desired orientation perturbed by the sampled tolerances
                Eigen::Affine3d sample(desired_matrix_ * delta);
                Eigen::Quaterniond q(sample.rotation());

                // Copy the elements of q into the state
                base::SO3StateSpace::StateType *substate =
                    space_->getSubstateAtLocation(state, loc_)->as<base::SO3StateSpace::StateType>();
                substate->w = q.w();
                substate->x = q.x();
                substate->y = q.y();
                substate->z = q.z();

                return true;
            }

            /// \brief Project a state given the constraints.  If a valid
            /// projection cannot be found, this method will return false.
            virtual bool project(base::State *state)
            {
                // TODO: Should we first check for isSatisfied?
                if (!isSatisfied(state))
                    return sample(state);
                return true;
            }

            /// \brief Return the internal roll, pitch, and yaw angles used
            /// to check whether a state satisfies this orientation constraint
            const Eigen::Vector3d &getDesiredRPY() const
            {
                return rpy_;
            }

            /// \brief Set the desired orientation using a quaternion.  \e state
            /// should be of type SO3StateSpace::StateType.
            void setDesiredOrientation(const base::State *state)
            {
                const base::SO3StateSpace::StateType *quat =
                    space_->getSubstateAtLocation(state, loc_)->as<base::SO3StateSpace::StateType>();

                quat_ = Eigen::Quaterniond(quat->w, quat->x, quat->y, quat->z);
                quat_.normalize();
                desired_matrix_ = quat_.toRotationMatrix();
                rpy_ = desired_matrix_.eulerAngles(0, 1, 2);  // rpy, in that order
            }

            /// \brief Set the desired orientation using Euler angles.
            void setDesiredOrientation(double roll, double pitch, double yaw)
            {
                desired_matrix_ = Eigen::Affine3d(Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) *
                                                  Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                                                  Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()))
                                      .rotation();

                quat_ = Eigen::Quaterniond(desired_matrix_);
                quat_.normalize();
                rpy_ = desired_matrix_.eulerAngles(0, 1, 2);  // rpy, in that order
            }

            /// \brief Set the tolerances allowed about each axis to the
            /// given value (in radians)
            void setTolerance(double tol)
            {
                setTolerance(tol, tol, tol);
            }

            /// \brief Set the tolerances allowed about each axis to the
            /// given values (in radians)
            void setTolerance(double tolX, double tolY, double tolZ)
            {
                // Adding a numerical wiggle factor to tolerances
                double tol = 1e-6;

                tolX_ = tolX + tol;
                tolY_ = tolY + tol;
                tolZ_ = tolZ + tol;
            }

        protected:
            base::StateSpace::SubstateLocation loc_;
            RNG rng_;

            /// \brief The rotation matrix induced by the desired orientation
            Eigen::Matrix3d desired_matrix_;
            /// \brief The quaternion induced by the desired orientation
            Eigen::Quaterniond quat_;
            /// \brief The roll, pitch, and yaw for the nominal orientation
            /// (rotations about X, Y, then Z)
            Eigen::Vector3d rpy_;

            /// \brief The tolerance allowed from the nominal orientation about each axis.
            double tolX_, tolY_, tolZ_;
        };
    }
}

#endif