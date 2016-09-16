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

#ifndef OMPL_GEOMETRIC_CONSTRAINTS_KINEMATIC_CONSTRAINT_
#define OMPL_GEOMETRIC_CONSTRAINTS_KINEMATIC_CONSTRAINT_

#include <boost/function.hpp>
#include <limits>
#include <vector>

#include "ompl/base/Constraint.h"
#include "ompl/geometric/constraints/PoseConstraint.h"

#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SO3StateSpace.h>
#include <ompl/util/Exception.h>

namespace ompl
{
    namespace geometric
    {
        /// Representation of a kinematic constraint.  This constraint is highly
        /// abstract and relies on callback functions to perform forward and
        /// inverse kinematics.
        class KinematicConstraint : public base::Constraint
        {
        public:
            // \brief A function definition for forward kinematics.  For the given state,
            /// compute the global reference frame of each link in the kinematic chain
            /// Assumed that the number of frames returned will be # links + 1
            /// (origin and end effector frames are included).
            typedef boost::function<void(const base::State *, std::vector<Eigen::Affine3d> &)> ForwardKinematicsFn;

            /// \brief A function definition for inverse kinematics.  For the given poses
            /// (for given link index), compute the joint positions for the kinematic
            /// chain that will achieve the poses.  If this computation fails, this
            /// function should return false.
            typedef boost::function<bool(base::State *, const std::map<unsigned int, Eigen::Affine3d> &poses)>
                InverseKinematicsFn;

            /// \brief Constructor.  Takes (entire) state space and the location of the (sub)space that
            /// is being constrained.  The number of links in the constrained portion of the state space
            /// is required, along with function pointers to a forward and an inverse kinematics routine.
            KinematicConstraint(const base::StateSpacePtr &space, const base::StateSpace::SubstateLocation &loc,
                                unsigned int numLinks, ForwardKinematicsFn fk, InverseKinematicsFn ik)
              : base::Constraint(space), loc_(loc), numLinks_(numLinks), fk_(fk), ik_(ik)
            {
                constraints_.resize(numLinks_ + 1);

                if (!fk_)
                    throw Exception("%s: Forward kinematics function is NULL", __FUNCTION__);
                if (!ik_)
                    throw Exception("%s: Inverse kinematics function is NULL", __FUNCTION__);
            }

            virtual ~KinematicConstraint()
            {
            }

            /// \brief Check whether this state satisfies the constraints
            virtual bool isSatisfied(const base::State *state) const
            {
                // Get a pointer to the portion of the state space
                // that this constraint refers to
                const base::State *substate = space_->getSubstateAtLocation(state, loc_);

                // Compute global reference frames using FK
                std::vector<Eigen::Affine3d> frames;
                fk_(substate, frames);

                bool valid = true;
                for (size_t i = 0; i < constraints_.size() && valid; ++i)
                {
                    if (constraints_[i])
                        valid = constraints_[i]->isSatisfied(frames[i]);
                }

                return valid;
            }

            /// \brief Return the distance from satisfaction of a state
            /// This is not implemented for kinematic constraints
            virtual double distance(const base::State *state) const
            {
                return std::numeric_limits<double>::max();
            }

            /// \brief Sample a state given the constraints.  If a state cannot
            /// be sampled, this method will return false.
            virtual bool sample(base::State *state)
            {
                // Get a pointer to the substate to sample
                base::State *substate = space_->getSubstateAtLocation(state, loc_);

                // Need to sample each constraint
                std::map<unsigned int, Eigen::Affine3d> poses;
                for (size_t i = 0; i < constraints_.size(); ++i)
                {
                    if (!constraints_[i])
                        continue;

                    Eigen::Affine3d pose;
                    constraints_[i]->sample(pose);
                    poses[i] = pose;
                }

                return ik_(substate, poses);
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

            virtual void setPoseConstraint(unsigned int n, const PoseConstraintPtr &pose)
            {
                constraints_[n] = pose;
            }

            virtual void setPoseConstraint(unsigned int n, double x, double y, double z, double roll, double pitch,
                                           double yaw)
            {
                PoseConstraintPtr pose(
                    new PoseConstraint(space_, boost::bind(&KinematicConstraint::getPose, this, _1, _2, _3, n)));
                pose->setPosition(x, y, z);
                pose->setOrientation(roll, pitch, yaw);
                constraints_[n] = pose;
            }

        protected:
            // Callback for PoseConstraint.  Returns the pose of the given frame (link)
            void getPose(const base::State *state, Eigen::Vector3d &position, Eigen::Vector3d &rpy,
                         unsigned int link) const
            {
                // Get a pointer to the portion of the state space
                // that this constraint refers to
                const base::State *substate = space_->getSubstateAtLocation(state, loc_);

                // Compute global reference frames using FK
                std::vector<Eigen::Affine3d> frames;
                fk_(substate, frames);

                position = frames[link].translation();
                rpy = frames[link].rotation().eulerAngles(0, 1, 2);  // rpy, in that order
            }

            base::StateSpace::SubstateLocation loc_;
            unsigned int numLinks_;
            ForwardKinematicsFn fk_;
            InverseKinematicsFn ik_;

            /// \brief The pose constraint for each frame of the kinematic system
            std::vector<PoseConstraintPtr> constraints_;
        };
    }
}

#endif
