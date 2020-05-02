/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, Rice University
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
 *   * Neither the name of Rice University nor the names of its
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

#ifndef PLANAR_MANIPULATOR_IK_GOAL_H_
#define PLANAR_MANIPULATOR_IK_GOAL_H_

#include <ompl/base/goals/GoalLazySamples.h>
#include <eigen3/Eigen/Dense>
#include "PlanarManipulator.h"
#include "PlanarManipulatorStateSpace.h"

#include <boost/math/constants/constants.hpp>

#ifndef PI
#define PI boost::math::constants::pi<double>()
#define TWOPI boost::math::constants::two_pi<double>()
#endif

// A goal that allows for specification of position (and optionally, the
// orientation) for the end effector of a planar manipulator.
// Uses GoalLazySamples to sample valid IK positions
class PlanarManipulatorIKGoal : public ompl::base::GoalLazySamples
{
public:
    // If fixedOrientation is false, the orientation in the goalPose is not
    // considered (a random orientation will be sampled).
    PlanarManipulatorIKGoal(const ompl::base::SpaceInformationPtr &si, const Eigen::Affine2d &goalPose,
                            const PlanarManipulator *manipulator, bool fixedOrientation = true)
      : ompl::base::GoalLazySamples(
            si, [this](const ompl::base::GoalLazySamples *, ompl::base::State *st) { return sampleGoalThread(st); },
            true)
      , goalPose_(goalPose)
      , manipulator_(manipulator)
      , fixedOrientation_(fixedOrientation)
    {
    }

    virtual double distanceGoal(const ompl::base::State *st) const
    {
        const double *angles = st->as<PlanarManipulatorStateSpace::StateType>()->values;

        // Figure out where the end effector is
        Eigen::Affine2d eeFrame;
        manipulator_->FK(angles, eeFrame);

        double cartesianDist = (eeFrame.translation() - goalPose_.translation()).norm();

        // Orientation does not matter
        if (!fixedOrientation_)
            return cartesianDist;

        double eeRot = acos(eeFrame.matrix()(0, 0));
        double goalRot = acos(goalPose_.matrix()(0, 0));
        double angleDiff = fabs(goalRot - eeRot);

        // return the translational and rotational differences
        return cartesianDist + angleDiff;
    }

protected:
    bool sampleGoalThread(ompl::base::State *st) const
    {
        std::vector<double> seed(manipulator_->getNumLinks());
        std::vector<double> soln(manipulator_->getNumLinks());

        bool good = false;
        unsigned int maxTries = 1000;
        unsigned int tries = 0;
        do
        {
            // random seed
            for (size_t i = 0; i < seed.size(); ++i)
                seed[i] = rng_.uniformReal(-PI, PI);

            for (size_t i = 0; i < 10 && !good; ++i)
            {
                Eigen::Affine2d pose(goalPose_);
                if (!fixedOrientation_)
                {
                    // Sample the orientation if it does not matter
                    pose.rotate(rng_.uniformReal(-PI, PI));
                }

                if (manipulator_->FABRIK(soln, seed, pose))
                {
                    // copy values into state
                    memcpy(st->as<PlanarManipulatorStateSpace::StateType>()->values, &soln[0],
                           soln.size() * sizeof(double));
                    // GoalLazySamples will check validity
                    good = true;
                }
                tries++;
            }

        } while (!good && tries < maxTries);

        return good;
    }

    Eigen::Affine2d goalPose_;
    const PlanarManipulator *manipulator_;
    bool fixedOrientation_;
    mutable ompl::RNG rng_;
};

#endif
