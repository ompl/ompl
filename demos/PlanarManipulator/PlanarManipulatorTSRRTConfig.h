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

#ifndef PLANAR_MANIPULATOR_TSRRT_CONFIG_H_
#define PLANAR_MANIPULATOR_TSRRT_CONFIG_H_

#include "PlanarManipulator.h"

#include <ompl/geometric/planners/rrt/TSRRT.h>
#include <ompl/util/RandomNumbers.h>

class PlanarManipTaskSpaceConfig : public ompl::geometric::TaskSpaceConfig
{
public:
    PlanarManipTaskSpaceConfig(const PlanarManipulator *manip, const ompl::base::RealVectorBounds &bounds)
      : manip_(manip), task_space_bounds_(bounds)
    {
    }

    virtual int getDimension() const
    {
        return 2;
    }

    virtual void project(const ompl::base::State *state, Eigen::Ref<Eigen::VectorXd> ts_proj) const
    {
        project(state->as<PlanarManipulatorStateSpace::StateType>()->values, ts_proj);
    }

    virtual void sample(Eigen::Ref<Eigen::VectorXd> ts_proj) const
    {
        // Sample point uniformly in task-space
        for (unsigned int dim = 0; dim < task_space_bounds_.low.size(); ++dim)
        {
            ts_proj[dim] = rng_.uniformReal(task_space_bounds_.low[dim], task_space_bounds_.high[dim]);
        }
    }

    // Given a point in task-space, generate a configuraton space state
    // that projects to this point.  seed is the nearest task-space neighbor.
    // Returns false if a lifted configuration could not be generated.
    virtual bool lift(const Eigen::Ref<Eigen::VectorXd> &ts_proj, const ompl::base::State *seed,
                      ompl::base::State *state) const
    {
        const PlanarManipulatorStateSpace::StateType *pm_seed = seed->as<PlanarManipulatorStateSpace::StateType>();
        std::vector<double> seed_angles(manip_->getNumLinks());
        for (unsigned int ix = 0; ix < seed_angles.size(); ++ix)
        {
            seed_angles[ix] = (*pm_seed)[ix];
        }

        // Allow any angle.
        Eigen::Affine2d end_frame(Eigen::Rotation2Dd(0.0) * Eigen::Translation2d(ts_proj[0], ts_proj[1]));

        std::vector<double> lifted_angles;
        if (!manip_->FABRIK(lifted_angles, seed_angles, end_frame, /*xyTol=*/1e-4,
                            /*thetaTol=*/2.0 * M_PI))
        {
            return false;
        }

        PlanarManipulatorStateSpace::StateType *pm_state = state->as<PlanarManipulatorStateSpace::StateType>();
        for (unsigned int ix = 0; ix < manip_->getNumLinks(); ++ix)
        {
            (*pm_state)[ix] = lifted_angles[ix];
        }
        return true;
    }

private:
    // Forward kinematics to find the end-effector position (proj) in the task space.
    void project(const double *angles, Eigen::Ref<Eigen::VectorXd> proj) const
    {
        Eigen::Affine2d frame;
        manip_->FK(angles, frame);
        proj[0] = frame.translation()(0);
        proj[1] = frame.translation()(1);
    }

    const PlanarManipulator *manip_;
    const ompl::base::RealVectorBounds task_space_bounds_;
    mutable ompl::RNG rng_;
};

#endif
