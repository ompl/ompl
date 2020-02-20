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

#ifndef PLANAR_MANIPULATOR_PROJECTOR_H_
#define PLANAR_MANIPULATOR_PROJECTOR_H_

#include <vector>
#include <ompl/base/ProjectionEvaluator.h>
#include <ompl/tools/config/MagicConstants.h>
#include <boost/math/constants/constants.hpp>

#include "PlanarManipulator.h"

// A random, 2D projection for a planar kinematic chain
// Assuming the state space is a set of real numbers corresponding to the joint angles
class PlanarManipulatorProjectionEvaluator : public ompl::base::ProjectionEvaluator
{
public:
    PlanarManipulatorProjectionEvaluator(const ompl::base::StateSpace *space, const PlanarManipulator *manip)
      : ompl::base::ProjectionEvaluator(space), manip_(manip)
    {
    }

    virtual void defaultCellSizes()
    {
        cellSizes_.resize(getDimension());
        bounds_.resize(getDimension());
        for (size_t i = 0; i < getDimension(); ++i)
        {
            cellSizes_[i] = boost::math::constants::pi<double>() / ompl::magic::PROJECTION_DIMENSION_SPLITS;
            bounds_.low[i] = -boost::math::constants::pi<double>();
            bounds_.high[i] = boost::math::constants::pi<double>();
        }
    }

    virtual unsigned int getDimension(void) const
    {
        return 2;
    }

    void project(const ompl::base::State *state, Eigen::Ref<Eigen::VectorXd> projection) const
    {
        Eigen::Affine2d frame;
        manip_->FK(state->as<PlanarManipulatorStateSpace::StateType>()->values, frame);
        projection[0] = frame.translation()(0);
        projection[1] = frame.translation()(1);
    }

protected:
    ompl::base::ProjectionMatrix projectionMatrix_;
    const PlanarManipulator *manip_;
};

// A 4D projection for a planar kinematic chain
template <class T>
class PlanarManipulator4DProjectionEvaluator : public ompl::base::ProjectionEvaluator
{
public:
    PlanarManipulator4DProjectionEvaluator(const ompl::base::StateSpace *space, const PlanarManipulator *manip)
      : ompl::base::ProjectionEvaluator(space), manip_(manip)
    {
    }

    virtual void defaultCellSizes()
    {
        cellSizes_.resize(getDimension());
        bounds_.resize(getDimension());
        for (size_t i = 0; i < getDimension(); ++i)
        {
            cellSizes_[i] = boost::math::constants::pi<double>() / ompl::magic::PROJECTION_DIMENSION_SPLITS;
            bounds_.low[i] = -boost::math::constants::pi<double>();
            bounds_.high[i] = boost::math::constants::pi<double>();
        }
    }

    virtual unsigned int getDimension(void) const
    {
        return 4;
    }

    void project(const ompl::base::State *state, Eigen::Ref<Eigen::VectorXd> projection) const
    {
        std::vector<Eigen::Affine2d> frames;
        manip_->FK(state->as<T::StateType>()->values, frames);

        const Eigen::Affine2d midFrame = frames[(manip_->getNumLinks() / 2) - 1];
        projection[0] = midFrame.translation()(0);
        projection[1] = midFrame.translation()(1);

        const Eigen::Affine2d endFrame = frames.back();
        projection[2] = endFrame.translation()(0);
        projection[3] = endFrame.translation()(1);
    }

protected:
    const PlanarManipulator *manip_;
};

#endif