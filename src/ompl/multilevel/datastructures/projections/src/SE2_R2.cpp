/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020,
 *  Max Planck Institute for Intelligent Systems (MPI-IS).
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
 *   * Neither the name of the MPI-IS nor the names
 *     of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written
 *     permission.
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

/* Author: Andreas Orthey */

#include <ompl/multilevel/datastructures/projections/SE2_R2.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

using namespace ompl::multilevel;

Projection_SE2_R2::Projection_SE2_R2(ompl::base::StateSpacePtr BundleSpace, ompl::base::StateSpacePtr BaseSpace)
  : BaseT(BundleSpace, BaseSpace)
{
    setType(PROJECTION_SE2_R2);
}

void Projection_SE2_R2::projectFiber(const ompl::base::State *xBundle, ompl::base::State *xFiber) const
{
    const auto *xBundle_SE2 = xBundle->as<base::SE2StateSpace::StateType>();
    auto *xFiber_SO2 = xFiber->as<base::SO2StateSpace::StateType>();
    xFiber_SO2->value = xBundle_SE2->getYaw();
}

void Projection_SE2_R2::project(const ompl::base::State *xBundle, ompl::base::State *xBase) const
{
    const auto *xBundle_SE2 = xBundle->as<base::SE2StateSpace::StateType>();
    auto *xBase_R2 = xBase->as<base::RealVectorStateSpace::StateType>();
    xBase_R2->values[0] = xBundle_SE2->getX();
    xBase_R2->values[1] = xBundle_SE2->getY();
}

void Projection_SE2_R2::lift(const ompl::base::State *xBase, const ompl::base::State *xFiber,
                             ompl::base::State *xBundle) const
{
    auto *xBundle_SE2 = xBundle->as<base::SE2StateSpace::StateType>();
    const auto *xBase_R2 = xBase->as<base::RealVectorStateSpace::StateType>();
    const auto *xFiber_SO2 = xFiber->as<base::SO2StateSpace::StateType>();

    xBundle_SE2->setXY(xBase_R2->values[0], xBase_R2->values[1]);
    xBundle_SE2->setYaw(xFiber_SO2->value);
}

ompl::base::StateSpacePtr Projection_SE2_R2::computeFiberSpace()
{
    unsigned int N = getDimension();
    unsigned int Y = getBaseDimension();
    if (N != 3 && Y != 2)
    {
        OMPL_ERROR("Assumed input is SE(2) -> R2, but got %d -> %d dimensions.", N, Y);
        throw "Invalid Dimensionality";
    }
    return std::make_shared<base::SO2StateSpace>();
}
