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

#include <ompl/multilevel/datastructures/projections/SE3_R3.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/SO3StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

using namespace ompl::multilevel;

Projection_SE3_R3::Projection_SE3_R3(ompl::base::StateSpacePtr BundleSpace, ompl::base::StateSpacePtr BaseSpace)
  : BaseT(BundleSpace, BaseSpace)
{
    setType(PROJECTION_SE3_R3);
}

void Projection_SE3_R3::projectFiber(const ompl::base::State *xBundle, ompl::base::State *xFiber) const
{
    const auto *xBundle_SE3 = xBundle->as<base::SE3StateSpace::StateType>();
    const auto *xBundle_SO3 = &xBundle_SE3->rotation();
    auto *xFiber_SO3 = xFiber->as<base::SO3StateSpace::StateType>();

    xFiber_SO3->x = xBundle_SO3->x;
    xFiber_SO3->y = xBundle_SO3->y;
    xFiber_SO3->z = xBundle_SO3->z;
    xFiber_SO3->w = xBundle_SO3->w;
}

void Projection_SE3_R3::project(const ompl::base::State *xBundle, ompl::base::State *xBase) const
{
    const auto *xBundle_SE3 = xBundle->as<base::SE3StateSpace::StateType>();
    auto *xBase_R3 = xBase->as<base::RealVectorStateSpace::StateType>();
    xBase_R3->values[0] = xBundle_SE3->getX();
    xBase_R3->values[1] = xBundle_SE3->getY();
    xBase_R3->values[2] = xBundle_SE3->getZ();
}

void Projection_SE3_R3::lift(const ompl::base::State *xBase, const ompl::base::State *xFiber,
                             ompl::base::State *xBundle) const
{
    auto *xBundle_SE3 = xBundle->as<base::SE3StateSpace::StateType>();
    auto *xBundle_SO3 = &xBundle_SE3->rotation();

    const auto *xBase_R3 = xBase->as<base::RealVectorStateSpace::StateType>();
    const auto *xFiber_SO3 = xFiber->as<base::SO3StateSpace::StateType>();

    xBundle_SE3->setXYZ(xBase_R3->values[0], xBase_R3->values[1], xBase_R3->values[2]);

    xBundle_SO3->x = xFiber_SO3->x;
    xBundle_SO3->y = xFiber_SO3->y;
    xBundle_SO3->z = xFiber_SO3->z;
    xBundle_SO3->w = xFiber_SO3->w;
}

ompl::base::StateSpacePtr Projection_SE3_R3::computeFiberSpace()
{
    unsigned int N = getDimension();
    unsigned int Y = getBaseDimension();
    if (N != 6 && Y != 3)
    {
        OMPL_ERROR("Assumed input is SE(3) -> R3, but got %d -> %d dimensions.", N, Y);
        throw "Invalid Dimensionality";
    }
    return std::make_shared<base::SO3StateSpace>();
}
