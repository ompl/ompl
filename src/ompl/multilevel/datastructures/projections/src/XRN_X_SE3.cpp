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

#include <ompl/multilevel/datastructures/projections/XRN_X_SE3.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/SO3StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

using namespace ompl::multilevel;

Projection_SE3RN_SE3::Projection_SE3RN_SE3(ompl::base::StateSpacePtr BundleSpace, ompl::base::StateSpacePtr BaseSpace)
  : BaseT(BundleSpace, BaseSpace)
{
    setType(PROJECTION_SE3RN_SE3);
}

void Projection_SE3RN_SE3::project(const ompl::base::State *xBundle, ompl::base::State *xBase) const
{
    const auto *xBundle_SE3 = xBundle->as<base::CompoundState>()->as<base::SE3StateSpace::StateType>(0);
    const auto *xBundle_SO3 = &xBundle_SE3->rotation();

    auto *xBase_SE3 = xBase->as<base::SE3StateSpace::StateType>();
    auto *xBase_SO3 = &xBase_SE3->rotation();

    xBase_SE3->setXYZ(xBundle_SE3->getX(), xBundle_SE3->getY(), xBundle_SE3->getZ());
    xBase_SO3->x = xBundle_SO3->x;
    xBase_SO3->y = xBundle_SO3->y;
    xBase_SO3->z = xBundle_SO3->z;
    xBase_SO3->w = xBundle_SO3->w;
}

void Projection_SE3RN_SE3::lift(const ompl::base::State *xBase, const ompl::base::State *xFiber,
                                ompl::base::State *xBundle) const
{
    auto *xBundle_SE3 = xBundle->as<base::CompoundState>()->as<base::SE3StateSpace::StateType>(0);
    auto *xBundle_SO3 = &xBundle_SE3->rotation();
    auto *xBundle_RN = xBundle->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

    const auto *xBase_SE3 = xBase->as<base::SE3StateSpace::StateType>();
    const auto *xBase_SO3 = &xBase_SE3->rotation();

    const auto *xFiber_RN = xFiber->as<base::RealVectorStateSpace::StateType>();

    xBundle_SE3->setXYZ(xBase_SE3->getX(), xBase_SE3->getY(), xBase_SE3->getZ());
    xBundle_SO3->x = xBase_SO3->x;
    xBundle_SO3->y = xBase_SO3->y;
    xBundle_SO3->z = xBase_SO3->z;
    xBundle_SO3->w = xBase_SO3->w;

    for (unsigned int k = 0; k < getFiberDimension(); k++)
    {
        xBundle_RN->values[k] = xFiber_RN->values[k];
    }
}
