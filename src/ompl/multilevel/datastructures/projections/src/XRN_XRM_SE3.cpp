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

#include <ompl/multilevel/datastructures/projections/XRN_XRM_SE3.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/SO3StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

using namespace ompl::multilevel;

Projection_SE3RN_SE3RM::Projection_SE3RN_SE3RM(ompl::base::StateSpacePtr BundleSpace,
                                               ompl::base::StateSpacePtr BaseSpace)
  : BaseT(BundleSpace, BaseSpace)
{
    setType(PROJECTION_SE3RN_SE3RM);
}

void Projection_SE3RN_SE3RM::project(const ompl::base::State *xBundle, ompl::base::State *xBase) const
{
    const auto *xBundle_SE3 = xBundle->as<base::CompoundState>()->as<base::SE3StateSpace::StateType>(0);
    const auto *xBundle_SO3 = &xBundle_SE3->rotation();
    const auto *xBundle_RN = xBundle->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

    auto *xBase_SE3 = xBase->as<base::CompoundState>()->as<base::SE3StateSpace::StateType>(0);
    auto *xBase_SO3 = &xBase_SE3->rotation();
    auto *xBase_RM = xBase->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

    xBase_SE3->setXYZ(xBundle_SE3->getX(), xBundle_SE3->getY(), xBundle_SE3->getZ());
    xBase_SO3->x = xBundle_SO3->x;
    xBase_SO3->y = xBundle_SO3->y;
    xBase_SO3->z = xBundle_SO3->z;
    xBase_SO3->w = xBundle_SO3->w;

    for (unsigned int k = 0; k < getBaseDimension() - 6; k++)
    {
        xBase_RM->values[k] = xBundle_RN->values[k];
    }
}

void Projection_SE3RN_SE3RM::lift(const ompl::base::State *xBase, const ompl::base::State *xFiber,
                                  ompl::base::State *xBundle) const
{
    auto *xBundle_SE3 = xBundle->as<base::CompoundState>()->as<base::SE3StateSpace::StateType>(0);
    auto *xBundle_SO3 = &xBundle_SE3->rotation();
    auto *xBundle_RN = xBundle->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

    const auto *xBase_SE3 = xBase->as<base::CompoundState>()->as<base::SE3StateSpace::StateType>(0);
    const auto *xBase_SO3 = &xBase_SE3->rotation();
    const auto *xBase_RM = xBase->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

    const auto *xFiber_RJ = xFiber->as<base::RealVectorStateSpace::StateType>();

    xBundle_SE3->setXYZ(xBase_SE3->getX(), xBase_SE3->getY(), xBase_SE3->getZ());
    xBundle_SO3->x = xBase_SO3->x;
    xBundle_SO3->y = xBase_SO3->y;
    xBundle_SO3->z = xBase_SO3->z;
    xBundle_SO3->w = xBase_SO3->w;

    //[X Y Z YAW PITCH ROLL] [1...M-1][M...N-1]
    // SE3                                        RN
    unsigned int M = getDimension() - getFiberDimension() - 6;
    unsigned int N = getFiberDimension();

    for (unsigned int k = 0; k < M; k++)
    {
        xBundle_RN->values[k] = xBase_RM->values[k];
    }
    for (unsigned int k = M; k < M + N; k++)
    {
        xBundle_RN->values[k] = xFiber_RJ->values[k - M];
    }
}
