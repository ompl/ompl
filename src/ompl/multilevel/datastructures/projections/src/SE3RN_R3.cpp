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

#include <ompl/multilevel/datastructures/projections/SE3RN_R3.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/SO3StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

using namespace ompl::multilevel;

Projection_SE3RN_R3::Projection_SE3RN_R3(ompl::base::StateSpacePtr BundleSpace, ompl::base::StateSpacePtr BaseSpace)
  : BaseT(BundleSpace, BaseSpace)
{
    setType(PROJECTION_SE3RN_R3);
}

void Projection_SE3RN_R3::projectFiber(const ompl::base::State *xBundle, ompl::base::State *xFiber) const
{
    const auto *xBundle_SE3 = xBundle->as<base::CompoundState>()->as<base::SE3StateSpace::StateType>(0);
    const auto *xBundle_SO3 = &xBundle_SE3->rotation();
    const auto *xBundle_RN = xBundle->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

    auto *xFiber_SO3 = xFiber->as<base::CompoundState>()->as<base::SO3StateSpace::StateType>(0);
    auto *xFiber_RN = xFiber->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

    xFiber_SO3->x = xBundle_SO3->x;
    xFiber_SO3->y = xBundle_SO3->y;
    xFiber_SO3->z = xBundle_SO3->z;
    xFiber_SO3->w = xBundle_SO3->w;

    for (unsigned int k = 0; k < getFiberDimension() - 3; k++)
    {
        xFiber_RN->values[k] = xBundle_RN->values[k];
    }
}

void Projection_SE3RN_R3::project(const ompl::base::State *xBundle, ompl::base::State *xBase) const
{
    const auto *xBundle_SE3 = xBundle->as<base::CompoundState>()->as<base::SE3StateSpace::StateType>(0);
    auto *xBase_R3 = xBase->as<base::RealVectorStateSpace::StateType>();

    xBase_R3->values[0] = xBundle_SE3->getX();
    xBase_R3->values[1] = xBundle_SE3->getY();
    xBase_R3->values[2] = xBundle_SE3->getZ();
}

void Projection_SE3RN_R3::lift(const ompl::base::State *xBase, const ompl::base::State *xFiber,
                               ompl::base::State *xBundle) const
{
    auto *xBundle_SE3 = xBundle->as<base::CompoundState>()->as<base::SE3StateSpace::StateType>(0);
    auto *xBundle_SO3 = &xBundle_SE3->rotation();
    auto *xBundle_RN = xBundle->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

    const auto *xBase_R3 = xBase->as<base::RealVectorStateSpace::StateType>();
    const auto *xFiber_SO3 = xFiber->as<base::CompoundState>()->as<base::SO3StateSpace::StateType>(0);
    const auto *xFiber_RN = xFiber->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

    xBundle_SE3->setXYZ(xBase_R3->values[0], xBase_R3->values[1], xBase_R3->values[2]);
    xBundle_SO3->x = xFiber_SO3->x;
    xBundle_SO3->y = xFiber_SO3->y;
    xBundle_SO3->z = xFiber_SO3->z;
    xBundle_SO3->w = xFiber_SO3->w;

    for (unsigned int k = 0; k < getFiberDimension() - 3; k++)
    {
        xBundle_RN->values[k] = xFiber_RN->values[k];
    }
}

ompl::base::StateSpacePtr Projection_SE3RN_R3::computeFiberSpace()
{
    base::CompoundStateSpace *Bundle_compound = getBundle()->as<base::CompoundStateSpace>();
    const std::vector<base::StateSpacePtr> Bundle_decomposed = Bundle_compound->getSubspaces();
    const std::vector<base::StateSpacePtr> Bundle_SE3_decomposed =
        Bundle_decomposed.at(0)->as<base::CompoundStateSpace>()->getSubspaces();

    const auto *Bundle_RN = Bundle_decomposed.at(1)->as<base::RealVectorStateSpace>();
    unsigned int N = Bundle_RN->getDimension();

    base::StateSpacePtr SO3(new base::SO3StateSpace());
    base::StateSpacePtr RN(new base::RealVectorStateSpace(N));
    RN->as<base::RealVectorStateSpace>()->setBounds(Bundle_RN->getBounds());

    return SO3 + RN;
}
