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

#include <ompl/multilevel/datastructures/projections/SE2RN_R2.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

#include <ompl/util/Exception.h>

using namespace ompl::multilevel;

Projection_SE2RN_R2::Projection_SE2RN_R2(ompl::base::StateSpacePtr BundleSpace, ompl::base::StateSpacePtr BaseSpace)
  : BaseT(BundleSpace, BaseSpace)
{
    setType(PROJECTION_SE2RN_R2);
}

void Projection_SE2RN_R2::projectFiber(const ompl::base::State *xBundle, ompl::base::State *xFiber) const
{
    const auto *xBundle_SE2 = xBundle->as<base::CompoundState>()->as<base::SE2StateSpace::StateType>(0);
    const auto *xBundle_RN = xBundle->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

    auto *xFiber_SO2 = xFiber->as<base::CompoundState>()->as<base::SO2StateSpace::StateType>(0);
    auto *xFiber_RN = xFiber->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

    xFiber_SO2->value = xBundle_SE2->getYaw();
    for (unsigned int k = 0; k < getFiberDimension() - 1; k++)
    {
        xFiber_RN->values[k] = xBundle_RN->values[k];
    }
}

void Projection_SE2RN_R2::project(const ompl::base::State *xBundle, ompl::base::State *xBase) const
{
    const auto *xBundle_SE2 = xBundle->as<base::CompoundState>()->as<base::SE2StateSpace::StateType>(0);
    auto *xBase_R2 = xBase->as<base::RealVectorStateSpace::StateType>();
    xBase_R2->values[0] = xBundle_SE2->getX();
    xBase_R2->values[1] = xBundle_SE2->getY();
}

void Projection_SE2RN_R2::lift(const ompl::base::State *xBase, const ompl::base::State *xFiber,
                               ompl::base::State *xBundle) const
{
    auto *xBundle_SE2 = xBundle->as<base::CompoundState>()->as<base::SE2StateSpace::StateType>(0);
    auto *xBundle_RN = xBundle->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

    const auto *xBase_R2 = xBase->as<base::RealVectorStateSpace::StateType>();

    const auto *xFiber_SO2 = xFiber->as<base::CompoundState>()->as<base::SO2StateSpace::StateType>(0);
    const auto *xFiber_RN = xFiber->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

    xBundle_SE2->setX(xBase_R2->values[0]);
    xBundle_SE2->setY(xBase_R2->values[1]);
    xBundle_SE2->setYaw(xFiber_SO2->value);

    for (unsigned int k = 0; k < getFiberDimension() - 1; k++)
    {
        xBundle_RN->values[k] = xFiber_RN->values[k];
    }
}

ompl::base::StateSpacePtr Projection_SE2RN_R2::computeFiberSpace()
{
    unsigned int N = getDimension();
    unsigned int Y = getBaseDimension();
    if (N > 3 && Y != 2)
    {
        OMPL_ERROR("Assumed input is SE(2)xRN -> R2, but got %d -> %d dimensions.", N, Y);
        throw Exception("Invalid Dimensionality");
    }

    base::CompoundStateSpace *Bundle_compound = getBundle()->as<base::CompoundStateSpace>();
    const std::vector<base::StateSpacePtr> Bundle_decomposed = Bundle_compound->getSubspaces();
    const std::vector<base::StateSpacePtr> Bundle_SE2_decomposed =
        Bundle_decomposed.at(0)->as<base::CompoundStateSpace>()->getSubspaces();

    const auto *Bundle_RN = Bundle_decomposed.at(1)->as<base::RealVectorStateSpace>();
    unsigned int N_RN = Bundle_RN->getDimension();

    base::StateSpacePtr SO2(new base::SO2StateSpace());
    base::StateSpacePtr RN(new base::RealVectorStateSpace(N_RN));
    RN->as<base::RealVectorStateSpace>()->setBounds(Bundle_RN->getBounds());

    return SO2 + RN;
}
