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

#include <ompl/multilevel/datastructures/projections/SO2N_SO2M.h>
#include <ompl/base/spaces/SO2StateSpace.h>

using namespace ompl::multilevel;

Projection_SO2N_SO2M::Projection_SO2N_SO2M(ompl::base::StateSpacePtr BundleSpace, ompl::base::StateSpacePtr BaseSpace)
  : BaseT(BundleSpace, BaseSpace)
{
    setType(PROJECTION_SO2N_SO2M);
}

void Projection_SO2N_SO2M::projectFiber(const ompl::base::State *xBundle, ompl::base::State *xFiber) const
{
    for (unsigned int k = 0; k < getFiberDimension(); k++)
    {
        const auto *SO2bundle =
            xBundle->as<base::CompoundState>()->as<base::SO2StateSpace::StateType>(k + getBaseDimension());
        auto *SO2fiber = xFiber->as<base::CompoundState>()->as<base::SO2StateSpace::StateType>(k);

        SO2fiber->value = SO2bundle->value;
    }
}

void Projection_SO2N_SO2M::project(const ompl::base::State *xBundle, ompl::base::State *xBase) const
{
    for (unsigned int k = 0; k < getBaseDimension(); k++)
    {
        const auto *SO2bundle = xBundle->as<base::CompoundState>()->as<base::SO2StateSpace::StateType>(k);

        base::SO2StateSpace::StateType *SO2base;

        if (getBaseDimension() <= 1)
        {
            SO2base = xBase->as<base::SO2StateSpace::StateType>();
        }
        else
        {
            SO2base = xBase->as<base::CompoundState>()->as<base::SO2StateSpace::StateType>(k);
        }

        SO2base->value = SO2bundle->value;
    }
}

void Projection_SO2N_SO2M::lift(const ompl::base::State *xBase, const ompl::base::State *xFiber,
                                ompl::base::State *xBundle) const
{
    for (unsigned int k = 0; k < getBaseDimension(); k++)
    {
        auto *SO2bundle = xBundle->as<base::CompoundState>()->as<base::SO2StateSpace::StateType>(k);
        const base::SO2StateSpace::StateType *SO2base;

        if (getBaseDimension() <= 1)
        {
            SO2base = xBase->as<base::SO2StateSpace::StateType>();
        }
        else
        {
            SO2base = xBase->as<base::CompoundState>()->as<base::SO2StateSpace::StateType>(k);
        }

        SO2bundle->value = SO2base->value;
    }
    for (unsigned int k = getBaseDimension(); k < getBaseDimension() + getFiberDimension(); k++)
    {
        auto *SO2bundle = xBundle->as<base::CompoundState>()->as<base::SO2StateSpace::StateType>(k);
        const auto *SO2fiber =
            xFiber->as<base::CompoundState>()->as<base::SO2StateSpace::StateType>(k - getBaseDimension());

        SO2bundle->value = SO2fiber->value;
    }
}

ompl::base::StateSpacePtr Projection_SO2N_SO2M::computeFiberSpace()
{
    unsigned int N = getDimension() - getBaseDimension();
    auto fiberSpace = std::make_shared<base::CompoundStateSpace>();
    for (unsigned int k = 0; k < N; k++)
    {
        base::StateSpacePtr SO2Space = std::make_shared<base::SO2StateSpace>();
        fiberSpace->addSubspace(SO2Space, 1.0);
    }
    return fiberSpace;
}
