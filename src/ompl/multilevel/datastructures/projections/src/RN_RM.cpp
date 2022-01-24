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

#include <ompl/multilevel/datastructures/projections/RN_RM.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

using namespace ompl::multilevel;

Projection_RN_RM::Projection_RN_RM(ompl::base::StateSpacePtr BundleSpace, ompl::base::StateSpacePtr BaseSpace)
  : BaseT(BundleSpace, BaseSpace)
{
    setType(PROJECTION_RN_RM);
}

void Projection_RN_RM::projectFiber(const ompl::base::State *xBundle, ompl::base::State *xFiber) const
{
    const auto *xBundle_RN = xBundle->as<base::RealVectorStateSpace::StateType>();

    auto *xFiber_RM = xFiber->as<base::RealVectorStateSpace::StateType>();

    for (unsigned int k = getBaseDimension(); k < getDimension(); k++)
    {
        xFiber_RM->values[k - getBaseDimension()] = xBundle_RN->values[k];
    }
}

void Projection_RN_RM::project(const ompl::base::State *xBundle, ompl::base::State *xBase) const
{
    const auto *xBundle_RN = xBundle->as<base::RealVectorStateSpace::StateType>();
    auto *xBase_RM = xBase->as<base::RealVectorStateSpace::StateType>();

    for (unsigned int k = 0; k < getBaseDimension(); k++)
    {
        xBase_RM->values[k] = xBundle_RN->values[k];
    }
}

void Projection_RN_RM::lift(const ompl::base::State *xBase, const ompl::base::State *xFiber,
                            ompl::base::State *xBundle) const
{
    auto *xBundle_RN = xBundle->as<base::RealVectorStateSpace::StateType>();
    const auto *xBase_RM = xBase->as<base::RealVectorStateSpace::StateType>();
    const auto *xFiber_RJ = xFiber->as<base::RealVectorStateSpace::StateType>();

    for (unsigned int k = 0; k < getBaseDimension(); k++)
    {
        xBundle_RN->values[k] = xBase_RM->values[k];
    }
    for (unsigned int k = getBaseDimension(); k < getDimension(); k++)
    {
        xBundle_RN->values[k] = xFiber_RJ->values[k - getBaseDimension()];
    }
}

ompl::base::StateSpacePtr Projection_RN_RM::computeFiberSpace()
{
    unsigned int N1 = getDimension();
    unsigned int N0 = getBaseDimension();
    unsigned int NX = N1 - N0;
    base::StateSpacePtr FiberSpace = std::make_shared<base::RealVectorStateSpace>(NX);
    base::RealVectorBounds Bundle_bounds =
        std::static_pointer_cast<base::RealVectorStateSpace>(getBundle())->getBounds();

    std::vector<double> low;
    low.resize(NX);
    std::vector<double> high;
    high.resize(NX);
    base::RealVectorBounds Fiber_bounds(NX);
    for (unsigned int k = 0; k < NX; k++)
    {
        Fiber_bounds.setLow(k, Bundle_bounds.low.at(k + N0));
        Fiber_bounds.setHigh(k, Bundle_bounds.high.at(k + N0));
    }
    std::static_pointer_cast<base::RealVectorStateSpace>(FiberSpace)->setBounds(Fiber_bounds);
    return FiberSpace;
}
