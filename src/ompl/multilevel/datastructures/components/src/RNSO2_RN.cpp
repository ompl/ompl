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

#include <ompl/multilevel/datastructures/components/RNSO2_RN.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

ompl::multilevel::ProjectionComponentWithFiber_RNSO2_RN::ProjectionComponentWithFiber_RNSO2_RN(ompl::base::StateSpacePtr BundleSpace, ompl::base::StateSpacePtr BaseSpace)
  : BaseT(BundleSpace, BaseSpace)
{
}

void ompl::multilevel::ProjectionComponentWithFiber_RNSO2_RN::projectFiber(const ompl::base::State *xBundle,
                                                                   ompl::base::State *xFiber) const
{
    const base::SO2StateSpace::StateType *xBundle_SO2 =
        xBundle->as<base::CompoundState>()->as<base::SO2StateSpace::StateType>(1);

    base::SO2StateSpace::StateType *xFiber_SO2 = xFiber->as<base::SO2StateSpace::StateType>();

    xFiber_SO2->value = xBundle_SO2->value;
}

void ompl::multilevel::ProjectionComponentWithFiber_RNSO2_RN::project(const ompl::base::State *xBundle,
                                                                  ompl::base::State *xBase) const
{
    const base::RealVectorStateSpace::StateType *xBundle_R3 =
        xBundle->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(0);
    base::RealVectorStateSpace::StateType *xBase_R3 = xBase->as<base::RealVectorStateSpace::StateType>();

    for (unsigned int k = 0; k < 3; k++)
    {
        xBase_R3->values[k] = xBundle_R3->values[k];
    }
}

void ompl::multilevel::ProjectionComponentWithFiber_RNSO2_RN::liftState(const ompl::base::State *xBase,
                                                                const ompl::base::State *xFiber,
                                                                ompl::base::State *xBundle) const
{
    base::RealVectorStateSpace::StateType *xBundle_R3 =
        xBundle->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(0);
    base::SO2StateSpace::StateType *xBundle_SO2 =
        xBundle->as<base::CompoundState>()->as<base::SO2StateSpace::StateType>(1);

    const base::SO2StateSpace::StateType *xFiber_SO2 = xFiber->as<base::SO2StateSpace::StateType>();

    const base::RealVectorStateSpace::StateType *xBase_R3 = xBase->as<base::RealVectorStateSpace::StateType>();

    for (unsigned int k = 0; k < 3; k++)
    {
        xBundle_R3->values[k] = xBase_R3->values[k];
    }
    xBundle_SO2->value = xFiber_SO2->value;
}

ompl::base::StateSpacePtr ompl::multilevel::ProjectionComponentWithFiber_RNSO2_RN::computeFiberSpace()
{
    unsigned int N = getDimension();
    unsigned int Y = getBaseDimension();
    if (Y != (N - 1))
    {
        OMPL_ERROR("Assumed input is SO(2)xRN -> RN, but got %d -> %d dimensions.", N, Y);
        throw "Invalid Dimensionality";
    }
    return std::make_shared<base::SO2StateSpace>();
}
