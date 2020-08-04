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

#include <ompl/multilevel/datastructures/components/Identity.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

ompl::multilevel::BundleSpaceComponent_Identity::BundleSpaceComponent_Identity(base::StateSpacePtr BundleSpace,
                                                                              base::StateSpacePtr BaseSpace)
  : BaseT(BundleSpace, BaseSpace)
{
}

void ompl::multilevel::BundleSpaceComponent_Identity::projectFiber(const ompl::base::State *, ompl::base::State *) const
{
    OMPL_WARN("Trying to project to fiber of Identity Bundle Space.");
}

void ompl::multilevel::BundleSpaceComponent_Identity::projectBase(const ompl::base::State *xBundle,
                                                                 ompl::base::State *xBase) const
{
    BundleSpace_->copyState(xBase, xBundle);
}

void ompl::multilevel::BundleSpaceComponent_Identity::liftState(const ompl::base::State *xBase,
                                                               const ompl::base::State *,
                                                               ompl::base::State *xBundle) const
{
    BundleSpace_->copyState(xBundle, xBase);
}

ompl::base::StateSpacePtr ompl::multilevel::BundleSpaceComponent_Identity::computeFiberSpace()
{
    return std::make_shared<base::RealVectorStateSpace>(0);
}
