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

#include <ompl/multilevel/datastructures/BundleSpaceComponent.h>
#include <ompl/util/Exception.h>

using namespace ompl::multilevel;

BundleSpaceComponent::BundleSpaceComponent(ompl::base::StateSpacePtr bundleSpace, ompl::base::StateSpacePtr baseSpace)
  : BundleSpaceProjection(bundleSpace, baseSpace)
{
    Fiber = std::make_shared<SpaceInformation>(fiberSpace);
    Fiber_sampler_ = Fiber->allocStateSampler();
}

void BundleSpaceComponent::liftState(
    const ompl::base::State *xBase, 
    ompl::base::State *xBundle) const
{
    sampleFiber(xFiberTmp_);
    liftState(xBase, xFiberTmp_, xBundle);
}

void BundleSpaceComponent::initFiberSpace()
{
    FiberSpace_ = computeFiberSpace();
}

ompl::base::StateSpacePtr BundleSpaceComponent::getFiberSpace() const
{
    return FiberSpace_;
}

unsigned int BundleSpaceComponent::getFiberDimension() const
{
    if (FiberSpace_)
        return FiberSpace_->getDimension();
    else
        return 0;
}

// unsigned int BundleSpaceComponent::getBaseDimension() const
// {
//     if (BaseSpace_)
//         return BaseSpace_->getDimension();
//     else
//         return 0;
// }

// unsigned int BundleSpaceComponent::getDimension() const
// {
//     return BundleSpace_->getDimension();
// }

// BundleSpaceComponentType BundleSpaceComponent::getType() const
// {
//     return type_;
// }

// void BundleSpaceComponent::setType(BundleSpaceComponentType &type)
// {
//     type_ = type;
// }

// std::string BundleSpaceComponent::stateTypeToString(ompl::base::StateSpacePtr space) const
// {
//     std::string tstr;
//     int type = space->getType();
//     if (type == base::STATE_SPACE_REAL_VECTOR)
//     {
//         int N = space->getDimension();
//         tstr = "R";
//         tstr += std::to_string(N);
//     }
//     else if (type == base::STATE_SPACE_SE2)
//     {
//         tstr = "SE2";
//     }
//     else if (type == base::STATE_SPACE_SE3)
//     {
//         tstr = "SE3";
//     }
//     else if (type == base::STATE_SPACE_SO2)
//     {
//         tstr = "SO2";
//     }
//     else if (type == base::STATE_SPACE_SO3)
//     {
//         tstr = "SO3";
//     }
//     else if (type == base::STATE_SPACE_TIME)
//     {
//         tstr = "T";
//     }
//     else if (space->isCompound())
//     {
//         base::CompoundStateSpace *space_compound = space->as<base::CompoundStateSpace>();
//         const std::vector<base::StateSpacePtr> space_decomposed = space_compound->getSubspaces();

//         for (unsigned int k = 0; k < space_decomposed.size(); k++)
//         {
//             base::StateSpacePtr s0 = space_decomposed.at(k);
//             tstr = tstr + stateTypeToString(s0);
//             if (k < space_decomposed.size() - 1)
//                 tstr += "x";
//         }
//     }
//     else
//     {
//         throw Exception("Unknown State Space");
//     }
//     return tstr;
// }

// std::string BundleSpaceComponent::getTypeAsString() const
// {
//     if (BaseSpace_)
//     {
//         std::string tstr = getBundleTypeAsString() + " -> " + getBaseTypeAsString();
//         if (type_ == BUNDLE_SPACE_CONSTRAINED_RELAXATION)
//         {
//             tstr += " (relaxation)";
//         }
//         else if (type_ == BUNDLE_SPACE_IDENTITY_PROJECTION)
//         {
//             tstr += " (identity)";
//         }
//         return tstr;
//     }
//     else
//     {
//         return getBundleTypeAsString();
//     }
// }

std::string BundleSpaceComponent::getFiberTypeAsString() const
{
    if (FiberSpace_)
        return stateTypeToString(FiberSpace_);
    else
        return "None";
}

// std::string BundleSpaceComponent::getBaseTypeAsString() const
// {
//     if (BaseSpace_)
//         return stateTypeToString(BaseSpace_);
//     else
//         return "None";
// }

// std::string BundleSpaceComponent::getBundleTypeAsString() const
// {
//     return stateTypeToString(BundleSpace_);
// }

// bool BundleSpaceComponent::isDynamic() const
// {
//     return isDynamic_;
// }

// void BundleSpaceComponent::print(std::ostream &out) const
// {
//     out << getTypeAsString() << std::endl;
// }

// namespace ompl
// {
//     namespace multilevel
//     {
//         std::ostream &operator<<(std::ostream &out, const BundleSpaceComponent &bundleSpaceComponent)
//         {
//             bundleSpaceComponent.print(out);
//             return out;
//         }
//     }
// }
