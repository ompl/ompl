/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021,
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

#include <ompl/multilevel/datastructures/Projection.h>
#include <ompl/util/Exception.h>

using namespace ompl::base;
using namespace ompl::multilevel;

Projection::Projection(ompl::base::StateSpacePtr bundleSpace, ompl::base::StateSpacePtr baseSpace)
  : bundleSpace_(bundleSpace), baseSpace_(baseSpace)
{
}

bool Projection::isAdmissible() const
{
    OMPL_WARN("NYI");
    return false;
}

unsigned int Projection::getBaseDimension() const
{
    if (baseSpace_)
    {
        return baseSpace_->getDimension();
    }
    else
    {
        return 0;
    }
}

unsigned int Projection::getDimension() const
{
    return bundleSpace_->getDimension();
}

unsigned int Projection::getCoDimension() const
{
    return getDimension() - getBaseDimension();
}

ompl::base::StateSpacePtr Projection::getBundle() const
{
    return bundleSpace_;
}

ompl::base::StateSpacePtr Projection::getBase() const
{
    return baseSpace_;
}

bool Projection::isFibered() const
{
    return false;
}

ProjectionType Projection::getType() const
{
    return type_;
}

void Projection::setType(const ProjectionType type)
{
    type_ = type;
}

std::string Projection::stateTypeToString(ompl::base::StateSpacePtr space) const
{
    std::string tstr;
    int type = space->getType();
    if (type == base::STATE_SPACE_REAL_VECTOR)
    {
        int N = space->getDimension();
        tstr = "R";
        tstr += std::to_string(N);
    }
    else if (type == base::STATE_SPACE_SE2)
    {
        tstr = "SE2";
    }
    else if (type == base::STATE_SPACE_SE3)
    {
        tstr = "SE3";
    }
    else if (type == base::STATE_SPACE_SO2)
    {
        tstr = "SO2";
    }
    else if (type == base::STATE_SPACE_SO3)
    {
        tstr = "SO3";
    }
    else if (type == base::STATE_SPACE_TIME)
    {
        tstr = "T";
    }
    else if (space->isCompound())
    {
        base::CompoundStateSpace *space_compound = space->as<base::CompoundStateSpace>();
        const std::vector<base::StateSpacePtr> space_decomposed = space_compound->getSubspaces();

        for (unsigned int k = 0; k < space_decomposed.size(); k++)
        {
            base::StateSpacePtr s0 = space_decomposed.at(k);
            tstr = tstr + stateTypeToString(s0);
            if (k < space_decomposed.size() - 1)
                tstr += "x";
        }
    }
    else
    {
        throw Exception("Unknown State Space");
    }
    return tstr;
}

std::string Projection::getTypeAsString() const
{
    if (baseSpace_)
    {
        std::string tstr = getBundleTypeAsString() + " -> " + getBaseTypeAsString();
        if (type_ == PROJECTION_CONSTRAINED_RELAXATION)
        {
            tstr += " (relax)";
        }
        else if (type_ == PROJECTION_IDENTITY)
        {
            tstr += " (id)";
        }
        return tstr;
    }
    else
    {
        return getBundleTypeAsString();
    }
}

std::string Projection::getBaseTypeAsString() const
{
    if (baseSpace_)
        return stateTypeToString(baseSpace_);
    else
        return "None";
}

std::string Projection::getBundleTypeAsString() const
{
    return stateTypeToString(bundleSpace_);
}

void Projection::print(std::ostream &out) const
{
    out << getTypeAsString() << std::endl;
}

namespace ompl
{
    namespace multilevel
    {
        std::ostream &operator<<(std::ostream &out, const Projection &projection)
        {
            projection.print(out);
            return out;
        }
    }
}

CompoundProjection::CompoundProjection(const StateSpacePtr &bundleSpace, const StateSpacePtr &baseSpace,
                                       const std::vector<ProjectionPtr> &components)
  : Projection(bundleSpace, baseSpace), components_(components)
{
    setType(PROJECTION_COMPOUND);
}

void CompoundProjection::lift(const State *xBase, State *xBundle) const
{
    unsigned int M = components_.size();

    if (M > 1)
    {
        for (unsigned int m = 0; m < M; m++)
        {
            const State *xmBase = xBase->as<CompoundState>()->as<State>(m);
            State *xmBundle = xBundle->as<CompoundState>()->as<State>(m);
            components_.at(m)->lift(xmBase, xmBundle);
        }
    }
    else
    {
        components_.front()->lift(xBase, xBundle);
    }
}

void CompoundProjection::project(const State *xBundle, State *xBase) const
{
    unsigned int M = components_.size();

    if (M > 1)
    {
        for (unsigned int m = 0; m < M; m++)
        {
            if (components_.at(m)->getBaseDimension() > 0)
            {
                const State *xmBundle = xBundle->as<CompoundState>()->as<State>(m);
                State *xmBase = xBase->as<CompoundState>()->as<State>(m);
                components_.at(m)->project(xmBundle, xmBase);
            }
        }
    }
    else
    {
        components_.front()->project(xBundle, xBase);
    }
}

unsigned int CompoundProjection::getDimension() const
{
    if (components_.size() > 0)
    {
        return components_.front()->getDimension();
    }
    else
    {
        return 0;
    }
}

unsigned int CompoundProjection::getCoDimension() const
{
    if (components_.size() > 0)
    {
        return components_.front()->getCoDimension();
    }
    else
    {
        return 0;
    }
}
unsigned int CompoundProjection::getBaseDimension() const
{
    if (components_.size() > 0)
    {
        return components_.front()->getBaseDimension();
    }
    else
    {
        return 0;
    }
}

bool CompoundProjection::isFibered() const
{
    for (unsigned int k = 0; k < components_.size(); k++)
    {
        if (!components_.at(k)->isFibered())
            return false;
    }
    return true;
}

void CompoundProjection::print(std::ostream &out) const
{
    for (unsigned int k = 0; k < components_.size(); k++)
    {
        out << components_.at(k) << "|";
    }
    out << std::endl;
}
