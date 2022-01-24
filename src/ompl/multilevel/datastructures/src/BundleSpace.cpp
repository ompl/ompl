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

#include <ompl/multilevel/datastructures/BundleSpace.h>
#include <ompl/multilevel/datastructures/Projection.h>

#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/SO3StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/TimeStateSpace.h>
#include <ompl/base/spaces/DiscreteStateSpace.h>
#include <ompl/tools/config/MagicConstants.h>

#include <ompl/util/Exception.h>
#include <cmath>  //to use isnan(d)

using namespace ompl::base;
using namespace ompl::multilevel;

unsigned int BundleSpace::counter_ = 0;

BundleSpace::BundleSpace(const SpaceInformationPtr &si, BundleSpace *child)
  : Planner(si, "BundleSpace"), childBundleSpace_(child), totalSpace_(si)
{
    id_ = counter_++;
    if (child)
    {
        baseSpace_ = childBundleSpace_->getBundle();
        childBundleSpace_->setParent(this);
        xBaseTmp_ = getBase()->allocState();
    }

    std::stringstream ss;
    ss << (*this);
    OMPL_DEBUG(ss.str().c_str());

    if (!Bundle_valid_sampler_)
    {
        Bundle_valid_sampler_ = getBundle()->allocValidStateSampler();
    }
    if (!Bundle_sampler_)
    {
        Bundle_sampler_ = getBundle()->allocStateSampler();
    }
    xBundleTmp_ = getBundle()->allocState();
}

BundleSpace::~BundleSpace()
{
    if (hasBaseSpace())
    {
        if (xBaseTmp_)
        {
            getBase()->freeState(xBaseTmp_);
        }
    }
    if (xBundleTmp_)
    {
        getBundle()->freeState(xBundleTmp_);
    }
}

bool BundleSpace::makeProjection()
{
    ProjectionFactory projectionFactory;

    projection_ = projectionFactory.makeProjection(getBundle(), getBase());

    if (!projection_)
        return false;

    sanityChecks();
    return true;
}

bool BundleSpace::hasBaseSpace() const
{
    return !(baseSpace_ == nullptr);
}

bool BundleSpace::findSection()
{
    return false;
}

bool BundleSpace::hasParent() const
{
    return !(parentBundleSpace_ == nullptr);
}

bool BundleSpace::isDynamic() const
{
    return isDynamic_;
}

void BundleSpace::setup()
{
    BaseT::setup();

    hasSolution_ = false;
    firstRun_ = true;

    if (pdef_)
    {
        if (!pdef_->hasOptimizationObjective())
        {
            OptimizationObjectivePtr lengthObj = std::make_shared<base::PathLengthOptimizationObjective>(getBundle());

            lengthObj->setCostThreshold(base::Cost(std::numeric_limits<double>::infinity()));
            pdef_->setOptimizationObjective(lengthObj);
        }
    }
    else
    {
        OMPL_ERROR("Called without ProblemDefinitionPtr");
        throw "NoProblemDef";
    }
}

GoalSampleableRegion *BundleSpace::getGoalPtr() const
{
    base::GoalSampleableRegion *goal = static_cast<base::GoalSampleableRegion *>(pdef_->getGoal().get());
    return goal;
}

void BundleSpace::clear()
{
    BaseT::clear();

    hasSolution_ = false;
    firstRun_ = true;

    pdef_->clearSolutionPaths();
}

void BundleSpace::sanityChecks() const
{
    const StateSpacePtr Bundle_space = getBundle()->getStateSpace();
    checkBundleSpaceMeasure("Bundle", Bundle_space);

    if (hasBaseSpace())
    {
        const StateSpacePtr Base_space = getBase()->getStateSpace();
        checkBundleSpaceMeasure("Base", Base_space);
        if (getProjection()->getDimension() != getBundleDimension())
        {
            throw Exception("BundleSpace Dimensions are wrong.");
        }
    }
}

void BundleSpace::checkBundleSpaceMeasure(std::string name, const StateSpacePtr space) const
{
    OMPL_DEVMSG1("%s dimension: %d measure: %f", name.c_str(), space->getDimension(), space->getMeasure());

    if ((space->getMeasure() >= std::numeric_limits<double>::infinity()))
    {
        throw Exception("Space infinite measure.");
    }
}

PlannerStatus BundleSpace::solve(const PlannerTerminationCondition &)
{
    throw Exception("A Bundle-Space cannot be solved alone. \
        Use class BundleSpaceSequence to solve Bundle-Spaces.");
}

void BundleSpace::setProblemDefinition(const ProblemDefinitionPtr &pdef)
{
    BaseT::setProblemDefinition(pdef);
}

void BundleSpace::resetCounter()
{
    BundleSpace::counter_ = 0;
}

void BundleSpace::setProjection(ProjectionPtr projection)
{
    projection_ = projection;
    if (getProjection() == nullptr)
    {
        OMPL_ERROR("Projection is nullptr.");
        throw "Projection is nullptr.";
    }
}

ProjectionPtr BundleSpace::getProjection() const
{
    return projection_;
}

void BundleSpace::allocIdentityState(State *s, StateSpacePtr space) const
{
    if (space->isCompound())
    {
        CompoundStateSpace *cspace = space->as<CompoundStateSpace>();
        const std::vector<StateSpacePtr> compounds = cspace->getSubspaces();
        for (unsigned int k = 0; k < compounds.size(); k++)
        {
            StateSpacePtr spacek = compounds.at(k);
            State *xk = s->as<CompoundState>()->as<State>(k);
            allocIdentityState(xk, spacek);
        }
    }
    else
    {
        int stype = space->getType();
        switch (stype)
        {
            case STATE_SPACE_SO3:
            {
                static_cast<SO3StateSpace::StateType *>(s)->setIdentity();
                break;
            }
            case STATE_SPACE_SO2:
            {
                static_cast<SO2StateSpace::StateType *>(s)->setIdentity();
                break;
            }
            case STATE_SPACE_TIME:
            {
                static_cast<TimeStateSpace::StateType *>(s)->position = 0;
                break;
            }
            case STATE_SPACE_DISCRETE:
            {
                DiscreteStateSpace *space_Discrete = space->as<DiscreteStateSpace>();
                int lb = space_Discrete->getLowerBound();
                static_cast<DiscreteStateSpace::StateType *>(s)->value = lb;
                break;
            }
            case STATE_SPACE_REAL_VECTOR:
            {
                RealVectorStateSpace::StateType *sRN = s->as<RealVectorStateSpace::StateType>();
                RealVectorStateSpace *RN = space->as<RealVectorStateSpace>();
                const std::vector<double> &bl = RN->getBounds().low;
                const std::vector<double> &bh = RN->getBounds().high;
                for (unsigned int k = 0; k < space->getDimension(); k++)
                {
                    double &v = sRN->values[k];
                    v = 0.0;

                    // if zero is not valid, use mid point as identity
                    if (v < bl.at(k) || v > bh.at(k))
                    {
                        v = bl.at(k) + 0.5 * (bh.at(k) - bl.at(k));
                    }
                }
                break;
            }
            default:
            {
                OMPL_ERROR("Type: %d not recognized.", stype);
                throw Exception("Type not recognized.");
            }
        }
    }
}

State *BundleSpace::allocIdentityState(StateSpacePtr space) const
{
    if (space != nullptr)
    {
        State *s = space->allocState();
        allocIdentityState(s, space);
        return s;
    }
    else
    {
        return nullptr;
    }
}

State *BundleSpace::allocIdentityStateBundle() const
{
    return allocIdentityState(getBundle()->getStateSpace());
}

State *BundleSpace::allocIdentityStateBase() const
{
    return allocIdentityState(getBase()->getStateSpace());
}

const SpaceInformationPtr &BundleSpace::getBundle() const
{
    return totalSpace_;
}

const SpaceInformationPtr &BundleSpace::getBase() const
{
    return baseSpace_;
}

unsigned int BundleSpace::getBaseDimension() const
{
    if (getBase())
        return getBase()->getStateDimension();
    else
        return 0;
}

unsigned int BundleSpace::getBundleDimension() const
{
    return getBundle()->getStateDimension();
}

unsigned int BundleSpace::getCoDimension() const
{
    return getBundleDimension() - getBaseDimension();
}

const StateSamplerPtr &BundleSpace::getBaseSamplerPtr() const
{
    if (hasBaseSpace())
    {
        return getChild()->getBundleSamplerPtr();
    }
    else
    {
        OMPL_ERROR("Cannot get Base Sampler without Base Space.");
        throw Exception("Tried Calling Non-existing base space sampler");
    }
}

const StateSamplerPtr &BundleSpace::getBundleSamplerPtr() const
{
    return Bundle_sampler_;
}

bool BundleSpace::isInfeasible()
{
    return false;
}

bool BundleSpace::hasConverged()
{
    return false;
}

bool BundleSpace::hasSolution()
{
    if (!hasSolution_)
    {
        PathPtr path;
        hasSolution_ = getSolution(path);
    }
    return hasSolution_;
}

BundleSpace *BundleSpace::getChild() const
{
    return childBundleSpace_;
}

void BundleSpace::setChild(BundleSpace *child)
{
    childBundleSpace_ = child;
}

BundleSpace *BundleSpace::getParent() const
{
    return parentBundleSpace_;
}

void BundleSpace::setParent(BundleSpace *parent)
{
    parentBundleSpace_ = parent;
}

unsigned int BundleSpace::getLevel() const
{
    return level_;
}

void BundleSpace::setLevel(unsigned int level)
{
    level_ = level;
}

OptimizationObjectivePtr BundleSpace::getOptimizationObjectivePtr() const
{
    return pdef_->getOptimizationObjective();
}

bool BundleSpace::sampleBundleValid(State *xRandom)
{
    bool found = false;

    unsigned int attempts = 0;
    do
    {
        sampleBundle(xRandom);
        found = getBundle()->getStateValidityChecker()->isValid(xRandom);
        attempts++;
    } while (attempts < magic::FIND_VALID_STATE_ATTEMPTS_WITHOUT_TERMINATION_CHECK && !found);
    return found;
}

void BundleSpace::sampleBundle(State *xRandom)
{
    if (!hasBaseSpace())
    {
        Bundle_sampler_->sampleUniform(xRandom);
    }
    else
    {
        if (getProjection()->getCoDimension() > 0)
        {
            // Adjusted sampling function: Sampling in G0 x Fiber
            getChild()->sampleFromDatastructure(xBaseTmp_);
            getProjection()->lift(xBaseTmp_, xRandom);
        }
        else
        {
            getChild()->sampleFromDatastructure(xRandom);
        }
    }
}

void BundleSpace::lift(const ompl::base::State *xBase, ompl::base::State *xBundle) const
{
    projection_->lift(xBase, xBundle);
}

void BundleSpace::project(const ompl::base::State *xBundle, ompl::base::State *xBase) const
{
    projection_->project(xBundle, xBase);
}

void BundleSpace::print(std::ostream &out) const
{
    out << getProjection();
}

namespace ompl
{
    namespace multilevel
    {
        std::ostream &operator<<(std::ostream &out, const BundleSpace &bundleSpace)
        {
            bundleSpace.print(out);
            return out;
        }
    }
}
