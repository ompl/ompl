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

#define DEBUG_BUNDLESPACE
#undef DEBUG_BUNDLESPACE

using namespace ompl::base;
using namespace ompl::multilevel;

unsigned int BundleSpace::counter_ = 0;

BundleSpace::BundleSpace(const SpaceInformationPtr &si, BundleSpace *baseBundleSpace_)
  : Planner(si, "BundleSpace"), Bundle(si), baseBundleSpace_(baseBundleSpace_)
{
    id_ = counter_++;

    //############################################################################
    // Check for dynamic spaces
    //############################################################################

    control::SpaceInformation *siC = dynamic_cast<control::SpaceInformation *>(getBundle().get());
    if (siC == nullptr)
    {
        isDynamic_ = false;
    }
    else
    {
        isDynamic_ = true;
    }
    OMPL_DEBUG("--- BundleSpace %d%s", id_, (isDynamic_ ? " (dynamic)" : ""));

    //############################################################################

    BundleSpaceComponentFactory componentFactory;

    if (!hasBaseSpace())
    {
        components_ = componentFactory.MakeBundleSpaceComponents(Bundle);
    }
    else
    {
        baseBundleSpace_->setTotalBundleSpace(this);
        Base = baseBundleSpace_->getBundle();

        components_ = componentFactory.MakeBundleSpaceComponents(Bundle, Base);

        MakeFiberSpace();
    }

    sanityChecks();

    std::stringstream ss;
    ss << (*this);
    OMPL_DEBUG(ss.str().c_str());

    if (!Bundle_valid_sampler_)
    {
        Bundle_valid_sampler_ = Bundle->allocValidStateSampler();
    }
    if (!Bundle_sampler_)
    {
        Bundle_sampler_ = Bundle->allocStateSampler();
    }
    if (hasBaseSpace())
    {
        xBaseTmp_ = getBase()->allocState();
        if (getFiberDimension() > 0)
            xFiberTmp_ = getFiber()->allocState();
    }
    xBundleTmp_ = getBundle()->allocState();
}

BundleSpace::~BundleSpace()
{
    if (hasBaseSpace())
    {
        if (xBaseTmp_)
        {
            Base->freeState(xBaseTmp_);
        }
        if (getFiberDimension() > 0 && xFiberTmp_)
        {
            Fiber->freeState(xFiberTmp_);
        }
    }
    if (xBundleTmp_)
    {
        Bundle->freeState(xBundleTmp_);
    }
    components_.clear();
}

bool BundleSpace::hasBaseSpace() const
{
    return !(baseBundleSpace_ == nullptr);
}

bool BundleSpace::hasTotalSpace() const
{
    return !(totalBundleSpace_ == nullptr);
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
        goal_ = pdef_->getGoal().get();

    if (getFiberDimension() > 0)
    {
        getFiber()->getStateSpace()->setup();
    }
}

void BundleSpace::clear()
{
    BaseT::clear();

    hasSolution_ = false;
    firstRun_ = true;
    if (!hasBaseSpace() && getFiberDimension() > 0)
        Fiber_sampler_.reset();

    pdef_->clearSolutionPaths();
}

void BundleSpace::MakeFiberSpace()
{
    StateSpacePtr Fiber_space = nullptr;
    if (components_.size() > 1)
    {
        Fiber_space = std::make_shared<CompoundStateSpace>();
        for (unsigned int m = 0; m < components_.size(); m++)
        {
            StateSpacePtr FiberM = components_.at(m)->getFiberSpace();
            double weight = (FiberM->getDimension() > 0 ? 1.0 : 0.0);
            std::static_pointer_cast<CompoundStateSpace>(Fiber_space)->addSubspace(FiberM, weight);
        }
    }
    else
    {
        Fiber_space = components_.front()->getFiberSpace();
    }

    if (Fiber_space != nullptr)
    {
        Fiber = std::make_shared<SpaceInformation>(Fiber_space);
        Fiber_sampler_ = Fiber->allocStateSampler();
    }
}

void BundleSpace::sanityChecks() const
{
    const StateSpacePtr Bundle_space = Bundle->getStateSpace();
    checkBundleSpaceMeasure("Bundle", Bundle_space);

    if (Base != nullptr)
    {
        const StateSpacePtr Base_space = Base->getStateSpace();
        checkBundleSpaceMeasure("Base", Base_space);
    }
    if (Fiber != nullptr)
    {
        const StateSpacePtr Fiber_space = Fiber->getStateSpace();
        checkBundleSpaceMeasure("Fiber", Fiber_space);
    }
    if (hasBaseSpace())
    {
        if ((getBaseDimension() + getFiberDimension() != getBundleDimension()))
        {
            OMPL_ERROR("Dimensions %d (Base) + %d (Fiber) != %d (Bundle)", getBaseDimension(), getFiberDimension(),
                       getBundleDimension());
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

    // if(space->getMeasure() <= 0)
    // {
    //     throw Exception("Space has zero measure.");
    // }
}

PlannerStatus BundleSpace::solve(const PlannerTerminationCondition &)
{
    throw Exception("A Bundle-Space cannot be solved alone. \
        Use class BundleSpaceSequence to solve Bundle-Spaces.");
}

void BundleSpace::setProblemDefinition(const ProblemDefinitionPtr &pdef)
{
    BaseT::setProblemDefinition(pdef);

    if (pdef_->hasOptimizationObjective())
    {
        opt_ = pdef_->getOptimizationObjective();
    }
    else
    {
        opt_ = std::make_shared<PathLengthOptimizationObjective>(getBundle());
        opt_->setCostThreshold(Cost(std::numeric_limits<double>::infinity()));
    }
}

void BundleSpace::resetCounter()
{
    BundleSpace::counter_ = 0;
}

// void BundleSpace::liftPath(
//       const std::vector<State*> pathBase,
//       const State* xFiberStart,
//       const State* xFiberGoal,
//       std::vector<State*> &pathBundle) const
// {
//     if(pathBase.size() != pathBundle.size())
//     {
//       OMPL_ERROR("Size of paths has to be identical in order to lift them.");
//       throw Exception("Path size inequivalent.");
//     }
//     if(getFiberDimension() > 0)
//     {
//         double lengthTotalPathBase = 0;
//         std::vector<double> lengthIntermediatePathBase;

//         for(unsigned int k = 1; k < pathBase.size(); k++){
//             double lengthKthSegment = getBase()->distance(pathBase.at(k-1), pathBase.at(k));
//             lengthIntermediatePathBase.push_back(lengthKthSegment);
//             lengthTotalPathBase += lengthKthSegment;
//         }

//         double lengthCurrent = 0;

//         State *xFiberCur = getFiber()->allocState();

//         for(unsigned int k = 0; k < pathBase.size(); k++)
//         {
//             double step = lengthCurrent / lengthTotalPathBase;
//             getFiber()->getStateSpace()->interpolate(xFiberStart, xFiberGoal, step, xFiberCur);

//             liftState(pathBase.at(k), xFiberCur, pathBundle.at(k));

//             if(k < pathBase.size() - 1)
//             {
//                 lengthCurrent += lengthIntermediatePathBase.at(k);
//             }
//         }
//         getFiber()->freeState(xFiberCur);

//     }else{
//         for(unsigned int k = 0; k < pathBase.size(); k++){
//             getBundle()->copyState(pathBundle.at(k), pathBase.at(k));
//         }
//     }
// }

// unsigned int BundleSpace::interpolateAlongBasePath(const std::vector<State *> basePath, double location,
//                                                   State *xResult) const
//{
//    double d_path = 0;
//    for (unsigned int k = 0; k < basePath.size() - 1; k++)
//    {
//        d_path += getBase()->distance(basePath.at(k), basePath.at(k + 1));
//    }

//    assert(location >= 0);
//    assert(location <= d_path);

//    double d_last_to_next = 0;

//    unsigned int ctr = 0;
//    double d = 0;
//    while (d <= location && ctr < basePath.size() - 1)
//    {
//        d_last_to_next = getBase()->distance(basePath.at(ctr), basePath.at(ctr + 1));
//        d += d_last_to_next;
//        ctr++;
//    }

//    State *xLast = basePath.at(ctr - 1);
//    State *xNext = basePath.at(ctr);

//    //|--------------------- d ----------------------------|
//    //|----------------- location -------------|
//    //                                |-- d_last_to_next --|
//    //                                |-step-|
//    //
//    //                              xLast                 xNext (ctr)

//    double step = 0.0;
//    if (d_last_to_next > 0)
//    {
//        step = std::fabs(d_last_to_next - (d - location)) / d_last_to_next;
//    }

//    getBase()->getStateSpace()->interpolate(xLast, xNext, step, xResult);

//    if ((std::isnan(step)) || (step < 0) || (step > 1))
//    {
//        std::cout << std::string(80, '#') << std::endl;
//        for (unsigned int k = 0; k < basePath.size(); k++)
//        {
//            getBase()->printState(basePath.at(k));
//        }
//        std::cout << std::string(80, '-') << std::endl;
//        getBase()->printState(xLast);
//        std::cout << "position:" << d - d_last_to_next << std::endl;
//        getBase()->printState(xNext);
//        std::cout << "position:" << d << std::endl;
//        std::cout << "location:" << location << std::endl;
//        std::cout << "d_last_to_next:" << d_last_to_next << std::endl;
//        std::cout << "step: " << step << std::endl;
//        std::cout << "step (not normalized): " << (d_last_to_next - (d - location)) << std::endl;
//        getBase()->printState(xResult);
//    }
//    return ctr;
//}

void BundleSpace::liftState(const State *xBase, const State *xFiber, State *xBundle) const
{
    unsigned int M = components_.size();

    if (M > 1)
    {
        for (unsigned int m = 0; m < M; m++)
        {
            const State *xmBase = xBase->as<CompoundState>()->as<State>(m);
            const State *xmFiber = xFiber->as<CompoundState>()->as<State>(m);
            State *xmBundle = xBundle->as<CompoundState>()->as<State>(m);
            components_.at(m)->liftState(xmBase, xmFiber, xmBundle);
        }
    }
    else
    {
        components_.front()->liftState(xBase, xFiber, xBundle);
    }

#ifdef DEBUG_BUNDLESPACE
    OMPL_WARN("Debugging ON");
    State *xF = getFiber()->allocState();
    projectFiber(xBundle, xF);

    if (getFiber()->distance(xFiber, xF) > 1e-5)
    {
        std::cout << std::string(80, '-') << std::endl;
        getFiber()->printState(xFiber);
        std::cout << std::string(80, '-') << std::endl;
        getFiber()->printState(xF);
        OMPL_ERROR("Fibers are not preserved after lifting.");
        throw base::Exception("NotPreserved");
    }
    getFiber()->freeState(xF);
#endif
}

void BundleSpace::projectFiber(const State *xBundle, State *xFiber) const
{
    unsigned int M = components_.size();

    if (M > 1)
    {
        for (unsigned int m = 0; m < M; m++)
        {
            if (components_.at(m)->getFiberDimension() > 0)
            {
                const State *xmBundle = xBundle->as<CompoundState>()->as<State>(m);
                State *xmFiber = xFiber->as<CompoundState>()->as<State>(m);
                components_.at(m)->projectFiber(xmBundle, xmFiber);
            }
        }
    }
    else
    {
        components_.front()->projectFiber(xBundle, xFiber);
    }
}

void BundleSpace::projectBase(const State *xBundle, State *xBase) const
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
                components_.at(m)->projectBase(xmBundle, xmBase);
            }
        }
    }
    else
    {
        components_.front()->projectBase(xBundle, xBase);
    }
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

State *BundleSpace::allocIdentityStateFiber() const
{
    return allocIdentityState(getFiber()->getStateSpace());
}

State *BundleSpace::allocIdentityStateBundle() const
{
    return allocIdentityState(getBundle()->getStateSpace());
}

State *BundleSpace::allocIdentityStateBase() const
{
    return allocIdentityState(getBase()->getStateSpace());
}

const SpaceInformationPtr &BundleSpace::getFiber() const
{
    return Fiber;
}

const SpaceInformationPtr &BundleSpace::getBundle() const
{
    return Bundle;
}

const SpaceInformationPtr &BundleSpace::getBase() const
{
    return Base;
}

unsigned int BundleSpace::getFiberDimension() const
{
    if (getFiber())
        return getFiber()->getStateDimension();
    else
        return 0;
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

const StateSamplerPtr &BundleSpace::getFiberSamplerPtr() const
{
    return Fiber_sampler_;
}

const StateSamplerPtr &BundleSpace::getBaseSamplerPtr() const
{
    if (hasBaseSpace())
    {
        return getBaseBundleSpace()->getBundleSamplerPtr();
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

bool BundleSpace::hasSolution()
{
    if (!hasSolution_)
    {
        PathPtr path;
        hasSolution_ = getSolution(path);
    }
    return hasSolution_;
}

BundleSpace *BundleSpace::getBaseBundleSpace() const
{
    return baseBundleSpace_;
}

void BundleSpace::setBaseBundleSpace(BundleSpace *baseBundleSpace)
{
    baseBundleSpace_ = baseBundleSpace;
}

BundleSpace *BundleSpace::getTotalBundleSpace() const
{
    return totalBundleSpace_;
}

void BundleSpace::setTotalBundleSpace(BundleSpace *totalBundleSpace)
{
    totalBundleSpace_ = totalBundleSpace;
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
    return opt_;
}

void BundleSpace::sampleFiber(State *xFiber)
{
    Fiber_sampler_->sampleUniform(xFiber);
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
        if (getFiberDimension() > 0)
        {
            // Adjusted sampling function: Sampling in G0 x Fiber
            baseBundleSpace_->sampleFromDatastructure(xBaseTmp_);
            sampleFiber(xFiberTmp_);
            liftState(xBaseTmp_, xFiberTmp_, xRandom);
        }
        else
        {
            baseBundleSpace_->sampleFromDatastructure(xRandom);
        }
    }
}

void BundleSpace::debugInvalidState(const State *x)
{
    const StateSpacePtr space = Bundle->getStateSpace();
    bool bounds = space->satisfiesBounds(x);
    if (!bounds)
    {
        std::vector<StateSpacePtr> Bundle_decomposed;
        if (!space->isCompound())
        {
            Bundle_decomposed.push_back(space);
        }
        else
        {
            CompoundStateSpace *Bundle_compound = space->as<CompoundStateSpace>();
            Bundle_decomposed = Bundle_compound->getSubspaces();
        }

        for (unsigned int m = 0; m < Bundle_decomposed.size(); m++)
        {
            StateSpacePtr spacek = Bundle_decomposed.at(m);
            int type = spacek->getType();
            switch (type)
            {
                case STATE_SPACE_REAL_VECTOR:
                {
                    auto *RN = spacek->as<RealVectorStateSpace>();
                    const RealVectorStateSpace::StateType *xk =
                        x->as<CompoundState>()->as<RealVectorStateSpace::StateType>(m);
                    std::vector<double> bl = RN->getBounds().low;
                    std::vector<double> bh = RN->getBounds().high;
                    for (unsigned int k = 0; k < bl.size(); k++)
                    {
                        double qk = xk->values[k];
                        double qkl = bl.at(k);
                        double qkh = bh.at(k);
                        if (qk < qkl || qk > qkh)
                        {
                            OMPL_ERROR("Out Of Bounds [component %d, link %d] %.2f <= %.2f <= %.2f", m, k, bl.at(k), qk,
                                       bh.at(k));
                        }
                    }
                    break;
                }
                case STATE_SPACE_SO2:
                {
                    double value = 0;
                    if (!space->isCompound())
                    {
                        const SO2StateSpace::StateType *xk = x->as<CompoundState>()->as<SO2StateSpace::StateType>(m);
                        value = xk->value;
                    }
                    else
                    {
                        const SO2StateSpace::StateType *xk = x->as<SO2StateSpace::StateType>();
                        value = xk->value;
                    }
                    OMPL_ERROR("Invalid: -%.2f <= %f <= +%.2f", 3.14, value, 3.14);
                    break;
                }
                default:
                {
                    OMPL_ERROR("Could not debug state type %d.", type);
                    break;
                }
            }
        }
    }
    else
    {
        std::cout << "Bounds satisfied. Must be collision problem." << std::endl;
    }
}

void BundleSpace::print(std::ostream &out) const
{
    unsigned int M = components_.size();
    out << "[";
    for (unsigned int m = 0; m < M; m++)
    {
        out << components_.at(m)->getTypeAsString() << (isDynamic_ ? "(dyn)" : "") << (m < M - 1 ? " | " : "");
    }
    out << "]";
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
