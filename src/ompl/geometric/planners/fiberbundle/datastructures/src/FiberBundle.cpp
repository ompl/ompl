/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, University of Stuttgart
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
 *   * Neither the name of the University of Stuttgart nor the names
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
#include <ompl/geometric/planners/fiberbundle/datastructures/FiberBundle.h>

#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/SO3StateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>

#include <ompl/util/Exception.h>

unsigned int ompl::geometric::FiberBundle::counter_ = 0;

ompl::geometric::FiberBundle::FiberBundle(const base::SpaceInformationPtr &si, FiberBundle *parent_)
  : base::Planner(si, "FiberBundle"), totalSpace(si), parent_(parent_)
{
    id_ = counter_++;

    //############################################################################
    //Check for dynamic spaces
    //############################################################################
    ompl::control::SpaceInformation *siC = dynamic_cast<ompl::control::SpaceInformation*>(si_.get());
    if(siC==nullptr) {
      isDynamic_ = false;
    }else{
      isDynamic_ = true;
    }
    OMPL_DEVMSG1("FiberBundle %d%s", id_, (isDynamic_?" (dynamic)":""));

    //############################################################################
    const base::StateSpacePtr totalSpace_space = totalSpace->getStateSpace();


    if (!hasParent())//parent_ == nullptr)
    {
        OMPL_DEVMSG1("ATOMIC_RN dimension: %d measure: %f", totalSpace_space->getDimension(), totalSpace_space->getMeasure());
        type_ = ATOMIC_RN;
    }
    else
    {
        parent_->setChild(this);

        baseSpace = parent_->getSpaceInformation();
        const base::StateSpacePtr baseSpace_space = baseSpace->getStateSpace();
        // fiber = totalSpace / baseSpace
        const base::StateSpacePtr fiber_space = computeFiberSpace(totalSpace_space, baseSpace_space);

        if (fiber_space != nullptr)
        {
            fiber = std::make_shared<base::SpaceInformation>(fiber_space);
            fiber_sampler_ = fiber->allocStateSampler();
            if (baseSpace_space->getDimension() + fiber_space->getDimension() != totalSpace_space->getDimension())
            {
                throw ompl::Exception("FiberBundle Dimensions are wrong.");
            }
            OMPL_DEVMSG1("baseSpace dimension: %d measure: %f", baseSpace_space->getDimension(), baseSpace_space->getMeasure());
            OMPL_DEVMSG1("fiber dimension: %d measure: %f", fiber_space->getDimension(), fiber_space->getMeasure());
            OMPL_DEVMSG1("totalSpace dimension: %d measure: %f", totalSpace_space->getDimension(), totalSpace_space->getMeasure());
            if ((baseSpace_space->getMeasure() <= 0) || (fiber_space->getMeasure() <= 0) || (totalSpace_space->getMeasure() <= 0))
            {
                throw ompl::Exception("Zero-measure FiberBundle detected.");
            }
            if (!fiber_sampler_)
            {
                fiber_sampler_ = fiber->allocStateSampler();
            }
            checkSpaceHasFiniteMeasure(fiber_space);
        }
        else
        {
            OMPL_DEVMSG1("baseSpace dimension: %d measure: %f", baseSpace_space->getDimension(), baseSpace_space->getMeasure());
            OMPL_DEVMSG1("totalSpace dimension: %d measure: %f", totalSpace_space->getDimension(), totalSpace_space->getMeasure());
        }
        checkSpaceHasFiniteMeasure(baseSpace_space);
    }
    checkSpaceHasFiniteMeasure(totalSpace_space);

    if (!totalSpace_valid_sampler_)
    {
        totalSpace_valid_sampler_ = totalSpace->allocValidStateSampler();
    }
    if (!totalSpace_sampler_)
    {
        totalSpace_sampler_ = totalSpace->allocStateSampler();
    }
    if (hasParent())//parent_ != nullptr)
    {
        s_baseSpace_tmp_ = baseSpace->allocState();
        if (fiber_dimension_ > 0)
            s_fiber_tmp_ = fiber->allocState();
    }
}

ompl::geometric::FiberBundle::~FiberBundle()
{
    if (hasParent())//parent_ != nullptr)
    {
        if (s_baseSpace_tmp_)
            baseSpace->freeState(s_baseSpace_tmp_);
        if (fiber && s_fiber_tmp_)
            fiber->freeState(s_fiber_tmp_);
    }
}

bool ompl::geometric::FiberBundle::hasParent() const
{
    return !(parent_ == nullptr);
}

bool ompl::geometric::FiberBundle::hasChild() const
{
    return !(child_ == nullptr);
}

bool ompl::geometric::FiberBundle::isDynamic() const
{
    return isDynamic_;
}

void ompl::geometric::FiberBundle::setup()
{
    BaseT::setup();
    hasSolution_ = false;
    firstRun_ = true;
    if(pdef_) goal_ = pdef_->getGoal().get();
}

void ompl::geometric::FiberBundle::clear()
{
    BaseT::clear();
    totalNumberOfSamples_ = 0;
    totalNumberOfFeasibleSamples_ = 0;

    hasSolution_ = false;
    firstRun_ = true;
    if (!hasParent() && fiber_dimension_ > 0)
        fiber_sampler_.reset();

    pdef_->clearSolutionPaths();
}

void ompl::geometric::FiberBundle::checkSpaceHasFiniteMeasure(const base::StateSpacePtr space) const
{
    if (space->getMeasure() >= std::numeric_limits<double>::infinity())
    {
        const base::StateSpacePtr baseSpace_space = baseSpace->getStateSpace();
        const base::StateSpacePtr totalSpace_space = totalSpace->getStateSpace();
        OMPL_ERROR("baseSpace dimension: %d measure: %f", baseSpace_space->getDimension(), baseSpace_space->getMeasure());
        OMPL_ERROR("totalSpace dimension: %d measure: %f", totalSpace_space->getDimension(), totalSpace_space->getMeasure());
        if (fiber != nullptr)
        {
            const base::StateSpacePtr fiber_space = fiber->getStateSpace();
            OMPL_ERROR("fiber dimension: %d measure: %f", fiber_space->getDimension(), fiber_space->getMeasure());
        }
        throw ompl::Exception("FiberBundle has no bounds");
    }
}

ompl::base::PlannerStatus ompl::geometric::FiberBundle::solve(const base::PlannerTerminationCondition &ptc)
{
    (void)ptc;
    throw ompl::Exception("A Fiber Bundle cannot be solved alone. Use class FiberBundleManager to solve Fiber Bundles.");
}

void ompl::geometric::FiberBundle::setProblemDefinition(const base::ProblemDefinitionPtr &pdef)
{
    BaseT::setProblemDefinition(pdef);

    if (pdef_->hasOptimizationObjective())
    {
        opt_ = pdef_->getOptimizationObjective();
    }
    else
    {
        opt_ = std::make_shared<base::PathLengthOptimizationObjective>(si_);
    }
}

void ompl::geometric::FiberBundle::resetCounter()
{
    FiberBundle::counter_ = 0;
}

const ompl::base::StateSpacePtr ompl::geometric::FiberBundle::computeFiberSpace(const base::StateSpacePtr totalSpace,
                                                                                     const base::StateSpacePtr baseSpace)
{
    type_ = identifyFiberSpaceType(totalSpace, baseSpace);

    base::StateSpacePtr fiber{nullptr};
    totalSpace_dimension_ = totalSpace->getDimension();
    baseSpace_dimension_ = baseSpace->getDimension();

    if (baseSpace_dimension_ == 0 || totalSpace_dimension_ == 0)
    {
        OMPL_ERROR("baseSpace has dimension %d.", baseSpace_dimension_);
        OMPL_ERROR("totalSpace has dimension %d.", totalSpace_dimension_);
        throw ompl::Exception("Detected Zero-dimensional FiberBundle.");
    }

    switch (type_)
    {
        case IDENTITY_SPACE_RN:
        case IDENTITY_SPACE_SE2:
        case IDENTITY_SPACE_SE2RN:
        case IDENTITY_SPACE_SO2RN:
        case IDENTITY_SPACE_SE3:
        case IDENTITY_SPACE_SE3RN:
        {
            fiber_dimension_ = 0;
            break;
        }
        case RN_RM:
        {
            unsigned int N = totalSpace_dimension_ - baseSpace_dimension_;
            fiber = std::make_shared<base::RealVectorStateSpace>(N);
            fiber_dimension_ = N;

            base::RealVectorBounds totalSpace_bounds = std::static_pointer_cast<base::RealVectorStateSpace>(totalSpace)->getBounds();
            std::vector<double> low;
            low.resize(N);
            std::vector<double> high;
            high.resize(N);
            base::RealVectorBounds fiber_bounds(N);
            for (unsigned int k = 0; k < N; k++)
            {
                fiber_bounds.setLow(k, totalSpace_bounds.low.at(k + baseSpace_dimension_));
                fiber_bounds.setHigh(k, totalSpace_bounds.high.at(k + baseSpace_dimension_));
            }
            std::static_pointer_cast<base::RealVectorStateSpace>(fiber)->setBounds(fiber_bounds);

            break;
        }
        case SE2_R2:
        {
            fiber_dimension_ = 1;
            fiber = std::make_shared<base::SO2StateSpace>();
            break;
        }
        case SE3_R3:
        {
            fiber_dimension_ = 3;
            fiber = std::make_shared<base::SO3StateSpace>();
            break;
        }
        case SE2RN_SE2:
        case SE3RN_SE3:
        case SO2RN_SO2:
        {
            base::CompoundStateSpace *totalSpace_compound = totalSpace->as<base::CompoundStateSpace>();
            const std::vector<base::StateSpacePtr> totalSpace_decomposed = totalSpace_compound->getSubspaces();

            fiber_dimension_ = totalSpace_decomposed.at(1)->getDimension();

            fiber = std::make_shared<base::RealVectorStateSpace>(fiber_dimension_);
            std::static_pointer_cast<base::RealVectorStateSpace>(fiber)->setBounds(
                std::static_pointer_cast<base::RealVectorStateSpace>(totalSpace_decomposed.at(1))->getBounds());

            break;
        }
        case SE2RN_R2:
        {
            base::CompoundStateSpace *totalSpace_compound = totalSpace->as<base::CompoundStateSpace>();
            const std::vector<base::StateSpacePtr> totalSpace_decomposed = totalSpace_compound->getSubspaces();
            const std::vector<base::StateSpacePtr> totalSpace_SE2_decomposed =
                totalSpace_decomposed.at(0)->as<base::CompoundStateSpace>()->getSubspaces();

            const base::RealVectorStateSpace *totalSpace_RN = totalSpace_decomposed.at(1)->as<base::RealVectorStateSpace>();
            unsigned int N = totalSpace_RN->getDimension();

            base::StateSpacePtr SO2(new base::SO2StateSpace());
            base::StateSpacePtr RN(new base::RealVectorStateSpace(N));
            RN->as<base::RealVectorStateSpace>()->setBounds(totalSpace_RN->getBounds());

            fiber = SO2 + RN;
            fiber_dimension_ = 1 + N;
            break;
        }
        case SE3RN_R3:
        {
            base::CompoundStateSpace *totalSpace_compound = totalSpace->as<base::CompoundStateSpace>();
            const std::vector<base::StateSpacePtr> totalSpace_decomposed = totalSpace_compound->getSubspaces();
            const std::vector<base::StateSpacePtr> totalSpace_SE3_decomposed =
                totalSpace_decomposed.at(0)->as<base::CompoundStateSpace>()->getSubspaces();

            // const base::SE3StateSpace *totalSpace_SE3 = totalSpace_SE3_decomposed.at(0)->as<base::SE3StateSpace>();
            // const base::SO3StateSpace *totalSpace_SO3 = totalSpace_SE3_decomposed.at(1)->as<base::SO3StateSpace>();
            const base::RealVectorStateSpace *totalSpace_RN = totalSpace_decomposed.at(1)->as<base::RealVectorStateSpace>();
            unsigned int N = totalSpace_RN->getDimension();

            base::StateSpacePtr SO3(new base::SO3StateSpace());
            base::StateSpacePtr RN(new base::RealVectorStateSpace(N));
            RN->as<base::RealVectorStateSpace>()->setBounds(totalSpace_RN->getBounds());

            fiber = SO3 + RN;
            fiber_dimension_ = 3 + N;
            break;
        }
        case SE2RN_SE2RM:
        case SO2RN_SO2RM:
        case SE3RN_SE3RM:
        {
            base::CompoundStateSpace *totalSpace_compound = totalSpace->as<base::CompoundStateSpace>();
            const std::vector<base::StateSpacePtr> totalSpace_decomposed = totalSpace_compound->getSubspaces();
            base::CompoundStateSpace *baseSpace_compound = baseSpace->as<base::CompoundStateSpace>();
            const std::vector<base::StateSpacePtr> baseSpace_decomposed = baseSpace_compound->getSubspaces();

            unsigned int N = totalSpace_decomposed.at(1)->getDimension();
            unsigned int M = baseSpace_decomposed.at(1)->getDimension();
            fiber_dimension_ = N - M;
            fiber = std::make_shared<base::RealVectorStateSpace>(fiber_dimension_);

            base::RealVectorBounds totalSpace_bounds =
                std::static_pointer_cast<base::RealVectorStateSpace>(totalSpace_decomposed.at(1))->getBounds();
            std::vector<double> low;
            low.resize(fiber_dimension_);
            std::vector<double> high;
            high.resize(fiber_dimension_);
            base::RealVectorBounds fiber_bounds(fiber_dimension_);
            for (unsigned int k = 0; k < fiber_dimension_; k++)
            {
                fiber_bounds.setLow(k, totalSpace_bounds.low.at(k + M));
                fiber_bounds.setHigh(k, totalSpace_bounds.high.at(k + M));
            }
            std::static_pointer_cast<base::RealVectorStateSpace>(fiber)->setBounds(fiber_bounds);
            break;
        }
        default:
        {
            OMPL_ERROR("Unknown Fiber type: %d", type_);
            throw ompl::Exception("Unknown type");
        }
    }
    return fiber;
}

ompl::geometric::FiberBundle::FiberBundleType
ompl::geometric::FiberBundle::identifyFiberSpaceType(const base::StateSpacePtr totalSpace, const base::StateSpacePtr baseSpace)
{
    //
    // We can currently handle 11 types of fiber bundle mappings.
    // Emptyset is used for constraint relaxations.
    //
    //   (1)  totalSpace Rn     , baseSpace Rm     [0<m<=n]  => fiber = R(n-m) \union {\emptyset}
    //   (2a) totalSpace SE2    , baseSpace R2               => fiber = SO2
    //   (2b) totalSpace SE2    , baseSpace SE2              => fiber = \emptyset
    //   (3a) totalSpace SE3    , baseSpace R3               => fiber = SO3
    //   (3b) totalSpace SE3    , baseSpace SE3              => fiber = \emptyset
    //
    //   (4)  totalSpace SE3xRn , baseSpace SE3              => fiber = Rn
    //   (5)  totalSpace SE3xRn , baseSpace R3               => fiber = SO3xRn
    //   (6)  totalSpace SE3xRn , baseSpace SE3xRm [0<m<=n ] => fiber = R(n-m) \union {\emptyset}
    //
    //   (7)  totalSpace SE2xRn , baseSpace SE2              => fiber = Rn
    //   (8)  totalSpace SE2xRn , baseSpace R2               => fiber = SO2xRN
    //   (9)  totalSpace SE2xRn , baseSpace SE2xRm [0<m<=n ] => fiber = R(n-m) \union {\emptyset}
    //
    //  (10)  totalSpace SO2xRn , baseSpace SO2              => fiber = Rn
    //  (11)  totalSpace SO2xRn , baseSpace SO2xRm [0<m<=n ] => fiber = R(n-m) \union {\emptyset}

    if (!totalSpace->isCompound())
    {
        ///##############################################################################/
        //------------------ non-compound cases:
        ///##############################################################################/
        //
        //------------------ (1) totalSpace = Rn, baseSpace = Rm, 0<m<n, fiber = R(n-m)
        if (totalSpace->getType() == base::STATE_SPACE_REAL_VECTOR)
        {
            unsigned int n = totalSpace->getDimension();
            if (baseSpace->getType() == base::STATE_SPACE_REAL_VECTOR)
            {
                unsigned int m = baseSpace->getDimension();
                if (n > m && m > 0)
                {
                    type_ = RN_RM;
                }
                else
                {
                    if (n == m && m > 0)
                    {
                        type_ = IDENTITY_SPACE_RN;
                    }
                    else
                    {
                        OMPL_ERROR("Not allowed: dimensionality needs to be monotonically increasing.");
                        OMPL_ERROR("We require n >= m > 0 but have n=%d >= m=%d > 0", n, m);
                        throw ompl::Exception("Invalid dimensionality");
                    }
                }
            }
            else
            {
                OMPL_ERROR("totalSpace is R^%d but baseSpace type %d is not handled.", n, baseSpace->getType());
                throw ompl::Exception("INVALID_STATE_TYPE");
            }
        }
        else
        {
            OMPL_ERROR("totalSpace is non-compound state, but its type %d is not handled.", totalSpace->getType());
            throw ompl::Exception("INVALID_STATE_TYPE");
        }
    }
    else
    {
        ///##############################################################################/
        //------------------ compound cases:
        ///##############################################################################/
        //
        //------------------ (2) totalSpace = SE2, baseSpace = R2, fiber = SO2
        ///##############################################################################/
        if (totalSpace->getType() == base::STATE_SPACE_SE2)
        {
            if (baseSpace->getType() == base::STATE_SPACE_REAL_VECTOR)
            {
                if (baseSpace->getDimension() == 2)
                {
                    type_ = SE2_R2;
                }
                else
                {
                    OMPL_ERROR("totalSpace is SE2 but baseSpace type %d is of dimension %d", baseSpace->getType(), baseSpace->getDimension());
                    throw ompl::Exception("Invalid dimensions.");
                }
            }
            else
            {
                if (baseSpace->getType() == base::STATE_SPACE_SE2)
                {
                    type_ = IDENTITY_SPACE_SE2;
                }
                else
                {
                    OMPL_ERROR("totalSpace is SE2 but baseSpace type %d is not handled.", baseSpace->getType());
                    throw ompl::Exception("INVALID_STATE_TYPE");
                }
            }
        }
        //------------------ (3) totalSpace = SE3, baseSpace = R3, fiber = SO3
        ///##############################################################################/
        else if (totalSpace->getType() == base::STATE_SPACE_SE3)
        {
            if (baseSpace->getType() == base::STATE_SPACE_REAL_VECTOR)
            {
                if (baseSpace->getDimension() == 3)
                {
                    type_ = SE3_R3;
                }
                else
                {
                    OMPL_ERROR("totalSpace is SE3 but baseSpace type %d is of dimension %d.", baseSpace->getType(), baseSpace->getDimension());
                    throw ompl::Exception("Invalid dimensions.");
                }
            }
            else
            {
                if (baseSpace->getType() == base::STATE_SPACE_SE3)
                {
                    type_ = IDENTITY_SPACE_SE3;
                }
                else
                {
                    OMPL_ERROR("totalSpace is SE2 but baseSpace type %d is not handled.", baseSpace->getType());
                    throw ompl::Exception("Invalid Fiber type");
                }
                OMPL_ERROR("totalSpace is SE3 but baseSpace type %d is not handled.", baseSpace->getType());
                throw ompl::Exception("Invalid Fiber type");
            }
        }
        ///##############################################################################/
        else
        {
            base::CompoundStateSpace *totalSpace_compound = totalSpace->as<base::CompoundStateSpace>();
            const std::vector<base::StateSpacePtr> totalSpace_decomposed = totalSpace_compound->getSubspaces();
            unsigned int totalSpace_subspaces = totalSpace_decomposed.size();
            if (totalSpace_subspaces == 2)
            {
                if (totalSpace_decomposed.at(0)->getType() == base::STATE_SPACE_SE3 &&
                    totalSpace_decomposed.at(1)->getType() == base::STATE_SPACE_REAL_VECTOR)
                {
                    unsigned int n = totalSpace_decomposed.at(1)->getDimension();
                    if (baseSpace->getType() == base::STATE_SPACE_SE3)
                    {
                        //------------------ (4) totalSpace = SE3xRn, baseSpace = SE3, fiber = Rn
                        ///##############################################################################/
                        type_ = SE3RN_SE3;
                    }
                    else if (baseSpace->getType() == base::STATE_SPACE_REAL_VECTOR)
                    {
                        //------------------ (5) totalSpace = SE3xRn, baseSpace = R3, fiber = SO3xRN
                        ///##############################################################################/
                        unsigned int m = baseSpace->getDimension();
                        if (m == 3)
                        {
                            type_ = SE3RN_R3;
                        }
                        else
                        {
                            OMPL_ERROR("Not allowed. baseSpace needs to be 3-dimensional but is %d dimensional", m);
                            throw ompl::Exception("Invalid dimensions.");
                        }
                    }
                    else
                    {
                        //------------------ (6) totalSpace = SE3xRn, baseSpace = SE3xRm, fiber = R(n-m)
                        ///##############################################################################/
                        base::CompoundStateSpace *baseSpace_compound = baseSpace->as<base::CompoundStateSpace>();
                        const std::vector<base::StateSpacePtr> baseSpace_decomposed = baseSpace_compound->getSubspaces();
                        unsigned int baseSpace_subspaces = baseSpace_decomposed.size();
                        if (baseSpace_subspaces == 2)
                        {
                            if (totalSpace_decomposed.at(0)->getType() == base::STATE_SPACE_SE3 &&
                                totalSpace_decomposed.at(1)->getType() == base::STATE_SPACE_REAL_VECTOR)
                            {
                                unsigned int m = baseSpace_decomposed.at(1)->getDimension();
                                if (m < n && m > 0)
                                {
                                    type_ = SE3RN_SE3RM;
                                }
                                else
                                {
                                    if (m == n)
                                    {
                                        type_ = IDENTITY_SPACE_SE3RN;
                                    }
                                    else
                                    {
                                        OMPL_ERROR("We require n >= m > 0, but have n=%d >= m=%d > 0.", n, m);
                                        throw ompl::Exception("Invalid dimensions.");
                                    }
                                }
                            }
                        }
                        else
                        {
                            OMPL_ERROR("State compound with %d subspaces not handled.", baseSpace_subspaces);
                            throw ompl::Exception("Invalid Fiber type");
                        }
                    }
                }
                else
                {
                    if (totalSpace_decomposed.at(0)->getType() == base::STATE_SPACE_SE2 &&
                        totalSpace_decomposed.at(1)->getType() == base::STATE_SPACE_REAL_VECTOR)
                    {
                        unsigned int n = totalSpace_decomposed.at(1)->getDimension();
                        if (baseSpace->getType() == base::STATE_SPACE_SE2)
                        {
                            //------------------ (7) totalSpace = SE2xRn, baseSpace = SE2, fiber = Rn
                            ///##############################################################################/
                            type_ = SE2RN_SE2;
                        }
                        else if (baseSpace->getType() == base::STATE_SPACE_REAL_VECTOR)
                        {
                            //------------------ (8) totalSpace = SE2xRn, baseSpace = R2, fiber = SO2xRN
                            ///##############################################################################/
                            unsigned int m = baseSpace->getDimension();
                            if (m == 2)
                            {
                                type_ = SE2RN_R2;
                            }
                            else
                            {
                                OMPL_ERROR("Not allowed. baseSpace needs to be 2-dimensional but is %d dimensional", m);
                                throw ompl::Exception("Invalid dimensions.");
                            }
                        }
                        else
                        {
                            //------------------ (9) totalSpace = SE2xRn, baseSpace = SE2xRm, fiber = R(n-m)
                            ///##############################################################################/
                            base::CompoundStateSpace *baseSpace_compound = baseSpace->as<base::CompoundStateSpace>();
                            const std::vector<base::StateSpacePtr> baseSpace_decomposed = baseSpace_compound->getSubspaces();
                            unsigned int baseSpace_subspaces = baseSpace_decomposed.size();
                            if (baseSpace_subspaces == 2)
                            {
                                if (totalSpace_decomposed.at(0)->getType() == base::STATE_SPACE_SE2 &&
                                    totalSpace_decomposed.at(1)->getType() == base::STATE_SPACE_REAL_VECTOR)
                                {
                                    unsigned int m = baseSpace_decomposed.at(1)->getDimension();
                                    if (m < n && m > 0)
                                    {
                                        type_ = SE2RN_SE2RM;
                                    }
                                    else
                                    {
                                        if (m == n)
                                        {
                                            type_ = IDENTITY_SPACE_SE2RN;
                                        }
                                        else
                                        {
                                            OMPL_ERROR("We require n >= m > 0, but have n=%d >= m=%d > 0.", n, m);
                                            throw ompl::Exception("Invalid dimensions.");
                                        }
                                    }
                                }
                                else
                                {
                                }
                            }
                            else
                            {
                                OMPL_ERROR("QO is compound with %d subspaces, but we only handle 2.", baseSpace_subspaces);
                                throw ompl::Exception("Invalid Fiber type");
                            }
                        }
                    }
                    else if (totalSpace_decomposed.at(0)->getType() == base::STATE_SPACE_SO2 &&
                             totalSpace_decomposed.at(1)->getType() == base::STATE_SPACE_REAL_VECTOR)
                    {
                        if (baseSpace->getType() == base::STATE_SPACE_SO2)
                        {
                            //------------------ (10) totalSpace = SO2xRn, baseSpace = SO2, fiber = Rn
                            ///##############################################################################/
                            type_ = SO2RN_SO2;
                        }
                        else
                        {
                            //------------------ (11) totalSpace = SO2xRn, baseSpace = SO2xRm, fiber = R(n-m)
                            ///##############################################################################/
                            if (baseSpace->isCompound())
                            {
                                base::CompoundStateSpace *baseSpace_compound = baseSpace->as<base::CompoundStateSpace>();
                                const std::vector<base::StateSpacePtr> baseSpace_decomposed = baseSpace_compound->getSubspaces();
                                unsigned int baseSpace_subspaces = baseSpace_decomposed.size();
                                if (baseSpace_subspaces == 2)
                                {
                                    if (totalSpace_decomposed.at(0)->getType() == base::STATE_SPACE_SO2 &&
                                        totalSpace_decomposed.at(1)->getType() == base::STATE_SPACE_REAL_VECTOR)
                                    {
                                        unsigned int n = totalSpace_decomposed.at(1)->getDimension();
                                        unsigned int m = baseSpace_decomposed.at(1)->getDimension();
                                        if (m < n && m > 0)
                                        {
                                            type_ = SO2RN_SO2RM;
                                        }
                                        else
                                        {
                                            if (m == n)
                                            {
                                                type_ = IDENTITY_SPACE_SO2RN;
                                            }
                                            else
                                            {
                                                OMPL_ERROR("We require n >= m > 0 but have n=%d >= m=%d > 0.", n, m);
                                                throw ompl::Exception("Invalid dimensions.");
                                            }
                                        }
                                    }
                                    else
                                    {
                                        OMPL_ERROR("Cannot project onto type %d.", totalSpace->getType());
                                        throw ompl::Exception("Invalid Fiber type.");
                                    }
                                }
                                else
                                {
                                    OMPL_ERROR("baseSpace has %d subspaces. We can handle only 2.", baseSpace_subspaces);
                                    throw ompl::Exception("Invalid Fiber type.");
                                }
                            }
                            else
                            {
                                OMPL_ERROR("Cannot project onto type %d.", baseSpace->getType());
                                throw ompl::Exception("Invalid Fiber type.");
                            }
                        }
                    }
                    else
                    {
                        OMPL_ERROR("State compound %d and %d not recognized.", totalSpace_decomposed.at(0)->getType(),
                                   totalSpace_decomposed.at(1)->getType());
                        throw ompl::Exception("Invalid Fiber type.");
                    }
                }
            }
            else
            {
                OMPL_ERROR("totalSpace has %d subspaces, but we only support 2.", totalSpace_subspaces);
                throw ompl::Exception("Invalid Fiber type.");
            }
        }
    }
    return type_;
}

void ompl::geometric::FiberBundle::mergeStates(const base::State *qbaseSpace, const base::State *qfiber, base::State *qtotalSpace) const
{
    ////input : qbaseSpace \in baseSpace, qfiber \in fiber
    ////output: qtotalSpace = qbaseSpace \circ qfiber \in totalSpace
    const base::StateSpacePtr totalSpace_space = totalSpace->getStateSpace();
    const base::StateSpacePtr fiber_space = fiber->getStateSpace();
    const base::StateSpacePtr baseSpace_space = parent_->getSpaceInformation()->getStateSpace();

    switch (type_)
    {
        case IDENTITY_SPACE_RN:
        case IDENTITY_SPACE_SE2:
        case IDENTITY_SPACE_SE2RN:
        case IDENTITY_SPACE_SO2RN:
        case IDENTITY_SPACE_SE3:
        case IDENTITY_SPACE_SE3RN:
        {
            throw ompl::Exception("Cannot merge states for Identity space");
        }
        case RN_RM:
        {
            base::RealVectorStateSpace::StateType *sTotalSpace = qtotalSpace->as<base::RealVectorStateSpace::StateType>();
            const base::RealVectorStateSpace::StateType *sBaseSpace = qbaseSpace->as<base::RealVectorStateSpace::StateType>();
            const base::RealVectorStateSpace::StateType *sFiber = qfiber->as<base::RealVectorStateSpace::StateType>();

            for (unsigned int k = 0; k < baseSpace_dimension_; k++)
            {
                sTotalSpace->values[k] = sBaseSpace->values[k];
            }
            for (unsigned int k = baseSpace_dimension_; k < totalSpace_dimension_; k++)
            {
                sTotalSpace->values[k] = sFiber->values[k - baseSpace_dimension_];
            }
            break;
        }
        case SE2_R2:
        {
            base::SE2StateSpace::StateType *sTotalSpace = qtotalSpace->as<base::SE2StateSpace::StateType>();
            const base::RealVectorStateSpace::StateType *sBaseSpace = qbaseSpace->as<base::RealVectorStateSpace::StateType>();
            const base::SO2StateSpace::StateType *sFiber = qfiber->as<base::SO2StateSpace::StateType>();

            sTotalSpace->setXY(sBaseSpace->values[0], sBaseSpace->values[1]);
            sTotalSpace->setYaw(sFiber->value);

            break;
        }
        case SE3_R3:
        {
            base::SE3StateSpace::StateType *sTotalSpace = qtotalSpace->as<base::SE3StateSpace::StateType>();
            base::SO3StateSpace::StateType *sTotalSpace_rotation = &sTotalSpace->rotation();

            const base::RealVectorStateSpace::StateType *sBaseSpace = qbaseSpace->as<base::RealVectorStateSpace::StateType>();
            const base::SO3StateSpace::StateType *sFiber = qfiber->as<base::SO3StateSpace::StateType>();

            sTotalSpace->setXYZ(sBaseSpace->values[0], sBaseSpace->values[1], sBaseSpace->values[2]);

            sTotalSpace_rotation->x = sFiber->x;
            sTotalSpace_rotation->y = sFiber->y;
            sTotalSpace_rotation->z = sFiber->z;
            sTotalSpace_rotation->w = sFiber->w;

            break;
        }
        case SE3RN_R3:
        {
            base::SE3StateSpace::StateType *sTotalSpace_SE3 =
                qtotalSpace->as<base::CompoundState>()->as<base::SE3StateSpace::StateType>(0);
            base::SO3StateSpace::StateType *sTotalSpace_SO3 = &sTotalSpace_SE3->rotation();
            base::RealVectorStateSpace::StateType *sTotalSpace_RN =
                qtotalSpace->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

            const base::RealVectorStateSpace::StateType *sBaseSpace = qbaseSpace->as<base::RealVectorStateSpace::StateType>();
            const base::SO3StateSpace::StateType *sFiber_SO3 =
                qfiber->as<base::CompoundState>()->as<base::SO3StateSpace::StateType>(0);
            const base::RealVectorStateSpace::StateType *sFiber_RN =
                qfiber->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

            sTotalSpace_SE3->setXYZ(sBaseSpace->values[0], sBaseSpace->values[1], sBaseSpace->values[2]);
            sTotalSpace_SO3->x = sFiber_SO3->x;
            sTotalSpace_SO3->y = sFiber_SO3->y;
            sTotalSpace_SO3->z = sFiber_SO3->z;
            sTotalSpace_SO3->w = sFiber_SO3->w;

            for (unsigned int k = 0; k < fiber_dimension_ - 3; k++)
            {
                sTotalSpace_RN->values[k] = sFiber_RN->values[k];
            }

            break;
        }
        case SE2RN_SE2:
        {
            base::SE2StateSpace::StateType *sTotalSpace_SE2 =
                qtotalSpace->as<base::CompoundState>()->as<base::SE2StateSpace::StateType>(0);
            base::RealVectorStateSpace::StateType *sTotalSpace_RN =
                qtotalSpace->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

            const base::SE2StateSpace::StateType *sBaseSpace = qbaseSpace->as<base::SE2StateSpace::StateType>();
            const base::RealVectorStateSpace::StateType *sFiber = qfiber->as<base::RealVectorStateSpace::StateType>();

            sTotalSpace_SE2->setX(sBaseSpace->getX());
            sTotalSpace_SE2->setY(sBaseSpace->getY());
            sTotalSpace_SE2->setYaw(sBaseSpace->getYaw());

            for (unsigned int k = 0; k < fiber_dimension_; k++)
            {
                sTotalSpace_RN->values[k] = sFiber->values[k];
            }
            break;
        }
        case SO2RN_SO2:
        {
            base::SO2StateSpace::StateType *sTotalSpace_SO2 =
                qtotalSpace->as<base::CompoundState>()->as<base::SO2StateSpace::StateType>(0);
            base::RealVectorStateSpace::StateType *sTotalSpace_RN =
                qtotalSpace->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

            const base::SO2StateSpace::StateType *sBaseSpace = qbaseSpace->as<base::SO2StateSpace::StateType>();
            const base::RealVectorStateSpace::StateType *sFiber = qfiber->as<base::RealVectorStateSpace::StateType>();

            sTotalSpace_SO2->value = sBaseSpace->value;

            for (unsigned int k = 0; k < fiber_dimension_; k++)
            {
                sTotalSpace_RN->values[k] = sFiber->values[k];
            }
            break;
        }
        case SO2RN_SO2RM:
        {
            base::SO2StateSpace::StateType *sTotalSpace_SO2 =
                qtotalSpace->as<base::CompoundState>()->as<base::SO2StateSpace::StateType>(0);
            base::RealVectorStateSpace::StateType *sTotalSpace_RN =
                qtotalSpace->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

            const base::SO2StateSpace::StateType *sBaseSpace_SO2 =
                qbaseSpace->as<base::CompoundState>()->as<base::SO2StateSpace::StateType>(0);
            const base::RealVectorStateSpace::StateType *sBaseSpace_RM =
                qbaseSpace->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

            const base::RealVectorStateSpace::StateType *sFiber = qfiber->as<base::RealVectorStateSpace::StateType>();

            sTotalSpace_SO2->value = sBaseSpace_SO2->value;

            unsigned int M = totalSpace_dimension_ - fiber_dimension_ - 1;
            unsigned int N = fiber_dimension_;

            for (unsigned int k = 0; k < M; k++)
            {
                sTotalSpace_RN->values[k] = sBaseSpace_RM->values[k];
            }
            for (unsigned int k = M; k < M + N; k++)
            {
                sTotalSpace_RN->values[k] = sFiber->values[k - M];
            }
            break;
        }

        case SE2RN_R2:
        {
            base::SE2StateSpace::StateType *sTotalSpace_SE2 =
                qtotalSpace->as<base::CompoundState>()->as<base::SE2StateSpace::StateType>(0);
            base::RealVectorStateSpace::StateType *sTotalSpace_RN =
                qtotalSpace->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

            const base::RealVectorStateSpace::StateType *sBaseSpace = qbaseSpace->as<base::RealVectorStateSpace::StateType>();
            const base::SO2StateSpace::StateType *sFiber_SO2 =
                qfiber->as<base::CompoundState>()->as<base::SO2StateSpace::StateType>(0);
            const base::RealVectorStateSpace::StateType *sFiber_RN =
                qfiber->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

            sTotalSpace_SE2->setX(sBaseSpace->values[0]);
            sTotalSpace_SE2->setY(sBaseSpace->values[1]);
            sTotalSpace_SE2->setYaw(sFiber_SO2->value);

            for (unsigned int k = 0; k < fiber_dimension_ - 1; k++)
            {
                sTotalSpace_RN->values[k] = sFiber_RN->values[k];
            }
            break;
        }
        case SE2RN_SE2RM:
        {
            base::SE2StateSpace::StateType *sTotalSpace_SE2 =
                qtotalSpace->as<base::CompoundState>()->as<base::SE2StateSpace::StateType>(0);
            base::RealVectorStateSpace::StateType *sTotalSpace_RN =
                qtotalSpace->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

            const base::SE2StateSpace::StateType *sBaseSpace_SE2 =
                qbaseSpace->as<base::CompoundState>()->as<base::SE2StateSpace::StateType>(0);
            const base::RealVectorStateSpace::StateType *sBaseSpace_RM =
                qbaseSpace->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

            const base::RealVectorStateSpace::StateType *sFiber = qfiber->as<base::RealVectorStateSpace::StateType>();

            sTotalSpace_SE2->setX(sBaseSpace_SE2->getX());
            sTotalSpace_SE2->setY(sBaseSpace_SE2->getY());
            sTotalSpace_SE2->setYaw(sBaseSpace_SE2->getYaw());

            //[X Y YAW] [1...M-1][M...N-1]
            // SE2               RN
            unsigned int M = totalSpace_dimension_ - fiber_dimension_ - 3;
            unsigned int N = fiber_dimension_;

            for (unsigned int k = 0; k < M; k++)
            {
                sTotalSpace_RN->values[k] = sBaseSpace_RM->values[k];
            }
            for (unsigned int k = M; k < M + N; k++)
            {
                sTotalSpace_RN->values[k] = sFiber->values[k - M];
            }
            break;
        }
        case SE3RN_SE3:
        {
            base::SE3StateSpace::StateType *sTotalSpace_SE3 =
                qtotalSpace->as<base::CompoundState>()->as<base::SE3StateSpace::StateType>(0);
            base::SO3StateSpace::StateType *sTotalSpace_SE3_rotation = &sTotalSpace_SE3->rotation();
            base::RealVectorStateSpace::StateType *sTotalSpace_RN =
                qtotalSpace->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

            const base::SE3StateSpace::StateType *sBaseSpace = qbaseSpace->as<base::SE3StateSpace::StateType>();
            const base::SO3StateSpace::StateType *sBaseSpace_rotation = &sBaseSpace->rotation();
            const base::RealVectorStateSpace::StateType *sFiber = qfiber->as<base::RealVectorStateSpace::StateType>();

            sTotalSpace_SE3->setXYZ(sBaseSpace->getX(), sBaseSpace->getY(), sBaseSpace->getZ());
            sTotalSpace_SE3_rotation->x = sBaseSpace_rotation->x;
            sTotalSpace_SE3_rotation->y = sBaseSpace_rotation->y;
            sTotalSpace_SE3_rotation->z = sBaseSpace_rotation->z;
            sTotalSpace_SE3_rotation->w = sBaseSpace_rotation->w;

            for (unsigned int k = 0; k < fiber_dimension_; k++)
            {
                sTotalSpace_RN->values[k] = sFiber->values[k];
            }

            break;
        }
        case SE3RN_SE3RM:
        {
            base::SE3StateSpace::StateType *sTotalSpace_SE3 =
                qtotalSpace->as<base::CompoundState>()->as<base::SE3StateSpace::StateType>(0);
            base::SO3StateSpace::StateType *sTotalSpace_SE3_rotation = &sTotalSpace_SE3->rotation();
            base::RealVectorStateSpace::StateType *sTotalSpace_RN =
                qtotalSpace->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

            const base::SE3StateSpace::StateType *sBaseSpace_SE3 =
                qbaseSpace->as<base::CompoundState>()->as<base::SE3StateSpace::StateType>(0);
            const base::SO3StateSpace::StateType *sBaseSpace_SE3_rotation = &sBaseSpace_SE3->rotation();
            const base::RealVectorStateSpace::StateType *sBaseSpace_RM =
                qbaseSpace->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

            const base::RealVectorStateSpace::StateType *sFiber = qfiber->as<base::RealVectorStateSpace::StateType>();

            sTotalSpace_SE3->setXYZ(sBaseSpace_SE3->getX(), sBaseSpace_SE3->getY(), sBaseSpace_SE3->getZ());
            sTotalSpace_SE3_rotation->x = sBaseSpace_SE3_rotation->x;
            sTotalSpace_SE3_rotation->y = sBaseSpace_SE3_rotation->y;
            sTotalSpace_SE3_rotation->z = sBaseSpace_SE3_rotation->z;
            sTotalSpace_SE3_rotation->w = sBaseSpace_SE3_rotation->w;

            //[X Y Z YAW PITCH ROLL] [1...M-1][M...N-1]
            // SE3                                        RN
            unsigned int M = totalSpace_dimension_ - fiber_dimension_ - 6;
            unsigned int N = fiber_dimension_;

            for (unsigned int k = 0; k < M; k++)
            {
                sTotalSpace_RN->values[k] = sBaseSpace_RM->values[k];
            }
            for (unsigned int k = M; k < M + N; k++)
            {
                sTotalSpace_RN->values[k] = sFiber->values[k - M];
            }
            break;
        }
        default:
        {
            OMPL_ERROR("Type %d not implemented.", type_);
            throw ompl::Exception("Cannot merge states.");
        }
    }
}

void ompl::geometric::FiberBundle::projectFiber(const base::State *q, base::State *qfiber) const
{
    switch (type_)
    {
        case RN_RM:
        {
            const base::RealVectorStateSpace::StateType *sTotalSpace = q->as<base::RealVectorStateSpace::StateType>();
            base::RealVectorStateSpace::StateType *sFiber = qfiber->as<base::RealVectorStateSpace::StateType>();

            for (unsigned int k = baseSpace_dimension_; k < totalSpace_dimension_; k++)
            {
                sFiber->values[k - baseSpace_dimension_] = sTotalSpace->values[k];
            }
            break;
        }
        case SE2_R2:
        {
            const base::SE2StateSpace::StateType *sTotalSpace = q->as<base::SE2StateSpace::StateType>();
            base::SO2StateSpace::StateType *sFiber = qfiber->as<base::SO2StateSpace::StateType>();
            sFiber->value = sTotalSpace->getYaw();
            break;
        }
        case SE3_R3:
        {
            const base::SE3StateSpace::StateType *sTotalSpace = q->as<base::SE3StateSpace::StateType>();
            const base::SO3StateSpace::StateType *sTotalSpace_SO3 = &sTotalSpace->rotation();

            base::SO3StateSpace::StateType *sFiber_SO3 = qfiber->as<base::SO3StateSpace::StateType>();

            sFiber_SO3->x = sTotalSpace_SO3->x;
            sFiber_SO3->y = sTotalSpace_SO3->y;
            sFiber_SO3->z = sTotalSpace_SO3->z;
            sFiber_SO3->w = sTotalSpace_SO3->w;

            break;
        }
        case SE3RN_R3:
        {
            const base::SE3StateSpace::StateType *sTotalSpace_SE3 =
                q->as<base::CompoundState>()->as<base::SE3StateSpace::StateType>(0);
            const base::SO3StateSpace::StateType *sTotalSpace_SO3 = &sTotalSpace_SE3->rotation();
            const base::RealVectorStateSpace::StateType *sTotalSpace_RN =
                q->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

            base::SO3StateSpace::StateType *sFiber_SO3 =
                qfiber->as<base::CompoundState>()->as<base::SO3StateSpace::StateType>(0);
            base::RealVectorStateSpace::StateType *sFiber_RN =
                qfiber->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

            sFiber_SO3->x = sTotalSpace_SO3->x;
            sFiber_SO3->y = sTotalSpace_SO3->y;
            sFiber_SO3->z = sTotalSpace_SO3->z;
            sFiber_SO3->w = sTotalSpace_SO3->w;
            for (unsigned int k = 0; k < fiber_dimension_ - 3; k++)
            {
                sFiber_RN->values[k] = sTotalSpace_RN->values[k];
            }

            break;
        }
        case SE2RN_R2:
        {
            const base::SE2StateSpace::StateType *sTotalSpace_SE2 =
                q->as<base::CompoundState>()->as<base::SE2StateSpace::StateType>(0);
            const base::RealVectorStateSpace::StateType *sTotalSpace_RN =
                q->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

            base::SO2StateSpace::StateType *sFiber_SO2 =
                qfiber->as<base::CompoundState>()->as<base::SO2StateSpace::StateType>(0);
            base::RealVectorStateSpace::StateType *sFiber_RN =
                qfiber->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

            sFiber_SO2->value = sTotalSpace_SE2->getYaw();
            for (unsigned int k = 0; k < fiber_dimension_ - 1; k++)
            {
                sFiber_RN->values[k] = sTotalSpace_RN->values[k];
            }
            break;
        }
        case SE2RN_SE2RM:
        case SO2RN_SO2RM:
        {
            const base::RealVectorStateSpace::StateType *sTotalSpace_RN =
                q->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

            const base::RealVectorStateSpace::StateType *sFiber = qfiber->as<base::RealVectorStateSpace::StateType>();

            unsigned int N = totalSpace_dimension_ - fiber_dimension_ - 3;
            for (unsigned int k = N; k < totalSpace_dimension_ - 3; k++)
            {
                sFiber->values[k - N] = sTotalSpace_RN->values[k];
            }
            break;
        }
        case SE2RN_SE2:
        case SE3RN_SE3:
        case SO2RN_SO2:
        {
            const base::RealVectorStateSpace::StateType *sTotalSpace_RN =
                q->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);
            base::RealVectorStateSpace::StateType *sFiber = qfiber->as<base::RealVectorStateSpace::StateType>();

            for (unsigned int k = 0; k < fiber_dimension_; k++)
            {
                sFiber->values[k] = sTotalSpace_RN->values[k];
            }

            break;
        }
        case SE3RN_SE3RM:
        {
            const base::RealVectorStateSpace::StateType *sTotalSpace_RN =
                q->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

            const base::RealVectorStateSpace::StateType *sFiber = qfiber->as<base::RealVectorStateSpace::StateType>();

            unsigned int N = totalSpace_dimension_ - fiber_dimension_ - 6;
            for (unsigned int k = N; k < totalSpace_dimension_ - 6; k++)
            {
                sFiber->values[k - N] = sTotalSpace_RN->values[k];
            }
            break;
        }
        default:
        {
            OMPL_ERROR("Type %d not implemented.", type_);
            throw ompl::Exception("Cannot project onto fiber.");
        }
    }
}

void ompl::geometric::FiberBundle::projectBaseSpace(const base::State *q, base::State *qbaseSpace) const
{
    switch (type_)
    {
        case IDENTITY_SPACE_RN:
        case IDENTITY_SPACE_SE2:
        case IDENTITY_SPACE_SE2RN:
        case IDENTITY_SPACE_SO2RN:
        case IDENTITY_SPACE_SE3:
        case IDENTITY_SPACE_SE3RN:
        {
            // Identity function
            totalSpace->getStateSpace()->copyState(qbaseSpace, q);
            break;
        }
        case RN_RM:
        {
            const base::RealVectorStateSpace::StateType *sTotalSpace = q->as<base::RealVectorStateSpace::StateType>();
            base::RealVectorStateSpace::StateType *sBaseSpace = qbaseSpace->as<base::RealVectorStateSpace::StateType>();

            for (unsigned int k = 0; k < baseSpace_dimension_; k++)
            {
                sBaseSpace->values[k] = sTotalSpace->values[k];
            }
            break;
        }
        case SE2_R2:
        {
            const base::SE2StateSpace::StateType *sTotalSpace = q->as<base::SE2StateSpace::StateType>();
            base::RealVectorStateSpace::StateType *sBaseSpace = qbaseSpace->as<base::RealVectorStateSpace::StateType>();
            sBaseSpace->values[0] = sTotalSpace->getX();
            sBaseSpace->values[1] = sTotalSpace->getY();
            break;
        }
        case SE2RN_R2:
        {
            const base::SE2StateSpace::StateType *sTotalSpace =
                q->as<base::CompoundState>()->as<base::SE2StateSpace::StateType>(0);
            base::RealVectorStateSpace::StateType *sBaseSpace = qbaseSpace->as<base::RealVectorStateSpace::StateType>();
            sBaseSpace->values[0] = sTotalSpace->getX();
            sBaseSpace->values[1] = sTotalSpace->getY();
            break;
        }
        case SE2RN_SE2:
        {
            const base::SE2StateSpace::StateType *sTotalSpace_SE2 =
                q->as<base::CompoundState>()->as<base::SE2StateSpace::StateType>(0);
            base::SE2StateSpace::StateType *sBaseSpace_SE2 = qbaseSpace->as<base::SE2StateSpace::StateType>();

            sBaseSpace_SE2->setX(sTotalSpace_SE2->getX());
            sBaseSpace_SE2->setY(sTotalSpace_SE2->getY());
            sBaseSpace_SE2->setYaw(sTotalSpace_SE2->getYaw());

            break;
        }
        case SO2RN_SO2:
        {
            const base::SO2StateSpace::StateType *sTotalSpace_SO2 =
                q->as<base::CompoundState>()->as<base::SO2StateSpace::StateType>(0);
            base::SO2StateSpace::StateType *sBaseSpace_SO2 = qbaseSpace->as<base::SO2StateSpace::StateType>();

            sBaseSpace_SO2->value = sTotalSpace_SO2->value;

            break;
        }
        case SO2RN_SO2RM:
        {
            const base::SO2StateSpace::StateType *sTotalSpace_SO2 =
                q->as<base::CompoundState>()->as<base::SO2StateSpace::StateType>(0);
            const base::RealVectorStateSpace::StateType *sTotalSpace_RN =
                q->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

            base::SO2StateSpace::StateType *sBaseSpace_SO2 =
                qbaseSpace->as<base::CompoundState>()->as<base::SO2StateSpace::StateType>(0);
            base::RealVectorStateSpace::StateType *sBaseSpace_RM =
                qbaseSpace->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

            sBaseSpace_SO2->value = sTotalSpace_SO2->value;

            for (unsigned int k = 0; k < baseSpace_dimension_ - 1; k++)
            {
                sBaseSpace_RM->values[k] = sTotalSpace_RN->values[k];
            }
            break;
        }

        case SE2RN_SE2RM:
        {
            const base::SE2StateSpace::StateType *sTotalSpace_SE2 =
                q->as<base::CompoundState>()->as<base::SE2StateSpace::StateType>(0);
            const base::RealVectorStateSpace::StateType *sTotalSpace_RN =
                q->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

            base::SE2StateSpace::StateType *sBaseSpace_SE2 =
                qbaseSpace->as<base::CompoundState>()->as<base::SE2StateSpace::StateType>(0);
            base::RealVectorStateSpace::StateType *sBaseSpace_RN =
                qbaseSpace->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

            sBaseSpace_SE2->setX(sTotalSpace_SE2->getX());
            sBaseSpace_SE2->setY(sTotalSpace_SE2->getY());
            sBaseSpace_SE2->setYaw(sTotalSpace_SE2->getYaw());

            for (unsigned int k = 0; k < baseSpace_dimension_ - 3; k++)
            {
                sBaseSpace_RN->values[k] = sTotalSpace_RN->values[k];
            }
            break;
        }
        case SE3_R3:
        {
            const base::SE3StateSpace::StateType *sTotalSpace = q->as<base::SE3StateSpace::StateType>();
            base::RealVectorStateSpace::StateType *sBaseSpace = qbaseSpace->as<base::RealVectorStateSpace::StateType>();

            sBaseSpace->values[0] = sTotalSpace->getX();
            sBaseSpace->values[1] = sTotalSpace->getY();
            sBaseSpace->values[2] = sTotalSpace->getZ();

            break;
        }
        case SE3RN_R3:
        {
            const base::SE3StateSpace::StateType *sTotalSpace_SE3 =
                q->as<base::CompoundState>()->as<base::SE3StateSpace::StateType>(0);
            base::RealVectorStateSpace::StateType *sBaseSpace = qbaseSpace->as<base::RealVectorStateSpace::StateType>();

            sBaseSpace->values[0] = sTotalSpace_SE3->getX();
            sBaseSpace->values[1] = sTotalSpace_SE3->getY();
            sBaseSpace->values[2] = sTotalSpace_SE3->getZ();

            break;
        }
        case SE3RN_SE3:
        {
            const base::SE3StateSpace::StateType *sTotalSpace_SE3 =
                q->as<base::CompoundState>()->as<base::SE3StateSpace::StateType>(0);
            const base::SO3StateSpace::StateType *sTotalSpace_SE3_rotation = &sTotalSpace_SE3->rotation();

            base::SE3StateSpace::StateType *sBaseSpace = qbaseSpace->as<base::SE3StateSpace::StateType>();
            base::SO3StateSpace::StateType *sBaseSpace_rotation = &sBaseSpace->rotation();

            sBaseSpace->setXYZ(sTotalSpace_SE3->getX(), sTotalSpace_SE3->getY(), sTotalSpace_SE3->getZ());
            sBaseSpace_rotation->x = sTotalSpace_SE3_rotation->x;
            sBaseSpace_rotation->y = sTotalSpace_SE3_rotation->y;
            sBaseSpace_rotation->z = sTotalSpace_SE3_rotation->z;
            sBaseSpace_rotation->w = sTotalSpace_SE3_rotation->w;

            break;
        }
        case SE3RN_SE3RM:
        {
            const base::SE3StateSpace::StateType *sTotalSpace_SE3 =
                q->as<base::CompoundState>()->as<base::SE3StateSpace::StateType>(0);
            const base::SO3StateSpace::StateType *sTotalSpace_SE3_rotation = &sTotalSpace_SE3->rotation();
            const base::RealVectorStateSpace::StateType *sTotalSpace_RN =
                q->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

            base::SE3StateSpace::StateType *sBaseSpace_SE3 =
                qbaseSpace->as<base::CompoundState>()->as<base::SE3StateSpace::StateType>(0);
            base::SO3StateSpace::StateType *sBaseSpace_rotation = &sBaseSpace_SE3->rotation();
            base::RealVectorStateSpace::StateType *sBaseSpace_RN =
                qbaseSpace->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

            sBaseSpace_SE3->setXYZ(sTotalSpace_SE3->getX(), sTotalSpace_SE3->getY(), sTotalSpace_SE3->getZ());
            sBaseSpace_rotation->x = sTotalSpace_SE3_rotation->x;
            sBaseSpace_rotation->y = sTotalSpace_SE3_rotation->y;
            sBaseSpace_rotation->z = sTotalSpace_SE3_rotation->z;
            sBaseSpace_rotation->w = sTotalSpace_SE3_rotation->w;

            for (unsigned int k = 0; k < baseSpace_dimension_ - 6; k++)
            {
                sBaseSpace_RN->values[k] = sTotalSpace_RN->values[k];
            }
            break;
        }
        default:
        {
            OMPL_ERROR("Cannot project onto baseSpace. Type %d not implemented.", type_);
            throw ompl::Exception("Cannot project onto baseSpace.");
        }
    }
}

const ompl::base::SpaceInformationPtr &ompl::geometric::FiberBundle::getFiber() const
{
    return fiber;
}

const ompl::base::SpaceInformationPtr &ompl::geometric::FiberBundle::getTotalSpace() const
{
    return totalSpace;
}

const ompl::base::SpaceInformationPtr &ompl::geometric::FiberBundle::getBaseSpace() const
{
    return baseSpace;
}

unsigned int ompl::geometric::FiberBundle::getFiberDimension() const
{
    return fiber_dimension_;
}

unsigned int ompl::geometric::FiberBundle::getTotalSpaceDimension() const
{
    return totalSpace->getStateDimension();
}

unsigned int ompl::geometric::FiberBundle::getDimension() const
{
    return getTotalSpaceDimension();
}

unsigned int ompl::geometric::FiberBundle::getBaseSpaceDimension() const
{
    return baseSpace_dimension_;
}

const ompl::base::StateSamplerPtr &ompl::geometric::FiberBundle::getFiberSamplerPtr() const
{
    return fiber_sampler_;
}

const ompl::base::StateSamplerPtr &ompl::geometric::FiberBundle::getTotalSpaceSamplerPtr() const
{
    return totalSpace_sampler_;
}

bool ompl::geometric::FiberBundle::hasSolution()
{
    if (!hasSolution_)
    {
        base::PathPtr path;
        hasSolution_ = getSolution(path);
    }
    return hasSolution_;
}

unsigned int ompl::geometric::FiberBundle::getTotalNumberOfSamples() const
{
    return totalNumberOfSamples_;
}

unsigned int ompl::geometric::FiberBundle::getTotalNumberOfFeasibleSamples() const
{
    return totalNumberOfFeasibleSamples_;
}

ompl::geometric::FiberBundle *ompl::geometric::FiberBundle::getParent() const
{
    return parent_;
}

ompl::geometric::FiberBundle *ompl::geometric::FiberBundle::getChild() const
{
    return child_;
}

void ompl::geometric::FiberBundle::setChild(ompl::geometric::FiberBundle *child)
{
    child_ = child;
}

void ompl::geometric::FiberBundle::setParent(ompl::geometric::FiberBundle *parent)
{
    parent_ = parent;
}

unsigned int ompl::geometric::FiberBundle::getLevel() const
{
    return level_;
}

void ompl::geometric::FiberBundle::setLevel(unsigned int level)
{
    level_ = level;
}

ompl::geometric::FiberBundle::FiberBundleType ompl::geometric::FiberBundle::getType() const
{
    return type_;
}

ompl::base::OptimizationObjectivePtr ompl::geometric::FiberBundle::getOptimizationObjectivePtr() const
{
    return opt_;
}

bool ompl::geometric::FiberBundle::sampleBaseSpace(base::State *q_random)
{
    totalSpace_sampler_->sampleUniform(q_random);
    return true;
}

bool ompl::geometric::FiberBundle::sample(base::State *q_random)
{
    bool valid = false;
    if (!hasParent())
    {
        // return totalSpace_valid_sampler->sample(q_random);
        totalSpace_sampler_->sampleUniform(q_random);
        valid = totalSpace->isValid(q_random);
    }
    else
    {
        if (fiber_dimension_ > 0)
        {
            // Adjusted sampling function: Sampling in G0 x fiber
            fiber_sampler_->sampleUniform(s_fiber_tmp_);
            parent_->sampleBaseSpace(s_baseSpace_tmp_);
            mergeStates(s_baseSpace_tmp_, s_fiber_tmp_, q_random);
        }
        else
        {
            parent_->sampleBaseSpace(q_random);
        }
        valid = totalSpace->isValid(q_random);
    }
    totalNumberOfSamples_++;
    if (valid)
    {
        totalNumberOfFeasibleSamples_++;
    }

    return valid;
}

double ompl::geometric::FiberBundle::getImportance() const
{
    double N = (double)totalNumberOfSamples_;
    return 1.0 / (N + 1);
}

void ompl::geometric::FiberBundle::print(std::ostream &out) const
{
    out << "[FiberBundle: id" << id_ << " |lvl" << level_ << "] ";
    unsigned int sublevel = std::max(1U, level_);
    if (!hasParent())
    {
        out << "X" << sublevel << "=Q" << sublevel << ": ";
        if (totalSpace->getStateSpace()->getType() == base::STATE_SPACE_SE2)
        {
            out << "SE(2)";
        }
        else if (totalSpace->getStateSpace()->getType() == base::STATE_SPACE_SE3)
        {
            out << "SE(3)";
        }
        else if (totalSpace->getStateSpace()->getType() == base::STATE_SPACE_REAL_VECTOR)
        {
            out << "R^" << totalSpace->getStateDimension();
        }
        else
        {
            out << "unknown";
        }
    }
    else
    {
        out << "X" << sublevel << "=Q" << sublevel << ": ";
        switch (type_)
        {
            case FiberBundleType::IDENTITY_SPACE_RN:
            {
                out << "R^" << baseSpace_dimension_ << " | Q" << level_ + 1 << ": R^" << totalSpace_dimension_;
                break;
            }
            case FiberBundleType::IDENTITY_SPACE_SE2:
            {
                out << "SE(2)"
                    << " | Q" << level_ + 1 << ": SE(2)";
                break;
            }
            case FiberBundleType::IDENTITY_SPACE_SE2RN:
            {
                out << "SE(2)xR^" << baseSpace_dimension_ << " | Q" << level_ + 1 << ": SE(2)xR^" << totalSpace_dimension_;
                break;
            }
            case FiberBundleType::IDENTITY_SPACE_SO2RN:
            {
                out << "SO(2)xR^" << baseSpace_dimension_ << " | Q" << level_ + 1 << ": SO(2)xR^" << totalSpace_dimension_;
                break;
            }
            case FiberBundleType::IDENTITY_SPACE_SE3:
            {
                out << "SE(3)"
                    << " | Q" << level_ + 1 << ": SE(3)";
                break;
            }
            case FiberBundleType::IDENTITY_SPACE_SE3RN:
            {
                out << "SE(3)xR^" << baseSpace_dimension_ << " | Q" << level_ + 1 << ": SE(3)xR^" << totalSpace_dimension_;
                break;
            }
            case FiberBundleType::RN_RM:
            {
                out << "R^" << baseSpace_dimension_ << " | Q" << level_ + 1 << ": R^" << totalSpace_dimension_ << " | X" << level_ + 1
                    << ": R^" << totalSpace_dimension_ - baseSpace_dimension_;
                break;
            }
            case FiberBundleType::SE2_R2:
            {
                out << "R^2 | Q" << level_ + 1 << ": SE(2) | X" << level_ + 1 << ": SO(2)";
                break;
            }
            case FiberBundleType::SE3_R3:
            {
                out << "R^3 | Q" << level_ + 1 << ": SE(3) | X" << level_ + 1 << ": SO(3)";
                break;
            }
            case FiberBundleType::SE2RN_SE2:
            {
                out << "SE(2) | Q" << level_ + 1 << ": SE(2)xR^" << fiber_dimension_ << " | X" << level_ + 1 << ": R^"
                    << fiber_dimension_;
                break;
            }
            case FiberBundleType::SO2RN_SO2:
            {
                out << "SO(2) | Q" << level_ + 1 << ": SO(2)xR^" << fiber_dimension_ << " | X" << level_ + 1 << ": R^"
                    << fiber_dimension_;
                break;
            }
            case FiberBundleType::SE3RN_SE3:
            {
                out << "SE(3) | Q" << level_ + 1 << ": SE(3)xR^" << fiber_dimension_ << " | X" << level_ + 1 << ": R^"
                    << fiber_dimension_;
                break;
            }
            case FiberBundleType::SE2RN_SE2RM:
            {
                out << "SE(2)xR^" << baseSpace_dimension_ - 3 << " | Q" << level_ + 1 << ": SE(2)xR^" << totalSpace_dimension_ - 3
                    << " | X" << level_ + 1 << ": R^" << fiber_dimension_;
                break;
            }
            case FiberBundleType::SO2RN_SO2RM:
            {
                out << "SO(2)xR^" << baseSpace_dimension_ - 1 << " | Q" << level_ + 1 << ": SO(2)xR^" << totalSpace_dimension_ - 1
                    << " | X" << level_ + 1 << ": R^" << fiber_dimension_;
                break;
            }
            case FiberBundleType::SE3RN_SE3RM:
            {
                out << "SE(3)xR^" << baseSpace_dimension_ - 6 << " | Q" << level_ + 1 << ": SE(3)xR^" << totalSpace_dimension_ - 6
                    << " | X" << level_ + 1 << ": R^" << fiber_dimension_;
                break;
            }
            default:
            {
                out << "unknown type_: " << type_;
            }
        }
    }
    out << std::endl << " --[Importance:" << getImportance() << "]";
    out << std::endl << " --[Measure   :" << totalSpace->getSpaceMeasure() << "]";
}

namespace ompl
{
    namespace geometric
    {
        std::ostream &operator<<(std::ostream &out, const FiberBundle &fiberbundle_)
        {
            fiberbundle_.print(out);
            return out;
        }
    }  // namespace geometric
}  // namespace ompl
