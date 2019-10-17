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
#include <ompl/geometric/planners/quotientspace/datastructures/QuotientSpace.h>

#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/SO3StateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>

#include <ompl/util/Exception.h>

unsigned int ompl::geometric::QuotientSpace::counter_ = 0;

ompl::geometric::QuotientSpace::QuotientSpace(const base::SpaceInformationPtr &si, QuotientSpace *parent_)
  : base::Planner(si, "QuotientSpace"), Q1(si), parent_(parent_)
{
    id_ = counter_++;

    OMPL_DEVMSG1("QuotientSpace %s", id_);

    if (parent_ != nullptr)
        parent_->setChild(this);  // need to be able to traverse down the tree

    const base::StateSpacePtr Q1_space = Q1->getStateSpace();


    if (parent_ == nullptr)
    {
        OMPL_DEVMSG1("ATOMIC dimension: %d measure: %f", Q1_space->getDimension(), Q1_space->getMeasure());
        type_ = ATOMIC;
    }
    else
    {
        Q0 = parent_->getSpaceInformation();
        const base::StateSpacePtr Q0_space = Q0->getStateSpace();
        // X1 = Q1 / Q0
        const base::StateSpacePtr X1_space = computeQuotientSpace(Q1_space, Q0_space);

        if (X1_space != nullptr)
        {
            X1 = std::make_shared<base::SpaceInformation>(X1_space);
            X1_sampler_ = X1->allocStateSampler();
            if (Q0_space->getDimension() + X1_space->getDimension() != Q1_space->getDimension())
            {
                throw ompl::Exception("QuotientSpace Dimensions are wrong.");
            }
            OMPL_DEVMSG1("Q0 dimension: %d measure: %f", Q0_space->getDimension(), Q0_space->getMeasure());
            OMPL_DEVMSG1("X1 dimension: %d measure: %f", X1_space->getDimension(), X1_space->getMeasure());
            OMPL_DEVMSG1("Q1 dimension: %d measure: %f", Q1_space->getDimension(), Q1_space->getMeasure());
            if ((Q0_space->getMeasure() <= 0) || (X1_space->getMeasure() <= 0) || (Q1_space->getMeasure() <= 0))
            {
                throw ompl::Exception("Zero-measure QuotientSpace detected.");
            }
            if (!X1_sampler_)
            {
                X1_sampler_ = X1->allocStateSampler();
            }
            checkSpaceHasFiniteMeasure(X1_space);
        }
        else
        {
            OMPL_DEVMSG1("Q0 dimension: %d measure: %f", Q0_space->getDimension(), Q0_space->getMeasure());
            OMPL_DEVMSG1("Q1 dimension: %d measure: %f", Q1_space->getDimension(), Q1_space->getMeasure());
        }
        checkSpaceHasFiniteMeasure(Q0_space);
    }
    checkSpaceHasFiniteMeasure(Q1_space);

    if (!Q1_valid_sampler_)
    {
        Q1_valid_sampler_ = Q1->allocValidStateSampler();
    }
    if (!Q1_sampler_)
    {
        Q1_sampler_ = Q1->allocStateSampler();
    }
    if (parent_ != nullptr)
    {
        s_Q0_tmp_ = Q0->allocState();
        if (X1_dimension_ > 0)
            s_X1_tmp_ = X1->allocState();
    }
}

ompl::geometric::QuotientSpace::~QuotientSpace()
{
    if (parent_ != nullptr)
    {
        if (s_Q0_tmp_)
            Q0->freeState(s_Q0_tmp_);
        if (X1 && s_X1_tmp_)
            X1->freeState(s_X1_tmp_);
    }
}

void ompl::geometric::QuotientSpace::setup()
{
    BaseT::setup();
    hasSolution_ = false;
    firstRun_ = true;
}

void ompl::geometric::QuotientSpace::clear()
{
    BaseT::clear();
    totalNumberOfSamples_ = 0;
    totalNumberOfFeasibleSamples_ = 0;

    hasSolution_ = false;
    firstRun_ = true;
    if (parent_ == nullptr && X1_dimension_ > 0)
        X1_sampler_.reset();

    pdef_->clearSolutionPaths();
}

void ompl::geometric::QuotientSpace::checkSpaceHasFiniteMeasure(const base::StateSpacePtr space) const
{
    if (space->getMeasure() >= std::numeric_limits<double>::infinity())
    {
        const base::StateSpacePtr Q0_space = Q0->getStateSpace();
        const base::StateSpacePtr Q1_space = Q1->getStateSpace();
        OMPL_ERROR("Q0 dimension: %d measure: %f", Q0_space->getDimension(), Q0_space->getMeasure());
        OMPL_ERROR("Q1 dimension: %d measure: %f", Q1_space->getDimension(), Q1_space->getMeasure());
        if (X1 != nullptr)
        {
            const base::StateSpacePtr X1_space = X1->getStateSpace();
            OMPL_ERROR("X1 dimension: %d measure: %f", X1_space->getDimension(), X1_space->getMeasure());
        }
        throw ompl::Exception("QuotientSpace has no bounds");
    }
}

ompl::base::PlannerStatus ompl::geometric::QuotientSpace::solve(const base::PlannerTerminationCondition &ptc)
{
    (void)ptc;
    throw ompl::Exception("A Quotient-Space cannot be solved alone. Use class MultiQuotient to solve Quotient-Spaces.");
}

void ompl::geometric::QuotientSpace::setProblemDefinition(const base::ProblemDefinitionPtr &pdef)
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

void ompl::geometric::QuotientSpace::resetCounter()
{
    QuotientSpace::counter_ = 0;
}

const ompl::base::StateSpacePtr ompl::geometric::QuotientSpace::computeQuotientSpace(const base::StateSpacePtr Q1,
                                                                                     const base::StateSpacePtr Q0)
{
    type_ = identifyQuotientSpaceType(Q1, Q0);

    base::StateSpacePtr X1{nullptr};
    Q1_dimension_ = Q1->getDimension();
    Q0_dimension_ = Q0->getDimension();

    if (Q0_dimension_ == 0 || Q1_dimension_ == 0)
    {
        OMPL_ERROR("Q0 has dimension %d.", Q0_dimension_);
        OMPL_ERROR("Q1 has dimension %d.", Q1_dimension_);
        throw ompl::Exception("Detected Zero-dimensional QuotientSpace.");
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
            X1_dimension_ = 0;
            break;
        }
        case RN_RM:
        {
            unsigned int N = Q1_dimension_ - Q0_dimension_;
            X1 = std::make_shared<base::RealVectorStateSpace>(N);
            X1_dimension_ = N;

            base::RealVectorBounds Q1_bounds = std::static_pointer_cast<base::RealVectorStateSpace>(Q1)->getBounds();
            std::vector<double> low;
            low.resize(N);
            std::vector<double> high;
            high.resize(N);
            base::RealVectorBounds X1_bounds(N);
            for (unsigned int k = 0; k < N; k++)
            {
                X1_bounds.setLow(k, Q1_bounds.low.at(k + Q0_dimension_));
                X1_bounds.setHigh(k, Q1_bounds.high.at(k + Q0_dimension_));
            }
            std::static_pointer_cast<base::RealVectorStateSpace>(X1)->setBounds(X1_bounds);

            break;
        }
        case SE2_R2:
        {
            X1_dimension_ = 1;
            X1 = std::make_shared<base::SO2StateSpace>();
            break;
        }
        case SE3_R3:
        {
            X1_dimension_ = 3;
            X1 = std::make_shared<base::SO3StateSpace>();
            break;
        }
        case SE2RN_SE2:
        case SE3RN_SE3:
        case SO2RN_SO2:
        {
            base::CompoundStateSpace *Q1_compound = Q1->as<base::CompoundStateSpace>();
            const std::vector<base::StateSpacePtr> Q1_decomposed = Q1_compound->getSubspaces();

            X1_dimension_ = Q1_decomposed.at(1)->getDimension();

            X1 = std::make_shared<base::RealVectorStateSpace>(X1_dimension_);
            std::static_pointer_cast<base::RealVectorStateSpace>(X1)->setBounds(
                std::static_pointer_cast<base::RealVectorStateSpace>(Q1_decomposed.at(1))->getBounds());

            break;
        }
        case SE2RN_R2:
        {
            base::CompoundStateSpace *Q1_compound = Q1->as<base::CompoundStateSpace>();
            const std::vector<base::StateSpacePtr> Q1_decomposed = Q1_compound->getSubspaces();
            const std::vector<base::StateSpacePtr> Q1_SE2_decomposed =
                Q1_decomposed.at(0)->as<base::CompoundStateSpace>()->getSubspaces();

            const base::RealVectorStateSpace *Q1_RN = Q1_decomposed.at(1)->as<base::RealVectorStateSpace>();
            unsigned int N = Q1_RN->getDimension();

            base::StateSpacePtr SO2(new base::SO2StateSpace());
            base::StateSpacePtr RN(new base::RealVectorStateSpace(N));
            RN->as<base::RealVectorStateSpace>()->setBounds(Q1_RN->getBounds());

            X1 = SO2 + RN;
            X1_dimension_ = 1 + N;
            break;
        }
        case SE3RN_R3:
        {
            base::CompoundStateSpace *Q1_compound = Q1->as<base::CompoundStateSpace>();
            const std::vector<base::StateSpacePtr> Q1_decomposed = Q1_compound->getSubspaces();
            const std::vector<base::StateSpacePtr> Q1_SE3_decomposed =
                Q1_decomposed.at(0)->as<base::CompoundStateSpace>()->getSubspaces();

            // const base::SE3StateSpace *Q1_SE3 = Q1_SE3_decomposed.at(0)->as<base::SE3StateSpace>();
            // const base::SO3StateSpace *Q1_SO3 = Q1_SE3_decomposed.at(1)->as<base::SO3StateSpace>();
            const base::RealVectorStateSpace *Q1_RN = Q1_decomposed.at(1)->as<base::RealVectorStateSpace>();
            unsigned int N = Q1_RN->getDimension();

            base::StateSpacePtr SO3(new base::SO3StateSpace());
            base::StateSpacePtr RN(new base::RealVectorStateSpace(N));
            RN->as<base::RealVectorStateSpace>()->setBounds(Q1_RN->getBounds());

            X1 = SO3 + RN;
            X1_dimension_ = 3 + N;
            break;
        }
        case SE2RN_SE2RM:
        case SO2RN_SO2RM:
        case SE3RN_SE3RM:
        {
            base::CompoundStateSpace *Q1_compound = Q1->as<base::CompoundStateSpace>();
            const std::vector<base::StateSpacePtr> Q1_decomposed = Q1_compound->getSubspaces();
            base::CompoundStateSpace *Q0_compound = Q0->as<base::CompoundStateSpace>();
            const std::vector<base::StateSpacePtr> Q0_decomposed = Q0_compound->getSubspaces();

            unsigned int N = Q1_decomposed.at(1)->getDimension();
            unsigned int M = Q0_decomposed.at(1)->getDimension();
            X1_dimension_ = N - M;
            X1 = std::make_shared<base::RealVectorStateSpace>(X1_dimension_);

            base::RealVectorBounds Q1_bounds =
                std::static_pointer_cast<base::RealVectorStateSpace>(Q1_decomposed.at(1))->getBounds();
            std::vector<double> low;
            low.resize(X1_dimension_);
            std::vector<double> high;
            high.resize(X1_dimension_);
            base::RealVectorBounds X1_bounds(X1_dimension_);
            for (unsigned int k = 0; k < X1_dimension_; k++)
            {
                X1_bounds.setLow(k, Q1_bounds.low.at(k + M));
                X1_bounds.setHigh(k, Q1_bounds.high.at(k + M));
            }
            std::static_pointer_cast<base::RealVectorStateSpace>(X1)->setBounds(X1_bounds);
            break;
        }
        default:
        {
            OMPL_ERROR("Unknown QuotientSpace type: %d", type_);
            throw ompl::Exception("Unknown type");
        }
    }
    return X1;
}

ompl::geometric::QuotientSpace::QuotientSpaceType
ompl::geometric::QuotientSpace::identifyQuotientSpaceType(const base::StateSpacePtr Q1, const base::StateSpacePtr Q0)
{
    //
    // We can currently handle 11 types of quotient-space mappings.
    // Emptyset is used for constraint relaxations.
    //
    //   (1)  Q1 Rn     , Q0 Rm     [0<m<=n]  => X1 = R(n-m) \union {\emptyset}
    //   (2a) Q1 SE2    , Q0 R2               => X1 = SO2
    //   (2b) Q1 SE2    , Q0 SE2              => X1 = \emptyset
    //   (3a) Q1 SE3    , Q0 R3               => X1 = SO3
    //   (3b) Q1 SE3    , Q0 SE3              => X1 = \emptyset
    //
    //   (4)  Q1 SE3xRn , Q0 SE3              => X1 = Rn
    //   (5)  Q1 SE3xRn , Q0 R3               => X1 = SO3xRn
    //   (6)  Q1 SE3xRn , Q0 SE3xRm [0<m<=n ] => X1 = R(n-m) \union {\emptyset}
    //
    //   (7)  Q1 SE2xRn , Q0 SE2              => X1 = Rn
    //   (8)  Q1 SE2xRn , Q0 R2               => X1 = SO2xRN
    //   (9)  Q1 SE2xRn , Q0 SE2xRm [0<m<=n ] => X1 = R(n-m) \union {\emptyset}
    //
    //  (10)  Q1 SO2xRn , Q0 SO2              => X1 = Rn
    //  (11)  Q1 SO2xRn , Q0 SO2xRm [0<m<=n ] => X1 = R(n-m) \union {\emptyset}

    if (!Q1->isCompound())
    {
        // ##############################################################################/
        //------------------ non-compound cases:
        // ##############################################################################/
        //
        //------------------ (1) Q1 = Rn, Q0 = Rm, 0<m<n, X1 = R(n-m)
        if (Q1->getType() == base::STATE_SPACE_REAL_VECTOR)
        {
            unsigned int n = Q1->getDimension();
            if (Q0->getType() == base::STATE_SPACE_REAL_VECTOR)
            {
                unsigned int m = Q0->getDimension();
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
                OMPL_ERROR("Q1 is R^%d but Q0 type %d is not handled.", n, Q0->getType());
                throw ompl::Exception("INVALID_STATE_TYPE");
            }
        }
        else
        {
            OMPL_ERROR("Q1 is non-compound state, but its type %d is not handled.", Q1->getType());
            throw ompl::Exception("INVALID_STATE_TYPE");
        }
    }
    else
    {
        // ##############################################################################/
        //------------------ compound cases:
        // ##############################################################################/
        //
        //------------------ (2) Q1 = SE2, Q0 = R2, X1 = SO2
        // ##############################################################################/
        if (Q1->getType() == base::STATE_SPACE_SE2)
        {
            if (Q0->getType() == base::STATE_SPACE_REAL_VECTOR)
            {
                if (Q0->getDimension() == 2)
                {
                    type_ = SE2_R2;
                }
                else
                {
                    OMPL_ERROR("Q1 is SE2 but Q0 type %d is of dimension %d", Q0->getType(), Q0->getDimension());
                    throw ompl::Exception("Invalid dimensions.");
                }
            }
            else
            {
                if (Q0->getType() == base::STATE_SPACE_SE2)
                {
                    type_ = IDENTITY_SPACE_SE2;
                }
                else
                {
                    OMPL_ERROR("Q1 is SE2 but Q0 type %d is not handled.", Q0->getType());
                    throw ompl::Exception("INVALID_STATE_TYPE");
                }
            }
        }
        //------------------ (3) Q1 = SE3, Q0 = R3, X1 = SO3
        // ##############################################################################/
        else if (Q1->getType() == base::STATE_SPACE_SE3)
        {
            if (Q0->getType() == base::STATE_SPACE_REAL_VECTOR)
            {
                if (Q0->getDimension() == 3)
                {
                    type_ = SE3_R3;
                }
                else
                {
                    OMPL_ERROR("Q1 is SE3 but Q0 type %d is of dimension %d.", Q0->getType(), Q0->getDimension());
                    throw ompl::Exception("Invalid dimensions.");
                }
            }
            else
            {
                if (Q0->getType() == base::STATE_SPACE_SE3)
                {
                    type_ = IDENTITY_SPACE_SE3;
                }
                else
                {
                    OMPL_ERROR("Q1 is SE2 but Q0 type %d is not handled.", Q0->getType());
                    throw ompl::Exception("Invalid QuotientSpace type");
                }
                OMPL_ERROR("Q1 is SE3 but Q0 type %d is not handled.", Q0->getType());
                throw ompl::Exception("Invalid QuotientSpace type");
            }
        }
        // ##############################################################################/
        else
        {
            base::CompoundStateSpace *Q1_compound = Q1->as<base::CompoundStateSpace>();
            const std::vector<base::StateSpacePtr> Q1_decomposed = Q1_compound->getSubspaces();
            unsigned int Q1_subspaces = Q1_decomposed.size();
            if (Q1_subspaces == 2)
            {
                if (Q1_decomposed.at(0)->getType() == base::STATE_SPACE_SE3 &&
                    Q1_decomposed.at(1)->getType() == base::STATE_SPACE_REAL_VECTOR)
                {
                    unsigned int n = Q1_decomposed.at(1)->getDimension();
                    if (Q0->getType() == base::STATE_SPACE_SE3)
                    {
                        //------------------ (4) Q1 = SE3xRn, Q0 = SE3, X1 = Rn
                        // ##############################################################################/
                        type_ = SE3RN_SE3;
                    }
                    else if (Q0->getType() == base::STATE_SPACE_REAL_VECTOR)
                    {
                        //------------------ (5) Q1 = SE3xRn, Q0 = R3, X1 = SO3xRN
                        // ##############################################################################/
                        unsigned int m = Q0->getDimension();
                        if (m == 3)
                        {
                            type_ = SE3RN_R3;
                        }
                        else
                        {
                            OMPL_ERROR("Not allowed. Q0 needs to be 3-dimensional but is %d dimensional", m);
                            throw ompl::Exception("Invalid dimensions.");
                        }
                    }
                    else
                    {
                        //------------------ (6) Q1 = SE3xRn, Q0 = SE3xRm, X1 = R(n-m)
                        // ##############################################################################/
                        base::CompoundStateSpace *Q0_compound = Q0->as<base::CompoundStateSpace>();
                        const std::vector<base::StateSpacePtr> Q0_decomposed = Q0_compound->getSubspaces();
                        unsigned int Q0_subspaces = Q0_decomposed.size();
                        if (Q0_subspaces == 2)
                        {
                            if (Q1_decomposed.at(0)->getType() == base::STATE_SPACE_SE3 &&
                                Q1_decomposed.at(1)->getType() == base::STATE_SPACE_REAL_VECTOR)
                            {
                                unsigned int m = Q0_decomposed.at(1)->getDimension();
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
                            OMPL_ERROR("State compound with %d subspaces not handled.", Q0_subspaces);
                            throw ompl::Exception("Invalid QuotientSpace type");
                        }
                    }
                }
                else
                {
                    if (Q1_decomposed.at(0)->getType() == base::STATE_SPACE_SE2 &&
                        Q1_decomposed.at(1)->getType() == base::STATE_SPACE_REAL_VECTOR)
                    {
                        unsigned int n = Q1_decomposed.at(1)->getDimension();
                        if (Q0->getType() == base::STATE_SPACE_SE2)
                        {
                            //------------------ (7) Q1 = SE2xRn, Q0 = SE2, X1 = Rn
                            // ##############################################################################/
                            type_ = SE2RN_SE2;
                        }
                        else if (Q0->getType() == base::STATE_SPACE_REAL_VECTOR)
                        {
                            //------------------ (8) Q1 = SE2xRn, Q0 = R2, X1 = SO2xRN
                            // ##############################################################################/
                            unsigned int m = Q0->getDimension();
                            if (m == 2)
                            {
                                type_ = SE2RN_R2;
                            }
                            else
                            {
                                OMPL_ERROR("Not allowed. Q0 needs to be 2-dimensional but is %d dimensional", m);
                                throw ompl::Exception("Invalid dimensions.");
                            }
                        }
                        else
                        {
                            //------------------ (9) Q1 = SE2xRn, Q0 = SE2xRm, X1 = R(n-m)
                            // ##############################################################################/
                            base::CompoundStateSpace *Q0_compound = Q0->as<base::CompoundStateSpace>();
                            const std::vector<base::StateSpacePtr> Q0_decomposed = Q0_compound->getSubspaces();
                            unsigned int Q0_subspaces = Q0_decomposed.size();
                            if (Q0_subspaces == 2)
                            {
                                if (Q1_decomposed.at(0)->getType() == base::STATE_SPACE_SE2 &&
                                    Q1_decomposed.at(1)->getType() == base::STATE_SPACE_REAL_VECTOR)
                                {
                                    unsigned int m = Q0_decomposed.at(1)->getDimension();
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
                                OMPL_ERROR("QO is compound with %d subspaces, but we only handle 2.", Q0_subspaces);
                                throw ompl::Exception("Invalid QuotientSpace type");
                            }
                        }
                    }
                    else if (Q1_decomposed.at(0)->getType() == base::STATE_SPACE_SO2 &&
                             Q1_decomposed.at(1)->getType() == base::STATE_SPACE_REAL_VECTOR)
                    {
                        if (Q0->getType() == base::STATE_SPACE_SO2)
                        {
                            //------------------ (10) Q1 = SO2xRn, Q0 = SO2, X1 = Rn
                            // ##############################################################################/
                            type_ = SO2RN_SO2;
                        }
                        else
                        {
                            //------------------ (11) Q1 = SO2xRn, Q0 = SO2xRm, X1 = R(n-m)
                            // ##############################################################################/
                            if (Q0->isCompound())
                            {
                                base::CompoundStateSpace *Q0_compound = Q0->as<base::CompoundStateSpace>();
                                const std::vector<base::StateSpacePtr> Q0_decomposed = Q0_compound->getSubspaces();
                                unsigned int Q0_subspaces = Q0_decomposed.size();
                                if (Q0_subspaces == 2)
                                {
                                    if (Q1_decomposed.at(0)->getType() == base::STATE_SPACE_SO2 &&
                                        Q1_decomposed.at(1)->getType() == base::STATE_SPACE_REAL_VECTOR)
                                    {
                                        unsigned int n = Q1_decomposed.at(1)->getDimension();
                                        unsigned int m = Q0_decomposed.at(1)->getDimension();
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
                                        OMPL_ERROR("Cannot project onto type %d.", Q1->getType());
                                        throw ompl::Exception("Invalid QuotientSpace type.");
                                    }
                                }
                                else
                                {
                                    OMPL_ERROR("Q0 has %d subspaces. We can handle only 2.", Q0_subspaces);
                                    throw ompl::Exception("Invalid QuotientSpace type.");
                                }
                            }
                            else
                            {
                                OMPL_ERROR("Cannot project onto type %d.", Q0->getType());
                                throw ompl::Exception("Invalid QuotientSpace type.");
                            }
                        }
                    }
                    else
                    {
                        OMPL_ERROR("State compound %d and %d not recognized.", Q1_decomposed.at(0)->getType(),
                                   Q1_decomposed.at(1)->getType());
                        throw ompl::Exception("Invalid QuotientSpace type.");
                    }
                }
            }
            else
            {
                OMPL_ERROR("Q1 has %d subspaces, but we only support 2.", Q1_subspaces);
                throw ompl::Exception("Invalid QuotientSpace type.");
            }
        }
    }
    return type_;
}

void ompl::geometric::QuotientSpace::mergeStates(const base::State *qQ0, const base::State *qX1, base::State *qQ1) const
{
    // input : qQ0 \in Q0, qX1 \in X1
    // output: qQ1 = qQ0 \circ qX1 \in Q1
    const base::StateSpacePtr Q1_space = Q1->getStateSpace();
    const base::StateSpacePtr X1_space = X1->getStateSpace();
    const base::StateSpacePtr Q0_space = parent_->getSpaceInformation()->getStateSpace();

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
            base::RealVectorStateSpace::StateType *sQ1 = qQ1->as<base::RealVectorStateSpace::StateType>();
            const base::RealVectorStateSpace::StateType *sQ0 = qQ0->as<base::RealVectorStateSpace::StateType>();
            const base::RealVectorStateSpace::StateType *sX1 = qX1->as<base::RealVectorStateSpace::StateType>();

            for (unsigned int k = 0; k < Q0_dimension_; k++)
            {
                sQ1->values[k] = sQ0->values[k];
            }
            for (unsigned int k = Q0_dimension_; k < Q1_dimension_; k++)
            {
                sQ1->values[k] = sX1->values[k - Q0_dimension_];
            }
            break;
        }
        case SE2_R2:
        {
            base::SE2StateSpace::StateType *sQ1 = qQ1->as<base::SE2StateSpace::StateType>();
            const base::RealVectorStateSpace::StateType *sQ0 = qQ0->as<base::RealVectorStateSpace::StateType>();
            const base::SO2StateSpace::StateType *sX1 = qX1->as<base::SO2StateSpace::StateType>();

            sQ1->setXY(sQ0->values[0], sQ0->values[1]);
            sQ1->setYaw(sX1->value);

            break;
        }
        case SE3_R3:
        {
            base::SE3StateSpace::StateType *sQ1 = qQ1->as<base::SE3StateSpace::StateType>();
            base::SO3StateSpace::StateType *sQ1_rotation = &sQ1->rotation();

            const base::RealVectorStateSpace::StateType *sQ0 = qQ0->as<base::RealVectorStateSpace::StateType>();
            const base::SO3StateSpace::StateType *sX1 = qX1->as<base::SO3StateSpace::StateType>();

            sQ1->setXYZ(sQ0->values[0], sQ0->values[1], sQ0->values[2]);

            sQ1_rotation->x = sX1->x;
            sQ1_rotation->y = sX1->y;
            sQ1_rotation->z = sX1->z;
            sQ1_rotation->w = sX1->w;

            break;
        }
        case SE3RN_R3:
        {
            base::SE3StateSpace::StateType *sQ1_SE3 =
                qQ1->as<base::CompoundState>()->as<base::SE3StateSpace::StateType>(0);
            base::SO3StateSpace::StateType *sQ1_SO3 = &sQ1_SE3->rotation();
            base::RealVectorStateSpace::StateType *sQ1_RN =
                qQ1->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

            const base::RealVectorStateSpace::StateType *sQ0 = qQ0->as<base::RealVectorStateSpace::StateType>();
            const base::SO3StateSpace::StateType *sX1_SO3 =
                qX1->as<base::CompoundState>()->as<base::SO3StateSpace::StateType>(0);
            const base::RealVectorStateSpace::StateType *sX1_RN =
                qX1->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

            sQ1_SE3->setXYZ(sQ0->values[0], sQ0->values[1], sQ0->values[2]);
            sQ1_SO3->x = sX1_SO3->x;
            sQ1_SO3->y = sX1_SO3->y;
            sQ1_SO3->z = sX1_SO3->z;
            sQ1_SO3->w = sX1_SO3->w;

            for (unsigned int k = 0; k < X1_dimension_ - 3; k++)
            {
                sQ1_RN->values[k] = sX1_RN->values[k];
            }

            break;
        }
        case SE2RN_SE2:
        {
            base::SE2StateSpace::StateType *sQ1_SE2 =
                qQ1->as<base::CompoundState>()->as<base::SE2StateSpace::StateType>(0);
            base::RealVectorStateSpace::StateType *sQ1_RN =
                qQ1->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

            const base::SE2StateSpace::StateType *sQ0 = qQ0->as<base::SE2StateSpace::StateType>();
            const base::RealVectorStateSpace::StateType *sX1 = qX1->as<base::RealVectorStateSpace::StateType>();

            sQ1_SE2->setX(sQ0->getX());
            sQ1_SE2->setY(sQ0->getY());
            sQ1_SE2->setYaw(sQ0->getYaw());

            for (unsigned int k = 0; k < X1_dimension_; k++)
            {
                sQ1_RN->values[k] = sX1->values[k];
            }
            break;
        }
        case SO2RN_SO2:
        {
            base::SO2StateSpace::StateType *sQ1_SO2 =
                qQ1->as<base::CompoundState>()->as<base::SO2StateSpace::StateType>(0);
            base::RealVectorStateSpace::StateType *sQ1_RN =
                qQ1->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

            const base::SO2StateSpace::StateType *sQ0 = qQ0->as<base::SO2StateSpace::StateType>();
            const base::RealVectorStateSpace::StateType *sX1 = qX1->as<base::RealVectorStateSpace::StateType>();

            sQ1_SO2->value = sQ0->value;

            for (unsigned int k = 0; k < X1_dimension_; k++)
            {
                sQ1_RN->values[k] = sX1->values[k];
            }
            break;
        }
        case SO2RN_SO2RM:
        {
            base::SO2StateSpace::StateType *sQ1_SO2 =
                qQ1->as<base::CompoundState>()->as<base::SO2StateSpace::StateType>(0);
            base::RealVectorStateSpace::StateType *sQ1_RN =
                qQ1->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

            const base::SO2StateSpace::StateType *sQ0_SO2 =
                qQ0->as<base::CompoundState>()->as<base::SO2StateSpace::StateType>(0);
            const base::RealVectorStateSpace::StateType *sQ0_RM =
                qQ0->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

            const base::RealVectorStateSpace::StateType *sX1 = qX1->as<base::RealVectorStateSpace::StateType>();

            sQ1_SO2->value = sQ0_SO2->value;

            unsigned int M = Q1_dimension_ - X1_dimension_ - 1;
            unsigned int N = X1_dimension_;

            for (unsigned int k = 0; k < M; k++)
            {
                sQ1_RN->values[k] = sQ0_RM->values[k];
            }
            for (unsigned int k = M; k < M + N; k++)
            {
                sQ1_RN->values[k] = sX1->values[k - M];
            }
            break;
        }

        case SE2RN_R2:
        {
            base::SE2StateSpace::StateType *sQ1_SE2 =
                qQ1->as<base::CompoundState>()->as<base::SE2StateSpace::StateType>(0);
            base::RealVectorStateSpace::StateType *sQ1_RN =
                qQ1->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

            const base::RealVectorStateSpace::StateType *sQ0 = qQ0->as<base::RealVectorStateSpace::StateType>();
            const base::SO2StateSpace::StateType *sX1_SO2 =
                qX1->as<base::CompoundState>()->as<base::SO2StateSpace::StateType>(0);
            const base::RealVectorStateSpace::StateType *sX1_RN =
                qX1->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

            sQ1_SE2->setX(sQ0->values[0]);
            sQ1_SE2->setY(sQ0->values[1]);
            sQ1_SE2->setYaw(sX1_SO2->value);

            for (unsigned int k = 0; k < X1_dimension_ - 1; k++)
            {
                sQ1_RN->values[k] = sX1_RN->values[k];
            }
            break;
        }
        case SE2RN_SE2RM:
        {
            base::SE2StateSpace::StateType *sQ1_SE2 =
                qQ1->as<base::CompoundState>()->as<base::SE2StateSpace::StateType>(0);
            base::RealVectorStateSpace::StateType *sQ1_RN =
                qQ1->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

            const base::SE2StateSpace::StateType *sQ0_SE2 =
                qQ0->as<base::CompoundState>()->as<base::SE2StateSpace::StateType>(0);
            const base::RealVectorStateSpace::StateType *sQ0_RM =
                qQ0->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

            const base::RealVectorStateSpace::StateType *sX1 = qX1->as<base::RealVectorStateSpace::StateType>();

            sQ1_SE2->setX(sQ0_SE2->getX());
            sQ1_SE2->setY(sQ0_SE2->getY());
            sQ1_SE2->setYaw(sQ0_SE2->getYaw());

            //[X Y YAW] [1...M-1][M...N-1]
            // SE2               RN
            unsigned int M = Q1_dimension_ - X1_dimension_ - 3;
            unsigned int N = X1_dimension_;

            for (unsigned int k = 0; k < M; k++)
            {
                sQ1_RN->values[k] = sQ0_RM->values[k];
            }
            for (unsigned int k = M; k < M + N; k++)
            {
                sQ1_RN->values[k] = sX1->values[k - M];
            }
            break;
        }
        case SE3RN_SE3:
        {
            base::SE3StateSpace::StateType *sQ1_SE3 =
                qQ1->as<base::CompoundState>()->as<base::SE3StateSpace::StateType>(0);
            base::SO3StateSpace::StateType *sQ1_SE3_rotation = &sQ1_SE3->rotation();
            base::RealVectorStateSpace::StateType *sQ1_RN =
                qQ1->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

            const base::SE3StateSpace::StateType *sQ0 = qQ0->as<base::SE3StateSpace::StateType>();
            const base::SO3StateSpace::StateType *sQ0_rotation = &sQ0->rotation();
            const base::RealVectorStateSpace::StateType *sX1 = qX1->as<base::RealVectorStateSpace::StateType>();

            sQ1_SE3->setXYZ(sQ0->getX(), sQ0->getY(), sQ0->getZ());
            sQ1_SE3_rotation->x = sQ0_rotation->x;
            sQ1_SE3_rotation->y = sQ0_rotation->y;
            sQ1_SE3_rotation->z = sQ0_rotation->z;
            sQ1_SE3_rotation->w = sQ0_rotation->w;

            for (unsigned int k = 0; k < X1_dimension_; k++)
            {
                sQ1_RN->values[k] = sX1->values[k];
            }

            break;
        }
        case SE3RN_SE3RM:
        {
            base::SE3StateSpace::StateType *sQ1_SE3 =
                qQ1->as<base::CompoundState>()->as<base::SE3StateSpace::StateType>(0);
            base::SO3StateSpace::StateType *sQ1_SE3_rotation = &sQ1_SE3->rotation();
            base::RealVectorStateSpace::StateType *sQ1_RN =
                qQ1->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

            const base::SE3StateSpace::StateType *sQ0_SE3 =
                qQ0->as<base::CompoundState>()->as<base::SE3StateSpace::StateType>(0);
            const base::SO3StateSpace::StateType *sQ0_SE3_rotation = &sQ0_SE3->rotation();
            const base::RealVectorStateSpace::StateType *sQ0_RM =
                qQ0->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

            const base::RealVectorStateSpace::StateType *sX1 = qX1->as<base::RealVectorStateSpace::StateType>();

            sQ1_SE3->setXYZ(sQ0_SE3->getX(), sQ0_SE3->getY(), sQ0_SE3->getZ());
            sQ1_SE3_rotation->x = sQ0_SE3_rotation->x;
            sQ1_SE3_rotation->y = sQ0_SE3_rotation->y;
            sQ1_SE3_rotation->z = sQ0_SE3_rotation->z;
            sQ1_SE3_rotation->w = sQ0_SE3_rotation->w;

            //[X Y Z YAW PITCH ROLL] [1...M-1][M...N-1]
            // SE3                                        RN
            unsigned int M = Q1_dimension_ - X1_dimension_ - 6;
            unsigned int N = X1_dimension_;

            for (unsigned int k = 0; k < M; k++)
            {
                sQ1_RN->values[k] = sQ0_RM->values[k];
            }
            for (unsigned int k = M; k < M + N; k++)
            {
                sQ1_RN->values[k] = sX1->values[k - M];
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

void ompl::geometric::QuotientSpace::projectX1(const base::State *q, base::State *qX1) const
{
    switch (type_)
    {
        case RN_RM:
        {
            const base::RealVectorStateSpace::StateType *sQ1 = q->as<base::RealVectorStateSpace::StateType>();
            base::RealVectorStateSpace::StateType *sX1 = qX1->as<base::RealVectorStateSpace::StateType>();

            for (unsigned int k = Q0_dimension_; k < Q1_dimension_; k++)
            {
                sX1->values[k - Q0_dimension_] = sQ1->values[k];
            }
            break;
        }
        case SE2_R2:
        {
            const base::SE2StateSpace::StateType *sQ1 = q->as<base::SE2StateSpace::StateType>();
            base::SO2StateSpace::StateType *sX1 = qX1->as<base::SO2StateSpace::StateType>();
            sX1->value = sQ1->getYaw();
            break;
        }
        case SE3_R3:
        {
            const base::SE3StateSpace::StateType *sQ1 = q->as<base::SE3StateSpace::StateType>();
            const base::SO3StateSpace::StateType *sQ1_SO3 = &sQ1->rotation();

            base::SO3StateSpace::StateType *sX1_SO3 = qX1->as<base::SO3StateSpace::StateType>();

            sX1_SO3->x = sQ1_SO3->x;
            sX1_SO3->y = sQ1_SO3->y;
            sX1_SO3->z = sQ1_SO3->z;
            sX1_SO3->w = sQ1_SO3->w;

            break;
        }
        case SE3RN_R3:
        {
            const base::SE3StateSpace::StateType *sQ1_SE3 =
                q->as<base::CompoundState>()->as<base::SE3StateSpace::StateType>(0);
            const base::SO3StateSpace::StateType *sQ1_SO3 = &sQ1_SE3->rotation();
            const base::RealVectorStateSpace::StateType *sQ1_RN =
                q->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

            base::SO3StateSpace::StateType *sX1_SO3 =
                qX1->as<base::CompoundState>()->as<base::SO3StateSpace::StateType>(0);
            base::RealVectorStateSpace::StateType *sX1_RN =
                qX1->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

            sX1_SO3->x = sQ1_SO3->x;
            sX1_SO3->y = sQ1_SO3->y;
            sX1_SO3->z = sQ1_SO3->z;
            sX1_SO3->w = sQ1_SO3->w;
            for (unsigned int k = 0; k < X1_dimension_ - 3; k++)
            {
                sX1_RN->values[k] = sQ1_RN->values[k];
            }

            break;
        }
        case SE2RN_R2:
        {
            const base::SE2StateSpace::StateType *sQ1_SE2 =
                q->as<base::CompoundState>()->as<base::SE2StateSpace::StateType>(0);
            const base::RealVectorStateSpace::StateType *sQ1_RN =
                q->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

            base::SO2StateSpace::StateType *sX1_SO2 =
                qX1->as<base::CompoundState>()->as<base::SO2StateSpace::StateType>(0);
            base::RealVectorStateSpace::StateType *sX1_RN =
                qX1->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

            sX1_SO2->value = sQ1_SE2->getYaw();
            for (unsigned int k = 0; k < X1_dimension_ - 1; k++)
            {
                sX1_RN->values[k] = sQ1_RN->values[k];
            }
            break;
        }
        case SE2RN_SE2RM:
        case SO2RN_SO2RM:
        {
            const base::RealVectorStateSpace::StateType *sQ1_RN =
                q->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

            const base::RealVectorStateSpace::StateType *sX1 = qX1->as<base::RealVectorStateSpace::StateType>();

            unsigned int N = Q1_dimension_ - X1_dimension_ - 3;
            for (unsigned int k = N; k < Q1_dimension_ - 3; k++)
            {
                sX1->values[k - N] = sQ1_RN->values[k];
            }
            break;
        }
        case SE2RN_SE2:
        case SE3RN_SE3:
        case SO2RN_SO2:
        {
            const base::RealVectorStateSpace::StateType *sQ1_RN =
                q->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);
            base::RealVectorStateSpace::StateType *sX1 = qX1->as<base::RealVectorStateSpace::StateType>();

            for (unsigned int k = 0; k < X1_dimension_; k++)
            {
                sX1->values[k] = sQ1_RN->values[k];
            }

            break;
        }
        case SE3RN_SE3RM:
        {
            const base::RealVectorStateSpace::StateType *sQ1_RN =
                q->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

            const base::RealVectorStateSpace::StateType *sX1 = qX1->as<base::RealVectorStateSpace::StateType>();

            unsigned int N = Q1_dimension_ - X1_dimension_ - 6;
            for (unsigned int k = N; k < Q1_dimension_ - 6; k++)
            {
                sX1->values[k - N] = sQ1_RN->values[k];
            }
            break;
        }
        default:
        {
            OMPL_ERROR("Type %d not implemented.", type_);
            throw ompl::Exception("Cannot project onto X1.");
        }
    }
}

void ompl::geometric::QuotientSpace::projectQ0(const base::State *q, base::State *qQ0) const
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
            Q1->getStateSpace()->copyState(qQ0, q);
            break;
        }
        case RN_RM:
        {
            const base::RealVectorStateSpace::StateType *sQ1 = q->as<base::RealVectorStateSpace::StateType>();
            base::RealVectorStateSpace::StateType *sQ0 = qQ0->as<base::RealVectorStateSpace::StateType>();

            for (unsigned int k = 0; k < Q0_dimension_; k++)
            {
                sQ0->values[k] = sQ1->values[k];
            }
            break;
        }
        case SE2_R2:
        {
            const base::SE2StateSpace::StateType *sQ1 = q->as<base::SE2StateSpace::StateType>();
            base::RealVectorStateSpace::StateType *sQ0 = qQ0->as<base::RealVectorStateSpace::StateType>();
            sQ0->values[0] = sQ1->getX();
            sQ0->values[1] = sQ1->getY();
            break;
        }
        case SE2RN_R2:
        {
            const base::SE2StateSpace::StateType *sQ1 =
                q->as<base::CompoundState>()->as<base::SE2StateSpace::StateType>(0);
            base::RealVectorStateSpace::StateType *sQ0 = qQ0->as<base::RealVectorStateSpace::StateType>();
            sQ0->values[0] = sQ1->getX();
            sQ0->values[1] = sQ1->getY();
            break;
        }
        case SE2RN_SE2:
        {
            const base::SE2StateSpace::StateType *sQ1_SE2 =
                q->as<base::CompoundState>()->as<base::SE2StateSpace::StateType>(0);
            base::SE2StateSpace::StateType *sQ0_SE2 = qQ0->as<base::SE2StateSpace::StateType>();

            sQ0_SE2->setX(sQ1_SE2->getX());
            sQ0_SE2->setY(sQ1_SE2->getY());
            sQ0_SE2->setYaw(sQ1_SE2->getYaw());

            break;
        }
        case SO2RN_SO2:
        {
            const base::SO2StateSpace::StateType *sQ1_SO2 =
                q->as<base::CompoundState>()->as<base::SO2StateSpace::StateType>(0);
            base::SO2StateSpace::StateType *sQ0_SO2 = qQ0->as<base::SO2StateSpace::StateType>();

            sQ0_SO2->value = sQ1_SO2->value;

            break;
        }
        case SO2RN_SO2RM:
        {
            const base::SO2StateSpace::StateType *sQ1_SO2 =
                q->as<base::CompoundState>()->as<base::SO2StateSpace::StateType>(0);
            const base::RealVectorStateSpace::StateType *sQ1_RN =
                q->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

            base::SO2StateSpace::StateType *sQ0_SO2 =
                qQ0->as<base::CompoundState>()->as<base::SO2StateSpace::StateType>(0);
            base::RealVectorStateSpace::StateType *sQ0_RM =
                qQ0->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

            sQ0_SO2->value = sQ1_SO2->value;

            for (unsigned int k = 0; k < Q0_dimension_ - 1; k++)
            {
                sQ0_RM->values[k] = sQ1_RN->values[k];
            }
            break;
        }

        case SE2RN_SE2RM:
        {
            const base::SE2StateSpace::StateType *sQ1_SE2 =
                q->as<base::CompoundState>()->as<base::SE2StateSpace::StateType>(0);
            const base::RealVectorStateSpace::StateType *sQ1_RN =
                q->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

            base::SE2StateSpace::StateType *sQ0_SE2 =
                qQ0->as<base::CompoundState>()->as<base::SE2StateSpace::StateType>(0);
            base::RealVectorStateSpace::StateType *sQ0_RN =
                qQ0->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

            sQ0_SE2->setX(sQ1_SE2->getX());
            sQ0_SE2->setY(sQ1_SE2->getY());
            sQ0_SE2->setYaw(sQ1_SE2->getYaw());

            for (unsigned int k = 0; k < Q0_dimension_ - 3; k++)
            {
                sQ0_RN->values[k] = sQ1_RN->values[k];
            }
            break;
        }
        case SE3_R3:
        {
            const base::SE3StateSpace::StateType *sQ1 = q->as<base::SE3StateSpace::StateType>();
            base::RealVectorStateSpace::StateType *sQ0 = qQ0->as<base::RealVectorStateSpace::StateType>();

            sQ0->values[0] = sQ1->getX();
            sQ0->values[1] = sQ1->getY();
            sQ0->values[2] = sQ1->getZ();

            break;
        }
        case SE3RN_R3:
        {
            const base::SE3StateSpace::StateType *sQ1_SE3 =
                q->as<base::CompoundState>()->as<base::SE3StateSpace::StateType>(0);
            base::RealVectorStateSpace::StateType *sQ0 = qQ0->as<base::RealVectorStateSpace::StateType>();

            sQ0->values[0] = sQ1_SE3->getX();
            sQ0->values[1] = sQ1_SE3->getY();
            sQ0->values[2] = sQ1_SE3->getZ();

            break;
        }
        case SE3RN_SE3:
        {
            const base::SE3StateSpace::StateType *sQ1_SE3 =
                q->as<base::CompoundState>()->as<base::SE3StateSpace::StateType>(0);
            const base::SO3StateSpace::StateType *sQ1_SE3_rotation = &sQ1_SE3->rotation();

            base::SE3StateSpace::StateType *sQ0 = qQ0->as<base::SE3StateSpace::StateType>();
            base::SO3StateSpace::StateType *sQ0_rotation = &sQ0->rotation();

            sQ0->setXYZ(sQ1_SE3->getX(), sQ1_SE3->getY(), sQ1_SE3->getZ());
            sQ0_rotation->x = sQ1_SE3_rotation->x;
            sQ0_rotation->y = sQ1_SE3_rotation->y;
            sQ0_rotation->z = sQ1_SE3_rotation->z;
            sQ0_rotation->w = sQ1_SE3_rotation->w;

            break;
        }
        case SE3RN_SE3RM:
        {
            const base::SE3StateSpace::StateType *sQ1_SE3 =
                q->as<base::CompoundState>()->as<base::SE3StateSpace::StateType>(0);
            const base::SO3StateSpace::StateType *sQ1_SE3_rotation = &sQ1_SE3->rotation();
            const base::RealVectorStateSpace::StateType *sQ1_RN =
                q->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

            base::SE3StateSpace::StateType *sQ0_SE3 =
                qQ0->as<base::CompoundState>()->as<base::SE3StateSpace::StateType>(0);
            base::SO3StateSpace::StateType *sQ0_rotation = &sQ0_SE3->rotation();
            base::RealVectorStateSpace::StateType *sQ0_RN =
                qQ0->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

            sQ0_SE3->setXYZ(sQ1_SE3->getX(), sQ1_SE3->getY(), sQ1_SE3->getZ());
            sQ0_rotation->x = sQ1_SE3_rotation->x;
            sQ0_rotation->y = sQ1_SE3_rotation->y;
            sQ0_rotation->z = sQ1_SE3_rotation->z;
            sQ0_rotation->w = sQ1_SE3_rotation->w;

            for (unsigned int k = 0; k < Q0_dimension_ - 6; k++)
            {
                sQ0_RN->values[k] = sQ1_RN->values[k];
            }
            break;
        }
        default:
        {
            OMPL_ERROR("Cannot project onto Q0. Type %d not implemented.", type_);
            throw ompl::Exception("Cannot project onto Q0.");
        }
    }
}

const ompl::base::SpaceInformationPtr &ompl::geometric::QuotientSpace::getX1() const
{
    return X1;
}

const ompl::base::SpaceInformationPtr &ompl::geometric::QuotientSpace::getQ1() const
{
    return Q1;
}

const ompl::base::SpaceInformationPtr &ompl::geometric::QuotientSpace::getQ0() const
{
    return Q0;
}

unsigned int ompl::geometric::QuotientSpace::getX1Dimension() const
{
    return X1_dimension_;
}

unsigned int ompl::geometric::QuotientSpace::getQ1Dimension() const
{
    return Q1->getStateDimension();
}

unsigned int ompl::geometric::QuotientSpace::getDimension() const
{
    return getQ1Dimension();
}

unsigned int ompl::geometric::QuotientSpace::getQ0Dimension() const
{
    return Q0_dimension_;
}

const ompl::base::StateSamplerPtr &ompl::geometric::QuotientSpace::getX1SamplerPtr() const
{
    return X1_sampler_;
}

const ompl::base::StateSamplerPtr &ompl::geometric::QuotientSpace::getQ1SamplerPtr() const
{
    return Q1_sampler_;
}

bool ompl::geometric::QuotientSpace::hasSolution()
{
    if (!hasSolution_)
    {
        base::PathPtr path;
        hasSolution_ = getSolution(path);
    }
    return hasSolution_;
}

unsigned int ompl::geometric::QuotientSpace::getTotalNumberOfSamples() const
{
    return totalNumberOfSamples_;
}

unsigned int ompl::geometric::QuotientSpace::getTotalNumberOfFeasibleSamples() const
{
    return totalNumberOfFeasibleSamples_;
}

ompl::geometric::QuotientSpace *ompl::geometric::QuotientSpace::getParent() const
{
    return parent_;
}

ompl::geometric::QuotientSpace *ompl::geometric::QuotientSpace::getChild() const
{
    return child_;
}

void ompl::geometric::QuotientSpace::setChild(ompl::geometric::QuotientSpace *child)
{
    child_ = child;
}

void ompl::geometric::QuotientSpace::setParent(ompl::geometric::QuotientSpace *parent)
{
    parent_ = parent;
}

unsigned int ompl::geometric::QuotientSpace::getLevel() const
{
    return level_;
}

void ompl::geometric::QuotientSpace::setLevel(unsigned int level)
{
    level_ = level;
}

ompl::geometric::QuotientSpace::QuotientSpaceType ompl::geometric::QuotientSpace::getType() const
{
    return type_;
}

ompl::base::OptimizationObjectivePtr ompl::geometric::QuotientSpace::getOptimizationObjectivePtr() const
{
    return opt_;
}

bool ompl::geometric::QuotientSpace::sampleQuotient(base::State *q_random)
{
    Q1_sampler_->sampleUniform(q_random);
    return true;
}

bool ompl::geometric::QuotientSpace::sample(base::State *q_random)
{
    bool valid = false;
    if (parent_ == nullptr)
    {
        // return Q1_valid_sampler->sample(q_random);
        Q1_sampler_->sampleUniform(q_random);
        valid = Q1->isValid(q_random);
    }
    else
    {
        if (X1_dimension_ > 0)
        {
            // Adjusted sampling function: Sampling in G0 x X1
            X1_sampler_->sampleUniform(s_X1_tmp_);
            parent_->sampleQuotient(s_Q0_tmp_);
            mergeStates(s_Q0_tmp_, s_X1_tmp_, q_random);
        }
        else
        {
            parent_->sampleQuotient(q_random);
        }
        valid = Q1->isValid(q_random);
    }
    totalNumberOfSamples_++;
    if (valid)
    {
        totalNumberOfFeasibleSamples_++;
    }

    return valid;
}

double ompl::geometric::QuotientSpace::getImportance() const
{
    double N = (double)totalNumberOfSamples_;
    return 1.0 / (N + 1);
}

void ompl::geometric::QuotientSpace::print(std::ostream &out) const
{
    out << "[QuotientSpace: id" << id_ << " |lvl" << level_ << "] ";
    unsigned int sublevel = std::max(1U, level_);
    if (parent_ == nullptr)
    {
        out << "X" << sublevel << "=Q" << sublevel << ": ";
        if (Q1->getStateSpace()->getType() == base::STATE_SPACE_SE2)
        {
            out << "SE(2)";
        }
        else if (Q1->getStateSpace()->getType() == base::STATE_SPACE_SE3)
        {
            out << "SE(3)";
        }
        else if (Q1->getStateSpace()->getType() == base::STATE_SPACE_REAL_VECTOR)
        {
            out << "R^" << Q1->getStateDimension();
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
            case QuotientSpace::IDENTITY_SPACE_RN:
            {
                out << "R^" << Q0_dimension_ << " | Q" << level_ + 1 << ": R^" << Q1_dimension_;
                break;
            }
            case QuotientSpace::IDENTITY_SPACE_SE2:
            {
                out << "SE(2)"
                    << " | Q" << level_ + 1 << ": SE(2)";
                break;
            }
            case QuotientSpace::IDENTITY_SPACE_SE2RN:
            {
                out << "SE(2)xR^" << Q0_dimension_ << " | Q" << level_ + 1 << ": SE(2)xR^" << Q1_dimension_;
                break;
            }
            case QuotientSpace::IDENTITY_SPACE_SO2RN:
            {
                out << "SO(2)xR^" << Q0_dimension_ << " | Q" << level_ + 1 << ": SO(2)xR^" << Q1_dimension_;
                break;
            }
            case QuotientSpace::IDENTITY_SPACE_SE3:
            {
                out << "SE(3)"
                    << " | Q" << level_ + 1 << ": SE(3)";
                break;
            }
            case QuotientSpace::IDENTITY_SPACE_SE3RN:
            {
                out << "SE(3)xR^" << Q0_dimension_ << " | Q" << level_ + 1 << ": SE(3)xR^" << Q1_dimension_;
                break;
            }
            case QuotientSpace::RN_RM:
            {
                out << "R^" << Q0_dimension_ << " | Q" << level_ + 1 << ": R^" << Q1_dimension_ << " | X" << level_ + 1
                    << ": R^" << Q1_dimension_ - Q0_dimension_;
                break;
            }
            case QuotientSpace::SE2_R2:
            {
                out << "R^2 | Q" << level_ + 1 << ": SE(2) | X" << level_ + 1 << ": SO(2)";
                break;
            }
            case QuotientSpace::SE3_R3:
            {
                out << "R^3 | Q" << level_ + 1 << ": SE(3) | X" << level_ + 1 << ": SO(3)";
                break;
            }
            case QuotientSpace::SE2RN_SE2:
            {
                out << "SE(2) | Q" << level_ + 1 << ": SE(2)xR^" << X1_dimension_ << " | X" << level_ + 1 << ": R^"
                    << X1_dimension_;
                break;
            }
            case QuotientSpace::SO2RN_SO2:
            {
                out << "SO(2) | Q" << level_ + 1 << ": SO(2)xR^" << X1_dimension_ << " | X" << level_ + 1 << ": R^"
                    << X1_dimension_;
                break;
            }
            case QuotientSpace::SE3RN_SE3:
            {
                out << "SE(3) | Q" << level_ + 1 << ": SE(3)xR^" << X1_dimension_ << " | X" << level_ + 1 << ": R^"
                    << X1_dimension_;
                break;
            }
            case QuotientSpace::SE2RN_SE2RM:
            {
                out << "SE(2)xR^" << Q0_dimension_ - 3 << " | Q" << level_ + 1 << ": SE(2)xR^" << Q1_dimension_ - 3
                    << " | X" << level_ + 1 << ": R^" << X1_dimension_;
                break;
            }
            case QuotientSpace::SO2RN_SO2RM:
            {
                out << "SO(2)xR^" << Q0_dimension_ - 1 << " | Q" << level_ + 1 << ": SO(2)xR^" << Q1_dimension_ - 1
                    << " | X" << level_ + 1 << ": R^" << X1_dimension_;
                break;
            }
            case QuotientSpace::SE3RN_SE3RM:
            {
                out << "SE(3)xR^" << Q0_dimension_ - 6 << " | Q" << level_ + 1 << ": SE(3)xR^" << Q1_dimension_ - 6
                    << " | X" << level_ + 1 << ": R^" << X1_dimension_;
                break;
            }
            default:
            {
                out << "unknown type_: " << type_;
            }
        }
    }
    out << " [Importance:" << getImportance() << "]";
}

namespace ompl
{
    namespace geometric
    {
        std::ostream &operator<<(std::ostream &out, const QuotientSpace &quotient_)
        {
            quotient_.print(out);
            return out;
        }
    }  // namespace geometric
}  // namespace ompl
