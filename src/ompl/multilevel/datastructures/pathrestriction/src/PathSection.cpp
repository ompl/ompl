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

#include <ompl/multilevel/datastructures/pathrestriction/PathSection.h>
#include <ompl/multilevel/datastructures/pathrestriction/PathRestriction.h>
#include <ompl/multilevel/datastructures/pathrestriction/Head.h>
#include <ompl/multilevel/datastructures/projections/FiberedProjection.h>

using namespace ompl::multilevel;

PathSection::PathSection(PathRestriction *restriction) : restriction_(restriction)
{
    BundleSpaceGraph *graph = restriction_->getBundleSpaceGraph();
    FiberedProjectionPtr projection = std::static_pointer_cast<FiberedProjection>(graph->getProjection());
    if (graph->getCoDimension() > 0)
    {
        base::StateSpacePtr fiber = projection->getFiberSpace();
        xFiberStart_ = fiber->allocState();
        xFiberGoal_ = fiber->allocState();
        xFiberTmp_ = fiber->allocState();
    }
    if (graph->getBaseDimension() > 0)
    {
        base::SpaceInformationPtr base = graph->getBase();
        xBaseTmp_ = base->allocState();
    }
    base::SpaceInformationPtr bundle = graph->getBundle();
    xBundleTmp_ = bundle->allocState();
    lastValid_.first = bundle->allocState();
}

PathSection::~PathSection()
{
    BundleSpaceGraph *graph = restriction_->getBundleSpaceGraph();
    base::SpaceInformationPtr bundle = graph->getBundle();

    if (graph->getCoDimension() > 0)
    {
        FiberedProjectionPtr projection = std::static_pointer_cast<FiberedProjection>(graph->getProjection());
        base::StateSpacePtr fiber = projection->getFiberSpace();
        fiber->freeState(xFiberStart_);
        fiber->freeState(xFiberGoal_);
        fiber->freeState(xFiberTmp_);
    }
    if (graph->getBaseDimension() > 0)
    {
        base::SpaceInformationPtr base = graph->getBase();
        base->freeState(xBaseTmp_);
    }

    bundle->freeStates(section_);
    bundle->freeState(lastValid_.first);
    bundle->freeState(xBundleTmp_);
}

bool PathSection::checkMotion(HeadPtr &head)
{
    BundleSpaceGraph *graph = restriction_->getBundleSpaceGraph();

    base::SpaceInformationPtr bundle = graph->getBundle();
    base::SpaceInformationPtr base = graph->getBase();

    for (unsigned int k = 1; k < section_.size(); k++)
    {
        if (bundle->checkMotion(head->getState(), section_.at(k), lastValid_))
        {
            if (k < section_.size() - 1)
            {
                Configuration *xLast = addFeasibleSegment(head->getConfiguration(), section_.at(k));

                double locationOnBasePath = restriction_->getLengthBasePathUntil(sectionBaseStateIndices_.at(k));

                head->setCurrent(xLast, locationOnBasePath);
            }
            else
            {
                addFeasibleGoalSegment(head->getConfiguration(), head->getTargetConfiguration());
                return true;
            }
        }
        else
        {
            lastValidIndexOnBasePath_ = sectionBaseStateIndices_.at(k - 1);

            base::State *lastValidBaseState = restriction_->getBasePath().at(lastValidIndexOnBasePath_);

            graph->project(lastValid_.first, xBaseTmp_);

            double distBaseSegment = base->distance(lastValidBaseState, xBaseTmp_);

            double locationOnBasePath =
                restriction_->getLengthBasePathUntil(lastValidIndexOnBasePath_) + distBaseSegment;

            //############################################################################
            // Get Last valid
            //############################################################################
            if (lastValid_.second > 0)
            {
                // add last valid into the bundle graph
                Configuration *xBundleLastValid = new Configuration(bundle, lastValid_.first);
                graph->addConfiguration(xBundleLastValid);
                graph->addBundleEdge(head->getConfiguration(), xBundleLastValid);

                head->setCurrent(xBundleLastValid, locationOnBasePath);
            }
            return false;
        }
    }
    return false;
}

ompl::base::State *PathSection::at(int k) const
{
    return section_.at(k);
}

const ompl::base::State *PathSection::back() const
{
    return section_.back();
}

const ompl::base::State *PathSection::front() const
{
    return section_.front();
}

void PathSection::interpolateL1FiberFirst(HeadPtr &head)
{
    section_.clear();
    sectionBaseStateIndices_.clear();

    BundleSpaceGraph *graph = restriction_->getBundleSpaceGraph();
    base::SpaceInformationPtr base = graph->getBase();
    base::SpaceInformationPtr bundle = graph->getBundle();

    int size = head->getNumberOfRemainingStates() + 1;

    FiberedProjectionPtr projection = std::static_pointer_cast<FiberedProjection>(graph->getProjection());

    if (graph->getCoDimension() > 0)
    {
        const base::State *xFiberStart = head->getStateFiber();
        const base::State *xFiberGoal = head->getStateTargetFiber();

        section_.resize(size + 1);

        bundle->allocStates(section_);

        projection->lift(head->getBaseStateAt(0), xFiberStart, section_.front());

        sectionBaseStateIndices_.push_back(head->getBaseStateIndexAt(0));

        for (unsigned int k = 1; k < section_.size(); k++)
        {
            projection->lift(head->getBaseStateAt(k - 1), xFiberGoal, section_.at(k));
            sectionBaseStateIndices_.push_back(head->getBaseStateIndexAt(k - 1));
        }
    }
    else
    {
        section_.resize(size);

        bundle->allocStates(section_);

        for (int k = 0; k < size; k++)
        {
            bundle->copyState(section_.at(k), head->getBaseStateAt(k));
            sectionBaseStateIndices_.push_back(head->getBaseStateIndexAt(k));
        }
    }
}

void PathSection::interpolateL1FiberLast(HeadPtr &head)
{
    section_.clear();
    sectionBaseStateIndices_.clear();

    BundleSpaceGraph *graph = restriction_->getBundleSpaceGraph();
    const base::SpaceInformationPtr bundle = graph->getBundle();
    const base::SpaceInformationPtr base = graph->getBase();

    int size = head->getNumberOfRemainingStates() + 1;

    FiberedProjectionPtr projection = std::static_pointer_cast<FiberedProjection>(graph->getProjection());
    if (graph->getCoDimension() > 0)
    {
        const base::State *xFiberStart = head->getStateFiber();
        const base::State *xFiberGoal = head->getStateTargetFiber();

        section_.resize(size + 1);

        bundle->allocStates(section_);

        for (int k = 0; k < size; k++)
        {
            projection->lift(head->getBaseStateAt(k), xFiberStart, section_.at(k));
            sectionBaseStateIndices_.push_back(head->getBaseStateIndexAt(k));
        }
        projection->lift(head->getBaseStateAt(size - 1), xFiberGoal, section_.back());
        sectionBaseStateIndices_.push_back(head->getBaseStateIndexAt(size - 1));
    }
    else
    {
        section_.resize(size);
        bundle->allocStates(section_);
        for (int k = 0; k < size; k++)
        {
            bundle->copyState(section_.at(k), head->getBaseStateAt(k));
            sectionBaseStateIndices_.push_back(head->getBaseStateIndexAt(k));
        }
    }
    sanityCheck(head);
}

void PathSection::interpolateL2(HeadPtr &head)
{
    section_.clear();
    sectionBaseStateIndices_.clear();

    BundleSpaceGraph *graph = restriction_->getBundleSpaceGraph();
    base::SpaceInformationPtr bundle = graph->getBundle();
    const std::vector<base::State *> basePath = restriction_->getBasePath();

    int size = head->getNumberOfRemainingStates() + 1;

    section_.resize(size);
    bundle->allocStates(section_);

    if (graph->getCoDimension() > 0)
    {
        const base::State *xFiberStart = head->getStateFiber();
        const base::State *xFiberGoal = head->getStateTargetFiber();

        double totalLengthBasePath = restriction_->getLengthBasePath();

        FiberedProjectionPtr projection = std::static_pointer_cast<FiberedProjection>(graph->getProjection());
        base::StateSpacePtr fiber = projection->getFiberSpace();

        for (unsigned int k = 0; k < restriction_->size(); k++)
        {
            double lengthCurrent = restriction_->getLengthBasePathUntil(k);
            double step = lengthCurrent / totalLengthBasePath;

            fiber->interpolate(xFiberStart, xFiberGoal, step, xFiberTmp_);

            projection->lift(restriction_->getBaseStateAt(k), xFiberTmp_, section_.at(k));

            sectionBaseStateIndices_.push_back(k);
        }
    }
    else
    {
        for (unsigned int k = 0; k < basePath.size(); k++)
        {
            bundle->copyState(section_.at(k), basePath.at(k));
            sectionBaseStateIndices_.push_back(k);
        }
    }
}

BundleSpaceGraph::Configuration *PathSection::addFeasibleSegment(Configuration *xLast, ompl::base::State *sNext)
{
    BundleSpaceGraph *graph = restriction_->getBundleSpaceGraph();

    base::SpaceInformationPtr bundle = graph->getBundle();

    Configuration *x = new Configuration(bundle, sNext);
    graph->addConfiguration(x);
    graph->addBundleEdge(xLast, x);

    x->parent = xLast;
    return x;
}

void PathSection::addFeasibleGoalSegment(Configuration *xLast, Configuration *xGoal)
{
    BundleSpaceGraph *graph = restriction_->getBundleSpaceGraph();
    if (xGoal->index < 0)  // graph->getGoalIndex())
    {
        graph->addConfiguration(xGoal);
        graph->addGoalConfiguration(xGoal);
    }
    graph->addBundleEdge(xLast, xGoal);

    xGoal->parent = xLast;
}

void PathSection::sanityCheck(HeadPtr &head)
{
    if (section_.size() > 0)
    {
        base::State *xi = head->getConfiguration()->state;
        base::State *xg = head->getTargetConfiguration()->state;

        BundleSpaceGraph *graph = restriction_->getBundleSpaceGraph();
        base::SpaceInformationPtr bundle = graph->getBundle();
        base::SpaceInformationPtr base = graph->getBase();

        double d1 = bundle->distance(section_.front(), xi);
        double d2 = bundle->distance(section_.back(), xg);

        if (d1 > 1e-5 || d2 > 1e-5)
        {
            std::stringstream buffer;
            buffer << "START STATE" << std::endl;
            bundle->printState(xi, buffer);
            bundle->printState(section_.front(), buffer);
            buffer << "Distance: " << d1 << std::endl;
            buffer << "GOAL STATE" << std::endl;
            bundle->printState(xg, buffer);
            buffer << "GOAL STATE (SECTION)" << std::endl;
            bundle->printState(section_.back(), buffer);
            buffer << "Dist:" << d2 << std::endl;
            int size = head->getNumberOfRemainingStates();
            buffer << "Section size: " << section_.size() << std::endl;
            buffer << "Remaining states: " << size << std::endl;
            buffer << "Restriction size:" << restriction_->size() << std::endl;
            buffer << "Base states:" << std::endl;
            buffer << *restriction_ << std::endl;
            OMPL_ERROR("Invalid Section: %s", buffer.str().c_str());
            throw Exception("Invalid Section");
        }
    }
}

void PathSection::sanityCheck()
{
    BundleSpaceGraph *graph = restriction_->getBundleSpaceGraph();
    base::SpaceInformationPtr bundle = graph->getBundle();
    bool feasible = true;
    for (unsigned int k = 1; k < section_.size(); k++)
    {
        base::State *sk1 = section_.at(k - 1);
        base::State *sk2 = section_.at(k);
        if (!bundle->checkMotion(sk1, sk2))
        {
            feasible = false;
            OMPL_ERROR("Error between states %d and %d.", k - 1, k);
            bundle->printState(sk1);
            bundle->printState(sk2);
        }
    }

    if (!feasible)
    {
        throw Exception("Reported feasible path section, \
        but path section is infeasible.");
    }
}

unsigned int PathSection::size() const
{
    return section_.size();
}

void PathSection::print(std::ostream &out) const
{
    BundleSpaceGraph *graph = restriction_->getBundleSpaceGraph();
    base::SpaceInformationPtr bundle = graph->getBundle();
    base::SpaceInformationPtr base = graph->getBase();

    out << std::string(80, '-') << std::endl;
    out << "PATH SECTION" << std::endl;
    out << std::string(80, '-') << std::endl;

    out << section_.size() << " states over " << restriction_->size() << " base states." << std::endl;

    int maxDisplay = 5;  // display first and last N elements
    for (int k = 0; k < (int)section_.size(); k++)
    {
        if (k > maxDisplay && k < std::max(0, (int)section_.size() - maxDisplay))
            continue;
        int idx = sectionBaseStateIndices_.at(k);
        out << "State " << k << ": ";
        bundle->printState(section_.at(k));
        out << "Over Base state (idx " << idx << ") ";
        base->printState(restriction_->getBasePath().at(idx));
        out << std::endl;
    }

    out << std::string(80, '-') << std::endl;
}

namespace ompl
{
    namespace multilevel
    {
        std::ostream &operator<<(std::ostream &out, const PathSection &s)
        {
            s.print(out);
            return out;
        }
    }
}
