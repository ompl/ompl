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

#include <ompl/multilevel/datastructures/pathrestriction/PathRestriction.h>
#include <ompl/multilevel/datastructures/pathrestriction/Head.h>
#include <ompl/multilevel/datastructures/pathrestriction/FindSection.h>
#include <ompl/multilevel/datastructures/pathrestriction/FindSectionSideStep.h>
#include <ompl/multilevel/datastructures/graphsampler/GraphSampler.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>

#include <ompl/base/Path.h>
#include <ompl/geometric/PathGeometric.h>
#include <numeric>
#include <ompl/util/Time.h>

using namespace ompl::multilevel;

PathRestriction::PathRestriction(BundleSpaceGraph *bundleSpaceGraph) : bundleSpaceGraph_(bundleSpaceGraph)
{
    setFindSectionStrategy(FindSectionType::SIDE_STEP);
}

void PathRestriction::setFindSectionStrategy(FindSectionType type)
{
    switch (type)
    {
        case FindSectionType::SIDE_STEP:
            findSection_ = std::make_shared<FindSectionSideStep>(this);
            break;
        case FindSectionType::NONE:
            findSection_ = nullptr;
            break;
        default:
            OMPL_ERROR("Find section strategy unknown: %s", type);
            throw ompl::Exception("Unknown Strategy");
            break;
    }
}

PathRestriction::~PathRestriction()
{
}

void PathRestriction::clear()
{
    basePath_.clear();
    lengthsIntermediateBasePath_.clear();
    lengthsCumulativeBasePath_.clear();
    lengthBasePath_ = 0;
}

BundleSpaceGraph *PathRestriction::getBundleSpaceGraph()
{
    return bundleSpaceGraph_;
}

void PathRestriction::setBasePath(ompl::base::PathPtr path)
{
    if (!path)
        return;
    auto geometricBasePath = std::static_pointer_cast<geometric::PathGeometric>(path);
    setBasePath(geometricBasePath->getStates());
}

void PathRestriction::setBasePath(std::vector<ompl::base::State *> basePath)
{
    basePath_.clear();
    lengthsIntermediateBasePath_.clear();
    lengthsCumulativeBasePath_.clear();
    lengthBasePath_ = 0.0;

    basePath_ = basePath;

    for (unsigned int k = 1; k < basePath_.size(); k++)
    {
        double lk = bundleSpaceGraph_->getBase()->distance(basePath_.at(k - 1), basePath_.at(k));
        lengthsIntermediateBasePath_.push_back(lk);
        lengthBasePath_ += lk;
        lengthsCumulativeBasePath_.push_back(lengthBasePath_);
    }
    OMPL_DEBUG("Set new base path with %d states and length %f.", basePath_.size(), lengthBasePath_);
}

void PathRestriction::interpolateBasePath(double t, ompl::base::State *&state) const
{
    base::SpaceInformationPtr base = bundleSpaceGraph_->getBase();

    if (t <= 0)
    {
        base->copyState(state, basePath_.front());
        return;
    }
    if (t >= lengthBasePath_)
    {
        base->copyState(state, basePath_.back());
        return;
    }

    unsigned int ctr = 0;
    while (t > lengthsCumulativeBasePath_.at(ctr) && ctr < lengthsCumulativeBasePath_.size() - 1)
    {
        ctr++;
    }

    base::State *s1 = basePath_.at(ctr);
    base::State *s2 = basePath_.at(ctr + 1);
    double d = lengthsIntermediateBasePath_.at(ctr);

    double dCum = (ctr > 0 ? lengthsCumulativeBasePath_.at(ctr - 1) : 0.0);
    double step = (t - dCum) / d;

    base->getStateSpace()->interpolate(s1, s2, step, state);
}

const std::vector<ompl::base::State *> &PathRestriction::getBasePath() const
{
    return basePath_;
}

double PathRestriction::getLengthBasePath() const
{
    return lengthBasePath_;
}

unsigned int PathRestriction::size() const
{
    return basePath_.size();
}

const ompl::base::State *PathRestriction::getBaseStateAt(int k) const
{
    return basePath_.at(k);
}

// distance between base states k and k+1
double PathRestriction::getLengthIntermediateBasePath(int k)
{
    return lengthsIntermediateBasePath_.at(k);
}

double PathRestriction::getLengthBasePathUntil(int k)
{
    if (k > (int)size())
    {
        OMPL_ERROR("Wrong index k=%d/%d", k, size());
        throw Exception("WrongIndex");
    }
    if (k <= 0)
        return 0;
    else
    {
        return lengthsCumulativeBasePath_.at(k - 1);
    }
}

int PathRestriction::getBasePathLastIndexFromLocation(double d)
{
    if (d > lengthBasePath_)
    {
        return size() - 1;
    }
    unsigned int ctr = 0;
    while (d >= lengthsCumulativeBasePath_.at(ctr) && ctr < lengthsCumulativeBasePath_.size() - 1)
    {
        ctr++;
    }
    return ctr;
}

bool PathRestriction::hasFeasibleSection(Configuration *const xStart, Configuration *const xGoal)
{
    if (findSection_ == nullptr)
        return false;

    HeadPtr head = std::make_shared<Head>(this, xStart, xGoal);

    ompl::time::point tStart = ompl::time::now();
    bool foundFeasibleSection = findSection_->solve(head);
    ompl::time::point t1 = ompl::time::now();

    OMPL_DEBUG("FindSection terminated after %.2fs (%d/%d vertices/edges).", ompl::time::seconds(t1 - tStart),
               bundleSpaceGraph_->getNumberOfVertices(), bundleSpaceGraph_->getNumberOfEdges());

    return foundFeasibleSection;
}

void PathRestriction::print(std::ostream &out) const
{
    const base::SpaceInformationPtr bundle = bundleSpaceGraph_->getBundle();
    const base::SpaceInformationPtr base = bundleSpaceGraph_->getBase();

    out << std::string(80, '-') << std::endl;
    out << "PATH RESTRICTION" << std::endl;
    out << std::string(80, '-') << std::endl;

    for (unsigned int k = 0; k < basePath_.size(); k++)
    {
        if (k > 5 && (int)k < std::max(0, (int)basePath_.size() - 5))
            continue;
        const base::State *bk = basePath_.at(k);
        base->printState(bk, out);
    }
    out << std::string(80, '-') << std::endl;
}

namespace ompl
{
    namespace multilevel
    {
        std::ostream &operator<<(std::ostream &out, const PathRestriction &r)
        {
            r.print(out);
            return out;
        }
    }
}
