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

#include <ompl/multilevel/planners/qrrt/BiQRRTImpl.h>
#include <ompl/multilevel/datastructures/graphsampler/GraphSampler.h>
#include <ompl/multilevel/datastructures/propagators/Geometric.h>
#include <ompl/multilevel/datastructures/metrics/Geodesic.h>
#include <ompl/tools/config/SelfConfig.h>
#include <boost/foreach.hpp>

#define foreach BOOST_FOREACH
using namespace ompl::multilevel;

ompl::multilevel::BiQRRTImpl::BiQRRTImpl(const base::SpaceInformationPtr &si, BundleSpace *parent_) : BaseT(si, parent_)
{
    setName("BiQRRTImpl" + std::to_string(id_));
    setImportance("exponential");
    setGraphSampler("randomvertex");
    getGraphSampler()->disableSegmentBias();
    distanceBetweenTrees_ = std::numeric_limits<double>::infinity();
}

ompl::multilevel::BiQRRTImpl::~BiQRRTImpl()
{
}
void BiQRRTImpl::setup()
{
    BaseT::setup();

    maxDistance_ = 0.1;

    if(!treeStart_)
    {
        treeStart_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Configuration *>(this));
        treeStart_->setDistanceFunction(
            [this](const Configuration *a, const Configuration *b) 
            { return distance(a, b); });
    }
    if(!treeGoal_)
    {
        treeGoal_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Configuration *>(this));
        treeGoal_->setDistanceFunction(
            [this](const Configuration *a, const Configuration *b) 
            { return distance(a, b); });
    }
}
void BiQRRTImpl::clear()
{
    BaseT::clear();
    if (treeStart_) treeStart_->clear();
    if (treeGoal_) treeGoal_->clear();
    distanceBetweenTrees_ = std::numeric_limits<double>::infinity();
}

void BiQRRTImpl::init()
{
    if (const base::State *state = pis_.nextStart())
    {
        qStart_ = new Configuration(getBundle(), state);
        qStart_->isStart = true;
        Vertex m = boost::add_vertex(qStart_, graph_);
        disjointSets_.make_set(m);
        qStart_->index = m;
        treeStart_->add(qStart_);
    }

    if (qStart_ == nullptr)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        throw ompl::Exception("Invalid initial states.");
    }

    if (const base::State *state = pis_.nextGoal())
    {
        qGoal_ = new Configuration(getBundle(), state);
        qGoal_->isGoal = true;
        Vertex m = boost::add_vertex(qGoal_, graph_);
        disjointSets_.make_set(m);
        qGoal_->index = m;
        treeGoal_->add(qGoal_);
        goalConfigurations_.push_back(qGoal_);
    }

    if (qGoal_ == nullptr && getGoalPtr()->canSample())
    {
        OMPL_ERROR("%s: There are no valid goal states!", getName().c_str());
        throw ompl::Exception("Invalid goal states.");
    }
}


void BiQRRTImpl::grow()
{
    //(0) If first run, add start configuration
    if (firstRun_)
    {
        init();
        firstRun_ = false;

        findSection();
    }

    TreeData &tree = activeInitialTree_ ? treeStart_ : treeGoal_;
    activeInitialTree_ = !activeInitialTree_;
    TreeData &otherTree = activeInitialTree_ ? treeStart_ : treeGoal_;

    //###########################################################
    //(1) Get Uniform Random Sample
    sampleBundle(xRandom_->state);

    //###########################################################
    //(2) Get Nearest in Current Highlighted Tree
    const Configuration *xNearest = tree->nearest(xRandom_);

    //###########################################################
    //(3) Connect Nearest to Random (within range)
    // Configuration *xNext = extendGraphTowards_Range(xNearest, xRandom_);

    double d = distance(xNearest, xRandom_);
    if (d > maxDistance_)
    {
        metric_->interpolateBundle(xNearest, xRandom_, maxDistance_ / d, xRandom_);
    }

    if (!propagator_->steer(xNearest, xRandom_, xRandom_))
    {
        return;
    }

    Configuration *xNext = new Configuration(getBundle(), xRandom_->state);
    Vertex m = boost::add_vertex(xNext, graph_);
    disjointSets_.make_set(m);
    xNext->index = m;
    tree->add(xNext);
    addBundleEdge(xNearest, xNext);

    //###########################################################
    //(4) If extension was successful, check if we reached goal
    if (xNext && !hasSolution_)
    {
        /* update distance between trees */
        Configuration *xOtherTree = otherTree->nearest(xNext);
        const double newDist = tree->getDistanceFunction()(xNext, xOtherTree);
        if (newDist < distanceBetweenTrees_)
        {
            distanceBetweenTrees_ = newDist;
            OMPL_INFORM("Estimated distance to go: %f", distanceBetweenTrees_);
        }

        bool satisfied = propagator_->steer(xNext, xOtherTree, xRandom_);

        if (satisfied)
        {
            addBundleEdge(xNext, xOtherTree);
            hasSolution_ = true;
        }
    }
}

void BiQRRTImpl::getPlannerData(ompl::base::PlannerData &data) const
{
    BaseT::getPlannerData(data);
    OMPL_DEBUG(" Start Tree has %d vertices, Goal Tree has %d vertices.", 
        treeStart_->size(), treeGoal_->size());
}
