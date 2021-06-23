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

#include <ompl/multilevel/planners/qrrt/STARImpl.h>
#include <ompl/multilevel/datastructures/graphsampler/GraphSampler.h>
#include <ompl/multilevel/datastructures/propagators/Geometric.h>
#include <ompl/multilevel/datastructures/metrics/Geodesic.h>
#include <ompl/tools/config/SelfConfig.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/special/TorusStateSpace.h>
#include <boost/foreach.hpp>

#define foreach BOOST_FOREACH
using namespace ompl::multilevel;

STARImpl::STARImpl(const base::SpaceInformationPtr &si, BundleSpace *parent_) : BaseT(si, parent_)
{
    setName("STARImpl" + std::to_string(id_));
    setImportance("exponential");
    setGraphSampler("randomvertex");
    getGraphSampler()->disableSegmentBias();
    distanceBetweenTrees_ = std::numeric_limits<double>::infinity();
}

STARImpl::~STARImpl()
{
}

void STARImpl::setup()
{
    BaseT::setup();

    maxDistance_ = 0.1;

    if(!treeStart_)
    {
      TreeData tree;
      tree.reset(tools::SelfConfig::getDefaultNearestNeighbors<Configuration *>(this));
      tree->setDistanceFunction(
          [this](const Configuration *a, const Configuration *b) 
          { return distance(a, b); });
      treeStart_ = std::make_shared<SparseTree>(tree);
      treeStart_->setup();
    }
    if(!treeGoal_)
    {
      TreeData tree;
      tree.reset(tools::SelfConfig::getDefaultNearestNeighbors<Configuration *>(this));
      tree->setDistanceFunction(
          [this](const Configuration *a, const Configuration *b) 
          { return distance(a, b); });

      treeGoal_ = std::make_shared<SparseTree>(tree);
      treeGoal_->setup();
    }
    // if(!treeStart_)
    // {
    //     treeStart_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Configuration *>(this));
    //     treeStart_->setDistanceFunction(
    //         [this](const Configuration *a, const Configuration *b) 
    //         { return distance(a, b); });
    // }
    // if(!treeGoal_)
    // {
    //     treeGoal_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Configuration *>(this));
    //     treeGoal_->setDistanceFunction(
    //         [this](const Configuration *a, const Configuration *b) 
    //         { return distance(a, b); });
    // }
}
void STARImpl::clear()
{
    BaseT::clear();
    treeStart_->clear();
    treeGoal_->clear();
    distanceBetweenTrees_ = std::numeric_limits<double>::infinity();
}

void STARImpl::init()
{
    if (const base::State *state = pis_.nextStart())
    {
        qStart_ = new Configuration(getBundle(), state);
        qStart_->isStart = true;
        addToTree(treeStart_, qStart_);
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
        addToTree(treeGoal_, qGoal_);
        goalConfigurations_.push_back(qGoal_);
    }

    if (qGoal_ == nullptr && getGoalPtr()->canSample())
    {
        OMPL_ERROR("%s: There are no valid goal states!", getName().c_str());
        throw ompl::Exception("Invalid goal states.");
    }
}

void STARImpl::addToTree(SparseTreePtr& tree, Configuration *x)
{
    Vertex m = boost::add_vertex(x, graph_);
    disjointSets_.make_set(m);
    x->index = m;
    tree->add(x);
    //analytics about tree elements
    // assert(m == treeElement_numberOfExtensions_.size()-1);
    // treeElement_numberOfExtensions_.push_back(0);
    // treeElement_numberOfSuccessfulExtensions_.push_back(0);
    // treeElement_isConverged_.push_back(false);
    // treeElement_numberOfUnsuccessfulSubsequentExtensions_.push_back(0);
    // x->importance = 0;
}

//void STARImpl::updateExtension(SparseTreePtr& tree, Configuration *x)
//{
//    int idx = x->index;
//    if(treeElement_numberOfUnsuccessfulSubsequentExtensions_.at(idx) > 1000)
//    {
//        treeElement_isConverged_.at(idx) = true;
//    }else{
//        int n = treeElement_numberOfExtensions_.at(idx);
//        int m = treeElement_numberOfSuccessfulExtensions_.at(idx);
//        tree->push(x, (double)m/(double)(n+1));
//    }
//}

//void STARImpl::updateUnsuccessfulExtension(SparseTreePtr& tree, Configuration *x)
//{
//  //DEBUG to visualize all samples
//    // Configuration *v = new Configuration(getBundle(), xRandom_->state);
//    // boost::add_vertex(v, graph_);

//    int idx = x->index;
//    treeElement_numberOfExtensions_.at(idx)++;
//    treeElement_numberOfUnsuccessfulSubsequentExtensions_.at(idx)++;
//    updateExtension(tree, x);
//}

//void STARImpl::updateSuccessfulExtension(SparseTreePtr& tree, Configuration *x)
//{
//    int idx = x->index;
//    treeElement_numberOfExtensions_.at(idx)++;
//    treeElement_numberOfSuccessfulExtensions_.at(idx)++;
//    treeElement_numberOfUnsuccessfulSubsequentExtensions_.at(idx) = 0;
//    updateExtension(tree, x);
//}

void STARImpl::grow()
{
    // TODO
    // [ ] Add conditional adding
    // [ ] Add termination criterion
    // [ ] Provide guarantees in terms of free space covered
    // [ ] Add explicit shell sampling to SO3
    // [x] Impl shell sampling
    // [x] Color start/goal trees differently
    //
    //  -> Longterm Goal is to solve 06D mug infeasible/feasible

    if (firstRun_)
    {
        init();
        firstRun_ = false;

        findSection();
    }

    //###########################################################
    //(0) Tree Selection. 
    SparseTreePtr &tree = activeInitialTree_ ? treeStart_ : treeGoal_;
    activeInitialTree_ = !activeInitialTree_;
    SparseTreePtr &otherTree = activeInitialTree_ ? treeStart_ : treeGoal_;

    //###########################################################
    //(1) State Selection
    //  [ ] Selected state based on 
    //    (i) number of unsuccessful attempts.
    //    (ii) frontier node status
    //    (iii) not yet expanded
    //    (iv) number of neighbors. More neighbors means more of the shell is
    //    blocked. 
    //
    // std::vector<Configuration*> treeElements;
    // tree->list(treeElements);

    // int selectedTreeElement = rng_.uniformInt(0, tree->size()-1);
    // Configuration *xSelected = treeElements.at(selectedTreeElement);

    Configuration *xSelected = tree->pop();

    //###########################################################
    //(2) Extend Selection
    // auto sampler = std::static_pointer_cast<base::RealVectorStateSampler>(getBundleSamplerPtr());

    double maxExt = getBundle()->getMaximumExtent();
    double sparseDelta = 0.1 * maxExt;

    auto sampler = getBundleSamplerPtr();
    sampler->sampleShell(xRandom_->state, xSelected->state, 
        sparseDelta, sparseDelta + 0.1*sparseDelta);

    bool valid = getBundle()->getStateValidityChecker()->isValid(xRandom_->state);
    if(!valid)
    {
        tree->push(xSelected, EXTENSION_FAILURE_INVALID_STATE);
        return;
    }

    //###########################################################
    //(3) Remove Covered Samples.
    //  Need to keep some to guarantee near-optimality.

    Configuration *xNearest = tree->nearest(xRandom_);
    double d = distance(xNearest, xRandom_);
    if (d <= sparseDelta && 
        getBundle()->checkMotion(xNearest->state, xRandom_->state))
    {
        tree->push(xSelected, EXTENSION_FAILURE_INSIDE_COVER);
        return;
    }

    //###########################################################
    //(4) Connect Selected to Random
    // Improvements: Do local planning to reach state. We could even use a
    // potential field method here. 

    if (!propagator_->steer(xSelected, xRandom_, xRandom_))
    {
        //If we find a valid sample which is not connectable, then this usually
        //indicates a complicated state space geometry which could be useful to
        //investigate further (i.e. allocate more resources to this)
        tree->push(xSelected, EXTENSION_FAILURE_NO_CONNECTION);
        return;
    }

    tree->push(xSelected, EXTENSION_SUCCESS);
    //###########################################################
    //(5) Valid Connected Element is added to Tree

    Configuration *xNext = new Configuration(getBundle(), xRandom_->state);
    addToTree(tree, xNext);
    addBundleEdge(xSelected, xNext);

    //###########################################################
    //(6) If extension was successful, check if we reached goal
    if (xNext && !hasSolution_)
    {
        /* update distance between trees */
        Configuration *xOtherTree = otherTree->nearest(xNext);
        const double newDist = distance(xNext, xOtherTree);
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
void STARImpl::getPlannerData(ompl::base::PlannerData &data) const
{
    BaseT::getPlannerData(data);
    OMPL_DEBUG(" Start Tree has %d vertices, Goal Tree has %d vertices.", 
        treeStart_->size(), treeGoal_->size());
}
