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
}

STARImpl::~STARImpl()
{
}

void STARImpl::grow()
{
  // TODO
  // [ ] Add conditional adding
  // [ ] Add termination criterion
  // [ ] Provide guarantees in terms of free space covered
  // [ ] Impl shell sampling
  // [x] Color start/goal trees differently

    //(0) If first run, add start configuration
    if (firstRun_)
    {
        init();
        firstRun_ = false;

        findSection();
    }

    //###########################################################
    //(0) Tree Selection. Can stay like that.
    TreeData &tree = activeInitialTree_ ? treeStart_ : treeGoal_;
    activeInitialTree_ = !activeInitialTree_;
    TreeData &otherTree = activeInitialTree_ ? treeStart_ : treeGoal_;

    //###########################################################
    //(1) State Selection
    //  [ ] Selected state based on 
    //    (i) number of unsuccessful attempts.
    //    (ii) frontier node status
    //    (iii) not yet expanded
    //    (iv) number of neighbors. More neighbors means more of the shell is
    //    blocked. 
    //
    std::vector<Configuration*> treeElements;
    tree->list(treeElements);

    int selectedTreeElement = rng_.uniformInt(0, tree->size()-1);
    Configuration *xSelected = treeElements.at(selectedTreeElement);

    //###########################################################
    //(2) Extend Selection
    // auto sampler = std::static_pointer_cast<base::RealVectorStateSampler>(getBundleSamplerPtr());
    double maxExt = getBundle()->getMaximumExtent();
    double sparseDelta = 0.05 * maxExt;

    int type = getBundle()->getStateSpace()->getType();
    if(type == base::STATE_SPACE_REAL_VECTOR)
    {
        auto sampler = std::static_pointer_cast<base::RealVectorStateSampler>(getBundleSamplerPtr());
        sampler->sampleShell(xRandom_->state, xSelected->state, 
            sparseDelta, sparseDelta + 0.1*sparseDelta);
    }else if(type == base::STATE_SPACE_TORUS)
    {
        auto sampler = std::static_pointer_cast<base::TorusStateSampler>(getBundleSamplerPtr());
        sampler->sampleShell(xRandom_->state, xSelected->state, 
            sparseDelta, sparseDelta + 0.1*sparseDelta);
    }else{
        auto sampler = getBundleSamplerPtr();
        sampler->sampleUniformNear(xRandom_->state, xSelected->state, sparseDelta + 0.1*sparseDelta);
        throw "NYI";
    }

    bool valid = getBundle()->getStateValidityChecker()->isValid(xRandom_->state);
    if(!valid)
    {
        //For debugging purposes
        Configuration *x = new Configuration(getBundle(), xRandom_->state);
        boost::add_vertex(x, graph_);
        return;
    }
    //###########################################################
    //(3) Remove Covered Samples.
    //  Need to keep some to guarantee near-optimality.
    Configuration *xNearest = tree->nearest(xRandom_);
    double d = distance(xNearest, xRandom_);
    if (d < sparseDelta)
    {
        //For debugging purposes
        Configuration *x = new Configuration(getBundle(), xRandom_->state);
        boost::add_vertex(x, graph_);
        //sample is covered by xNearest
        return;
    }

    //###########################################################
    //(4) Connect Selected to Random
    // Improvements: Do local planning to reach state. We could even use a
    // potential field method here. 

    if (!propagator_->steer(xSelected, xRandom_, xRandom_))
    {
        Configuration *x = new Configuration(getBundle(), xRandom_->state);
        boost::add_vertex(x, graph_);
        return;
    }

    //###########################################################
    //(5) Valid Connected Element is added to Tree
    Configuration *xNext = new Configuration(getBundle(), xRandom_->state);
    Vertex m = boost::add_vertex(xNext, graph_);
    disjointSets_.make_set(m);
    xNext->index = m;
    tree->add(xNext);
    addBundleEdge(xNearest, xNext);

    //###########################################################
    //(6) If extension was successful, check if we reached goal
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
