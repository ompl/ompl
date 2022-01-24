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

/* Author: Andreas Orthey, Sohaib Akbar */

#include <ompl/multilevel/planners/qmp/QMPImpl.h>
#include <ompl/tools/config/SelfConfig.h>
#include <boost/foreach.hpp>
#include <ompl/datastructures/NearestNeighbors.h>
#include "ompl/datastructures/PDF.h"
#include <ompl/geometric/PathGeometric.h>
#include <ompl/multilevel/datastructures/graphsampler/GraphSampler.h>

#define foreach BOOST_FOREACH

ompl::multilevel::QMPImpl::QMPImpl(const base::SpaceInformationPtr &si, BundleSpace *parent_) : BaseT(si, parent_)
{
    setName("QMPImpl" + std::to_string(id_));

    setMetric("geodesic");
    setGraphSampler("randomedge");
    setImportance("exponential");

    randomWorkStates_.resize(5);
    getBundle()->allocStates(randomWorkStates_);
}

ompl::multilevel::QMPImpl::~QMPImpl()
{
    getBundle()->freeStates(randomWorkStates_);
}

void ompl::multilevel::QMPImpl::clear()
{
    BaseT::clear();
    pdf.clear();
}

ompl::multilevel::BundleSpaceGraph::Vertex ompl::multilevel::QMPImpl::addConfiguration(Configuration *q)
{
    Vertex m = BaseT::addConfiguration(q);
    PDF_Element *q_element = pdf.add(q, 1.0);
    q->setPDFElement(q_element);
    return m;
}

void ompl::multilevel::QMPImpl::deleteConfiguration(Configuration *q)
{
    q->setPDFElement(nullptr);
    BaseT::deleteConfiguration(q);
}

void ompl::multilevel::QMPImpl::updatePDF(Configuration *q)
{
    unsigned int N = q->total_connection_attempts;
    unsigned int Ns = q->successful_connection_attempts;
    double val = (double)(N - Ns) / (double)N;
    pdf.update(static_cast<PDF_Element *>(q->getPDFElement()), val);
}

unsigned int ompl::multilevel::QMPImpl::computeK()
{
    return k_NearestNeighbors_;
}

void ompl::multilevel::QMPImpl::grow()
{
    if (firstRun_)
    {
        init();
        firstRun_ = false;

        if (qGoal_ == nullptr)
        {
            qGoal_ = new Configuration(getBundle());
            qGoal_->isGoal = true;
        }
        getGoalPtr()->sampleGoal(qGoal_->state);
        addConfiguration(qGoal_);
        addGoalConfiguration(qGoal_);

        findSection();
    }

    //(1) Get Random Sample
    if (!sampleBundleValid(xRandom_->state))
        return;

    //(2) Add Configuration if valid
    Configuration *xNew = new Configuration(getBundle(), xRandom_->state);
    addConfiguration(xNew);

    //(3) Connect to K nearest neighbors
    connectNeighbors(xNew);

    expand();

    if (!hasSolution_)
    {
        if (sameComponent(vStart_, getGoalIndex()))
        {
            hasSolution_ = true;
        }
    }
}

void ompl::multilevel::QMPImpl::connectNeighbors(Configuration *x)
{
    std::vector<Configuration *> nearestNeighbors;

    BaseT::nearestDatastructure_->nearestK(x, computeK(), nearestNeighbors);

    for (unsigned int k = 0; k < nearestNeighbors.size(); k++)
    {
        Configuration *xNear = nearestNeighbors.at(k);

        x->total_connection_attempts++;
        xNear->total_connection_attempts++;

        if (connect(xNear, x))
        {
            x->successful_connection_attempts++;
            xNear->successful_connection_attempts++;
        }
        updatePDF(xNear);
    }
    updatePDF(x);
}

void ompl::multilevel::QMPImpl::expand()
{
    if (pdf.empty())
        return;

    Configuration *q = pdf.sample(rng_.uniform01());

    int s = getBundle()->randomBounceMotion(getBundleSamplerPtr(), q->state, randomWorkStates_.size(),
                                            randomWorkStates_, false);
    if (s > 0)
    {
        Configuration *prev = q;

        Configuration *last = new Configuration(getBundle(), randomWorkStates_[--s]);
        addConfiguration(last);
        connectNeighbors(last);

        for (int i = 0; i < s; i++)
        {
            Configuration *tmp = new Configuration(getBundle(), randomWorkStates_[i]);
            addConfiguration(tmp);

            ompl::multilevel::BundleSpaceGraph::addEdge(prev->index, tmp->index);
            prev = tmp;
        }
        if (!sameComponent(prev->index, last->index))
            ompl::multilevel::BundleSpaceGraph::addEdge(prev->index, last->index);
    }
}
