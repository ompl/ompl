/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2012, Rice University
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
*   * Neither the name of the Rice University nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
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

/* Author: Ryan Luna */

#include "ompl/control/PlannerData.h"

ompl::control::PlannerData::PlannerData(const SpaceInformationPtr &siC)
  : base::PlannerData(std::static_pointer_cast<base::SpaceInformation>(siC)), siC_(siC)
{
}

ompl::control::PlannerData::~PlannerData()
{
    freeMemory();
}

bool ompl::control::PlannerData::removeVertex(unsigned int vIndex)
{
    return ompl::base::PlannerData::removeVertex(vIndex);
}

bool ompl::control::PlannerData::removeVertex(const ompl::base::PlannerDataVertex &st)
{
    unsigned int index = vertexIndex(st);
    if (index == INVALID_INDEX)
        return false;

    std::map<unsigned int, const base::PlannerDataEdge *> edgeMap;
    getEdges(index, edgeMap);

    for (auto &edgemapit : edgeMap)
    {
        // Before deleting the edge, free the control associated with it, if it was decoupled
        auto *ctrl =
            const_cast<Control *>(static_cast<const PlannerDataEdgeControl *>(edgemapit.second)->getControl());
        auto it = decoupledControls_.find(ctrl);
        if (it != decoupledControls_.end())
        {
            siC_->freeControl(*it);
            decoupledControls_.erase(it);
        }
    }

    return ompl::base::PlannerData::removeVertex(index);
}

bool ompl::control::PlannerData::removeEdge(unsigned int v1, unsigned int v2)
{
    return ompl::base::PlannerData::removeEdge(v1, v2);
}

bool ompl::control::PlannerData::removeEdge(const ompl::base::PlannerDataVertex &v1,
                                            const ompl::base::PlannerDataVertex &v2)
{
    unsigned int index1, index2;
    index1 = vertexIndex(v1);
    index2 = vertexIndex(v2);

    if (index1 == INVALID_INDEX || index2 == INVALID_INDEX)
        return false;

    // Before deleting the edge, free the control associated with it, if it was decoupled
    auto &edge = static_cast<PlannerDataEdgeControl &>(getEdge(index1, index2));
    auto *ctrl = const_cast<Control *>(edge.getControl());
    auto it = decoupledControls_.find(ctrl);
    if (it != decoupledControls_.end())
    {
        siC_->freeControl(*it);
        decoupledControls_.erase(it);
    }

    return ompl::base::PlannerData::removeEdge(index1, index2);
}

void ompl::control::PlannerData::clear()
{
    ompl::base::PlannerData::clear();

    freeMemory();
    decoupledControls_.clear();
}

void ompl::control::PlannerData::decoupleFromPlanner()
{
    ompl::base::PlannerData::decoupleFromPlanner();

    for (unsigned int i = 0; i < numVertices(); ++i)
    {
        for (unsigned int j = 0; j < numVertices(); ++j)
        {
            if (edgeExists(i, j))
            {
                auto &edge = static_cast<PlannerDataEdgeControl &>(getEdge(i, j));
                // If this edge's control is not in the decoupled list, clone it and add it
                auto *ctrl = const_cast<Control *>(edge.getControl());
                if (decoupledControls_.find(ctrl) == decoupledControls_.end())
                {
                    Control *clone = siC_->cloneControl(ctrl);
                    decoupledControls_.insert(clone);
                    // Replacing the shallow control pointer with our shiny new clone
                    edge.c_ = clone;
                }
            }
        }
    }
}

const ompl::control::SpaceInformationPtr &ompl::control::PlannerData::getSpaceInformation() const
{
    return siC_;
}

bool ompl::control::PlannerData::hasControls() const
{
    return true;
}

void ompl::control::PlannerData::freeMemory()
{
    for (auto decoupledControl : decoupledControls_)
        siC_->freeControl(decoupledControl);
}
