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

#include "ompl/base/PlannerDataStorage.h"
#include <boost/archive/archive_exception.hpp>

static const std::uint_fast32_t OMPL_PLANNER_DATA_ARCHIVE_MARKER = 0x5044414D;  // this spells PDAM

ompl::base::PlannerDataStorage::PlannerDataStorage() = default;

ompl::base::PlannerDataStorage::~PlannerDataStorage() = default;

void ompl::base::PlannerDataStorage::store(const PlannerData &pd, const char *filename)
{
    std::ofstream out(filename, std::ios::binary);
    store(pd, out);
    out.close();
}

void ompl::base::PlannerDataStorage::store(const PlannerData &pd, std::ostream &out)
{
    const SpaceInformationPtr &si = pd.getSpaceInformation();
    if (!out.good())
    {
        OMPL_ERROR("Failed to store PlannerData: output stream is invalid");
        return;
    }
    if (!si)
    {
        OMPL_ERROR("Failed to store PlannerData: SpaceInformation is invalid");
        return;
    }
    try
    {
        boost::archive::binary_oarchive oa(out);

        // Writing the header
        Header h;
        h.marker = OMPL_PLANNER_DATA_ARCHIVE_MARKER;
        h.vertex_count = pd.numVertices();
        h.edge_count = pd.numEdges();
        si->getStateSpace()->computeSignature(h.signature);
        oa << h;

        storeVertices(pd, oa);
        storeEdges(pd, oa);
    }
    catch (boost::archive::archive_exception &ae)
    {
        OMPL_ERROR("Failed to store PlannerData: %s", ae.what());
    }
}

void ompl::base::PlannerDataStorage::load(const char *filename, PlannerData &pd)
{
    std::ifstream in(filename, std::ios::binary);
    load(in, pd);
    in.close();
}

void ompl::base::PlannerDataStorage::load(std::istream &in, PlannerData &pd)
{
    pd.clear();

    const SpaceInformationPtr &si = pd.getSpaceInformation();
    if (!in.good())
    {
        OMPL_ERROR("Failed to load PlannerData: input stream is invalid");
        return;
    }
    if (!si)
    {
        OMPL_ERROR("Failed to load PlannerData: SpaceInformation is invalid");
        return;
    }
    // Loading the planner data:
    try
    {
        boost::archive::binary_iarchive ia(in);

        // Read the header
        Header h;
        ia >> h;

        // Checking the archive marker
        if (h.marker != OMPL_PLANNER_DATA_ARCHIVE_MARKER)
        {
            OMPL_ERROR("Failed to load PlannerData: PlannerData archive marker not found");
            return;
        }

        // Verify that the state space is the same
        std::vector<int> sig;
        si->getStateSpace()->computeSignature(sig);
        if (h.signature != sig)
        {
            OMPL_ERROR("Failed to load PlannerData: StateSpace signature mismatch");
            return;
        }

        // File seems ok... loading vertices and edges
        loadVertices(pd, h.vertex_count, ia);
        loadEdges(pd, h.edge_count, ia);
    }
    catch (boost::archive::archive_exception &ae)
    {
        OMPL_ERROR("Failed to load PlannerData: %s", ae.what());
    }
}
