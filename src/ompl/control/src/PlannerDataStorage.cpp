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

#include "ompl/control/PlannerDataStorage.h"
#include <boost/archive/archive_exception.hpp>

/// \cond IGNORE
static const boost::uint32_t OMPL_PLANNER_DATA_CONTROL_ARCHIVE_MARKER = 0x5044434D;  // this spells PDCM
/// \endcond

bool ompl::control::PlannerDataStorage::load(const char *filename, base::PlannerData &pd)
{
    return base::PlannerDataStorage::load(filename, pd);
}

bool ompl::control::PlannerDataStorage::load(std::istream &in, base::PlannerData &pd)
{
    if (!pd.hasControls())
    {
        OMPL_WARN("PlannerData does not have controls.  Invoking base::PlannerDataStorage::load");
        return base::PlannerDataStorage::load(in, pd);
    }

    auto *pdc = static_cast<control::PlannerData *>(&pd);
    pdc->clear();

    const SpaceInformationPtr &si = pdc->getSpaceInformation();
    if (!in.good())
    {
        OMPL_ERROR("Failed to load PlannerData: input stream is invalid");
        return false;
    }
    if (!si)
    {
        OMPL_ERROR("Failed to load PlannerData: SpaceInformation is invalid");
        return false;
    }
    // Loading the planner data:
    try
    {
        boost::archive::binary_iarchive ia(in);

        // Read the header
        Header h;
        ia >> h;

        // Checking the archive marker
        if (h.marker != OMPL_PLANNER_DATA_CONTROL_ARCHIVE_MARKER)
        {
            OMPL_ERROR("Failed to load PlannerData: PlannerData control archive marker not found");
            return false;
        }

        // Verify that the state space is the same
        std::vector<int> sig;
        si->getStateSpace()->computeSignature(sig);
        if (h.signature != sig)
        {
            OMPL_ERROR("Failed to load PlannerData: StateSpace signature mismatch");
            return false;
        }

        // Verify that the control space is the same
        sig.clear();
        si->getControlSpace()->computeSignature(sig);
        if (h.control_signature != sig)
        {
            OMPL_ERROR("Failed to load PlannerData: ControlSpace signature mismatch");
            return false;
        }

        // File seems ok... loading vertices and edges
        loadVertices(pd, h.vertex_count, ia);
        loadEdges(pd, h.edge_count, ia);
    }
    catch (boost::archive::archive_exception &ae)
    {
        OMPL_ERROR("Failed to load PlannerData: %s", ae.what());
        return false;
    }

    return true;
}

bool ompl::control::PlannerDataStorage::store(const base::PlannerData &pd, const char *filename)
{
    return base::PlannerDataStorage::store(pd, filename);
}

bool ompl::control::PlannerDataStorage::store(const base::PlannerData &pd, std::ostream &out)
{
    const auto *pdc = static_cast<const control::PlannerData *>(&pd);
    if (pdc == nullptr)
    {
        OMPL_WARN("Failed to cast PlannerData to control::PlannerData.  Invoking base::PlannerDataStorage::store");
        return base::PlannerDataStorage::store(pd, out);
    }

    const SpaceInformationPtr &si = pdc->getSpaceInformation();
    if (!out.good())
    {
        OMPL_ERROR("Failed to store PlannerData: output stream is invalid");
        return false;
    }
    if (!si)
    {
        OMPL_ERROR("Failed to store PlannerData: SpaceInformation is invalid");
        return false;
    }
    try
    {
        boost::archive::binary_oarchive oa(out);

        // Writing the header
        Header h;
        h.marker = OMPL_PLANNER_DATA_CONTROL_ARCHIVE_MARKER;
        h.vertex_count = pdc->numVertices();
        h.edge_count = pdc->numEdges();
        si->getStateSpace()->computeSignature(h.signature);
        si->getControlSpace()->computeSignature(h.control_signature);
        oa << h;

        storeVertices(pd, oa);
        storeEdges(pd, oa);
    }
    catch (boost::archive::archive_exception &ae)
    {
        OMPL_ERROR("Failed to store PlannerData: %s", ae.what());
        return false;
    }

    return true;
}
