/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, JSK, The University of Tokyo.
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
 *   * Neither the name of the JSK, The University of Tokyo nor the names of its
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

/* Author: Dave Coleman */

// OMPL
#include "ompl/tools/lightning/LightningDB.h"
#include "ompl/base/ScopedState.h"
#include "ompl/util/Time.h"
#include "ompl/tools/config/SelfConfig.h"
#include "ompl/base/PlannerDataStorage.h"

// Boost
#include <boost/filesystem.hpp>

ompl::tools::LightningDB::LightningDB(const base::StateSpacePtr &space)
{
    si_ = std::make_shared<base::SpaceInformation>(space);

    // Set nearest neighbor type
    nn_ = std::make_shared<ompl::NearestNeighborsSqrtApprox<ompl::base::PlannerDataPtr>>();

    // Use our custom distance function for nearest neighbor tree
    nn_->setDistanceFunction([this](const ompl::base::PlannerDataPtr &a, const ompl::base::PlannerDataPtr &b)
                             {
                                 return distanceFunction(a, b);
                             });

    // Load the PlannerData instance to be used for searching
    nnSearchKey_ = std::make_shared<ompl::base::PlannerData>(si_);
}

ompl::tools::LightningDB::~LightningDB()
{
    if (numUnsavedPaths_)
        OMPL_WARN("The database is being unloaded with unsaved experiences");
}

bool ompl::tools::LightningDB::load(const std::string &fileName)
{
    // Error checking
    if (fileName.empty())
    {
        OMPL_ERROR("Empty filename passed to save function");
        return false;
    }
    if (!boost::filesystem::exists(fileName))
    {
        OMPL_WARN("Database file does not exist: %s", fileName.c_str());
        return false;
    }

    // Load database from file, track loading time
    time::point start = time::now();

    OMPL_INFORM("Loading database from file: %s", fileName.c_str());

    // Open a binary input stream
    std::ifstream iStream(fileName.c_str(), std::ios::binary);

    // Get the total number of paths saved
    double numPaths = 0;
    iStream >> numPaths;

    // Check that the number of paths makes sense
    if (numPaths < 0 || numPaths > std::numeric_limits<double>::max())
    {
        OMPL_WARN("Number of paths to load %d is a bad value", numPaths);
        return false;
    }

    // Start loading all the PlannerDatas
    for (std::size_t i = 0; i < numPaths; ++i)
    {
        // Create a new planner data instance
        auto plannerData(std::make_shared<ompl::base::PlannerData>(si_));

        // Note: the StateStorage class checks if the states match for us
        plannerDataStorage_.load(iStream, *plannerData.get());

        // Add to nearest neighbor tree
        nn_->add(plannerData);
    }

    // Close file
    iStream.close();

    double loadTime = time::seconds(time::now() - start);
    OMPL_INFORM("Loaded database from file in %f sec with %d paths", loadTime, nn_->size());
    return true;
}

void ompl::tools::LightningDB::addPath(ompl::geometric::PathGeometric &solutionPath, double &insertionTime)
{
    // Benchmark runtime
    time::point startTime = time::now();
    {
        addPathHelper(solutionPath);
    }
    insertionTime = time::seconds(time::now() - startTime);
}

void ompl::tools::LightningDB::addPathHelper(ompl::geometric::PathGeometric &solutionPath)
{
    // Create a new planner data instance
    auto plannerData(std::make_shared<ompl::base::PlannerData>(si_));

    // Add the states to one nodes files
    for (auto &i : solutionPath.getStates())
    {
        ompl::base::PlannerDataVertex vert(i);  // TODO tag?

        plannerData->addVertex(vert);
    }

    // Deep copy the states in the vertices so that when the planner goes out of scope, all data remains intact
    plannerData->decoupleFromPlanner();

    // Add to nearest neighbor tree
    nn_->add(plannerData);

    numUnsavedPaths_++;
}

bool ompl::tools::LightningDB::saveIfChanged(const std::string &fileName)
{
    if (numUnsavedPaths_)
        return save(fileName);
    else
        OMPL_INFORM("Not saving because database has not changed");
    return true;
}

bool ompl::tools::LightningDB::save(const std::string &fileName)
{
    // Error checking
    if (fileName.empty())
    {
        OMPL_ERROR("Empty filename passed to save function");
        return false;
    }

    // Save database from file, track saving time
    time::point start = time::now();

    OMPL_INFORM("Saving database to file: %s", fileName.c_str());

    // Open a binary output stream
    std::ofstream outStream(fileName.c_str(), std::ios::binary);

    // Convert the NN tree to a vector
    std::vector<ompl::base::PlannerDataPtr> plannerDatas;
    nn_->list(plannerDatas);

    // Write the number of paths we will be saving
    double numPaths = plannerDatas.size();
    outStream << numPaths;

    // Start saving each planner data object
    for (std::size_t i = 0; i < numPaths; ++i)
    {
        ompl::base::PlannerData &pd = *plannerDatas[i].get();

        // Save a single planner data
        plannerDataStorage_.store(pd, outStream);
    }

    // Close file
    outStream.close();

    // Benchmark
    double loadTime = time::seconds(time::now() - start);
    OMPL_INFORM("Saved database to file in %f sec with %d paths", loadTime, plannerDatas.size());

    numUnsavedPaths_ = 0;

    return true;
}

void ompl::tools::LightningDB::getAllPlannerDatas(std::vector<ompl::base::PlannerDataPtr> &plannerDatas) const
{
    OMPL_DEBUG("LightningDB: getAllPlannerDatas");

    // Convert the NN tree to a vector
    nn_->list(plannerDatas);

    OMPL_DEBUG("Number of paths found: %d", plannerDatas.size());
}

std::vector<ompl::base::PlannerDataPtr> ompl::tools::LightningDB::findNearestStartGoal(int nearestK,
                                                                                       const base::State *start,
                                                                                       const base::State *goal)
{
    // Fill in our pre-made PlannerData instance with the new start and goal states to be searched for
    if (nnSearchKey_->numVertices() == 2)
    {
        nnSearchKey_->getVertex(0) = ompl::base::PlannerDataVertex(start);
        nnSearchKey_->getVertex(1) = ompl::base::PlannerDataVertex(goal);
    }
    else
    {
        nnSearchKey_->addVertex(ompl::base::PlannerDataVertex(start));
        nnSearchKey_->addVertex(ompl::base::PlannerDataVertex(goal));
    }
    assert(nnSearchKey_->numVertices() == 2);

    std::vector<ompl::base::PlannerDataPtr> nearest;
    nn_->nearestK(nnSearchKey_, nearestK, nearest);

    return nearest;
}

double ompl::tools::LightningDB::distanceFunction(const ompl::base::PlannerDataPtr &a,
                                                  const ompl::base::PlannerDataPtr &b) const
{
    // Bi-directional implementation - check path b from [start, goal] and [goal, start]
    return std::min(
        // [ a.start, b.start] + [a.goal + b.goal]
        si_->distance(a->getVertex(0).getState(), b->getVertex(0).getState()) +
            si_->distance(a->getVertex(a->numVertices() - 1).getState(), b->getVertex(b->numVertices() - 1).getState()),
        // [ a.start, b.goal] + [a.goal + b.start]
        si_->distance(a->getVertex(0).getState(), b->getVertex(b->numVertices() - 1).getState()) +
            si_->distance(a->getVertex(a->numVertices() - 1).getState(), b->getVertex(0).getState()));
}

std::size_t ompl::tools::LightningDB::getExperiencesCount() const
{
    return nn_->size();
}

std::size_t ompl::tools::LightningDB::getStatesCount() const
{
    // Loop through every PlannerData and sum the number of states
    std::size_t statesCount = 0;

    // Convert the NN tree to a vector
    std::vector<ompl::base::PlannerDataPtr> plannerDatas;
    nn_->list(plannerDatas);

    // Start saving each planner data object
    for (auto &plannerData : plannerDatas)
    {
        statesCount += plannerData->numVertices();
    }

    return statesCount;
}
