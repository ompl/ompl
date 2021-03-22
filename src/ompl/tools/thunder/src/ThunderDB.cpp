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
#include <ompl/tools/thunder/ThunderDB.h>
#include <ompl/base/ScopedState.h>
#include <ompl/util/Time.h>
#include <ompl/util/Console.h>
#include <ompl/tools/config/SelfConfig.h>
#include <ompl/base/PlannerDataStorage.h>

// Boost
#include <boost/filesystem.hpp>

ompl::tools::ThunderDB::ThunderDB(const base::StateSpacePtr &space) : numPathsInserted_(0), saving_enabled_(true)
{
    // Set space information
    si_ = std::make_shared<base::SpaceInformation>(space);
}

ompl::tools::ThunderDB::~ThunderDB()
{
    if (numPathsInserted_)
        OMPL_WARN("The database is being unloaded with unsaved experiences");
}

bool ompl::tools::ThunderDB::load(const std::string &fileName)
{
    // Error checking
    if (fileName.empty())
    {
        OMPL_ERROR("Empty filename passed to save function");
        return false;
    }
    if (!boost::filesystem::exists(fileName))
    {
        OMPL_INFORM("Database file does not exist: %s.", fileName.c_str());
        return false;
    }
    if (!spars_)
    {
        OMPL_ERROR("SPARSdb planner has not been passed into the ThunderDB yet");
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

    if (numPaths > 1)
    {
        OMPL_ERROR("Currently more than one planner data is disabled from loading");
        return false;
    }

    // Create a new planner data instance
    auto plannerData(std::make_shared<ompl::base::PlannerData>(si_));

    // Note: the StateStorage class checks if the states match for us
    plannerDataStorage_.load(iStream, *plannerData.get());

    OMPL_INFORM("ThunderDB: Loaded planner data with \n  %d vertices\n  %d edges\n  %d start states\n  %d goal states",
                plannerData->numVertices(), plannerData->numEdges(), plannerData->numStartVertices(),
                plannerData->numGoalVertices());

    // Add to SPARSdb
    OMPL_INFORM("Adding plannerData to SPARSdb:");
    spars_->setPlannerData(*plannerData);

    // Output the number of connected components
    OMPL_INFORM("  %d connected components", spars_->getNumConnectedComponents());

    // Close file
    iStream.close();

    double loadTime = time::seconds(time::now() - start);
    OMPL_INFORM("Loaded database from file in %f sec ", loadTime);
    return true;
}

bool ompl::tools::ThunderDB::addPath(ompl::geometric::PathGeometric &solutionPath, double &insertionTime)
{
    // Error check
    if (!spars_)
    {
        OMPL_ERROR("SPARSdb planner has not been passed into the ThunderDB yet");
        insertionTime = 0;
        return false;
    }

    // Prevent inserting into database
    if (!saving_enabled_)
    {
        OMPL_WARN("ThunderDB: Saving is disabled so not adding path");
        return false;
    }

    bool result;
    double seconds = 120;  // 10; // a large number, should never need to use this
    ompl::base::PlannerTerminationCondition ptc = ompl::base::timedPlannerTerminationCondition(seconds, 0.1);

    // Benchmark runtime
    time::point startTime = time::now();
    {
        result = spars_->addPathToRoadmap(ptc, solutionPath);
    }
    insertionTime = time::seconds(time::now() - startTime);

    OMPL_INFORM("SPARSdb now has %d states", spars_->getNumVertices());

    // Record this new addition
    numPathsInserted_++;

    return result;
}

bool ompl::tools::ThunderDB::saveIfChanged(const std::string &fileName)
{
    if (numPathsInserted_)
        return save(fileName);
    else
        OMPL_INFORM("Not saving because database has not changed");
    return true;
}

bool ompl::tools::ThunderDB::save(const std::string &fileName)
{
    // Disabled
    if (!saving_enabled_)
    {
        OMPL_WARN("Not saving because option disabled for ExperienceDB");
        return false;
    }

    // Error checking
    if (fileName.empty())
    {
        OMPL_ERROR("Empty filename passed to save function");
        return false;
    }
    if (!spars_)
    {
        OMPL_ERROR("SPARSdb planner has not been passed into the ThunderDB yet");
        return false;
    }

    // Save database from file, track saving time
    time::point start = time::now();

    OMPL_INFORM("Saving database to file: %s", fileName.c_str());

    // Open a binary output stream
    std::ofstream outStream(fileName.c_str(), std::ios::binary);

    // Populate multiple planner Datas
    std::vector<ompl::base::PlannerDataPtr> plannerDatas;

    // TODO: make this more than 1 planner data perhaps
    auto data(std::make_shared<base::PlannerData>(si_));
    spars_->getPlannerData(*data);
    OMPL_INFORM("Get planner data from SPARS2 with \n  %d vertices\n  %d edges\n  %d start states\n  %d goal states",
                data->numVertices(), data->numEdges(), data->numStartVertices(), data->numGoalVertices());

    plannerDatas.push_back(data);

    // Write the number of paths we will be saving
    double numPaths = plannerDatas.size();
    outStream << numPaths;

    // Start saving each planner data object
    for (std::size_t i = 0; i < numPaths; ++i)
    {
        ompl::base::PlannerData &pd = *plannerDatas[i].get();

        OMPL_INFORM("Saving experience %d with %d vertices and %d edges", i, pd.numVertices(), pd.numEdges());

        if (false)  // debug code
        {
            for (std::size_t i = 0; i < pd.numVertices(); ++i)
            {
                OMPL_INFORM("Vertex %d:", i);
                debugVertex(pd.getVertex(i));
            }
        }

        // Save a single planner data
        plannerDataStorage_.store(pd, outStream);
    }

    // Close file
    outStream.close();

    // Benchmark
    double loadTime = time::seconds(time::now() - start);
    OMPL_INFORM("Saved database to file in %f sec with %d planner datas", loadTime, plannerDatas.size());

    numPathsInserted_ = 0;

    return true;
}

void ompl::tools::ThunderDB::setSPARSdb(ompl::tools::SPARSdbPtr &prm)
{
    // OMPL_INFORM("-------------------------------------------------------");
    // OMPL_INFORM("setSPARSdb ");
    // OMPL_INFORM("-------------------------------------------------------");
    spars_ = prm;
}

ompl::tools::SPARSdbPtr &ompl::tools::ThunderDB::getSPARSdb()
{
    return spars_;
}

void ompl::tools::ThunderDB::getAllPlannerDatas(std::vector<ompl::base::PlannerDataPtr> &plannerDatas) const
{
    if (!spars_)
    {
        OMPL_ERROR("SPARSdb planner has not been passed into the ThunderDB yet");
        return;
    }

    auto data(std::make_shared<base::PlannerData>(si_));
    spars_->getPlannerData(*data);
    plannerDatas.push_back(data);

    // OMPL_DEBUG("ThunderDB::getAllPlannerDatas: Number of planner databases found: %d", plannerDatas.size());
}

bool ompl::tools::ThunderDB::findNearestStartGoal(int nearestK, const base::State *start, const base::State *goal,
                                                  ompl::geometric::SPARSdb::CandidateSolution &candidateSolution,
                                                  const base::PlannerTerminationCondition &ptc)
{
    bool result = spars_->getSimilarPaths(nearestK, start, goal, candidateSolution, ptc);

    if (!result)
    {
        OMPL_INFORM("RETRIEVE COULD NOT FIND SOLUTION ");
        OMPL_INFORM("spars::getSimilarPaths() returned false - retrieve could not find solution");
        return false;
    }

    OMPL_INFORM("spars::getSimilarPaths() returned true - found a solution of size %d",
                candidateSolution.getStateCount());
    return true;
}

void ompl::tools::ThunderDB::debugVertex(const ompl::base::PlannerDataVertex &vertex)
{
    debugState(vertex.getState());
}

void ompl::tools::ThunderDB::debugState(const ompl::base::State *state)
{
    si_->printState(state, std::cout);
}
