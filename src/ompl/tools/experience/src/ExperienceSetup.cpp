/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, University of Colorado, Boulder
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
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
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

/* Author: Dave Coleman
   Desc:   Common functions for experience-based planning. Used by lighting and thunder
*/

#include "ompl/tools/experience/ExperienceSetup.h"
#include "ompl/tools/multiplan/ParallelPlan.h"

// Boost
#include <boost/filesystem.hpp>

ompl::tools::ExperienceSetup::ExperienceSetup(const base::SpaceInformationPtr &si)
  : ompl::geometric::SimpleSetup(si)
{
  logInitialize();
};

ompl::tools::ExperienceSetup::ExperienceSetup(const base::StateSpacePtr &space)
  : ompl::geometric::SimpleSetup(space)
{
  logInitialize();
};

void ompl::tools::ExperienceSetup::logInitialize()
{
  // Header of CSV file
  csvDataLogStream_
    // Times
    << "planning_time,insertion_time,"
    // Solution properties
    << "planner,result,is_saved,"
    // Failure booleans
    << "approximate,too_short,insertion_failed,"
    // Lightning properties
    << "score,"
    // Thunder (SPARS) properties
    << "num_vertices,num_edges,num_connected_components,"
    // Hack for using python cause im lazy right now
    << "total_experiences,total_scratch,total_recall,total_failed,total_approximate,"
    << "total_too_short,total_insertion_failed,"
    << "avg_planning_time,avg_insertion_time" 
    << std::endl;
}

void ompl::tools::ExperienceSetup::convertLogToString(const ExperienceLog &log)
{
  csvDataLogStream_ 
    << log.planning_time << ","
    << log.insertion_time << ","
    << log.planner << ","
    << log.result << ","
    << log.is_saved << ","
    << log.approximate << ","
    << log.too_short << ","
    << log.insertion_failed << ","
    << log.score << ","
    << log.num_vertices << ","
    << log.num_edges << ","
    << log.num_connected_components
    << std::endl;
}

void ompl::tools::ExperienceSetup::saveDataLog(std::ostream &out)
{
    // Export to file and clear the stream
    out << csvDataLogStream_.str();
    csvDataLogStream_.str("");
}

bool ompl::tools::ExperienceSetup::getFilePath(const std::string &databaseName, const std::string &databaseDirectory)
{
    namespace fs = boost::filesystem;

    // Check that the directory exists, if not, create it
    fs::path rootPath;
    if (!std::string(getenv("HOME")).empty())
        rootPath = fs::path(getenv("HOME")); // Support Linux/Mac
    else if (!std::string(getenv("HOMEPATH")).empty())
        rootPath = fs::path(getenv("HOMEPATH")); // Support Windows
    else
    {
        OMPL_WARN("Unable to find a home path for this computer");
        rootPath = fs::path("");
    }

    rootPath = rootPath / fs::path(databaseDirectory);

    boost::system::error_code returnedError;
    fs::create_directories( rootPath, returnedError );

    if ( returnedError )
    {
        //did not successfully create directories
        OMPL_ERROR("Unable to create directory %s", databaseDirectory.c_str());
        return false;
    }

    //directories successfully created, append the group name as the file name
    rootPath = rootPath / fs::path(databaseName + ".ompl");
    filePath_ = rootPath.string();
    OMPL_INFORM("Setting database to %s", filePath_.c_str());

    return true;
}

const std::string& ompl::tools::ExperienceSetup::getFilePath() const
{
    return filePath_;
}

void ompl::tools::ExperienceSetup::enablePlanningFromRecall(bool enable)
{
    // Remember state
    recallEnabled_ = enable;

    // Flag the planners as possibly misconfigured
    configured_ = false;
}

void ompl::tools::ExperienceSetup::enablePlanningFromScratch(bool enable)
{
    // Remember state
    scratchEnabled_ = enable;

    // Flag the planners as possibly misconfigured
    configured_ = false;
}

