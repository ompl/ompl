/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, Rice University
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

/* Author: Ioan Sucan */

#include "ompl/benchmark/Benchmark.h"
#include "ompl/util/Time.h"
#include "ompl/util/Memory.h"
#include <boost/lexical_cast.hpp>
#include <boost/progress.hpp>
#include <fstream>
#include <sstream>

#if defined _WIN32
#  include <winsock2.h>
#else
#  include <unistd.h>
#endif

namespace ompl
{

    static std::string getHostname(void)
    {
        char buffer[1024];
        int len = gethostname(buffer, sizeof(buffer));
        if (len != 0)
            return std::string();
        else
            return std::string(buffer);
    }
}

std::string ompl::Benchmark::getResultsFilename(void) const
{
    return "ompl_" + exp_.host + "_" + boost::posix_time::to_iso_extended_string(exp_.startTime) + ".log";
}

bool ompl::Benchmark::saveResultsToFile(const char *filename) const
{
    bool result = false;

    std::ofstream fout(filename);
    if (fout.good())
    {
        result = saveResultsToStream(fout);
        msg_.inform("Results saved to '%s'", filename);
    }
    else
    {
        // try to save to a different file, if we can
        if (getResultsFilename() != std::string(filename))
            result = saveResultsToFile();

        msg_.error("Unable to write results to '%s'", filename);
    }
    return result;
}

bool ompl::Benchmark::saveResultsToFile(void) const
{
    std::string filename = getResultsFilename();
    return saveResultsToFile(filename.c_str());
}

bool ompl::Benchmark::saveResultsToStream(std::ostream &out) const
{
    if (exp_.planners.empty())
    {
        msg_.warn("There is no experimental data to save");
        return false;
    }

    if (!out.good())
    {
        msg_.error("Unable to write to stream");
        return false;
    }

    out << "Experiment " << (exp_.name.empty() ? "NO_NAME" : exp_.name) << std::endl;
    out << "Running on " << (exp_.host.empty() ? "UNKNOWN" : exp_.host) << std::endl;
    out << "Starting at " << boost::posix_time::to_iso_extended_string(exp_.startTime) << std::endl;
    out << "<<<|" << std::endl << exp_.setupInfo << "|>>>" << std::endl;

    out << exp_.maxTime << " seconds per run" << std::endl;
    out << exp_.maxMem << " MB per run" << std::endl;
    out << exp_.totalDuration << " seconds spent to collect the data" << std::endl;
    out << exp_.planners.size() << " planners" << std::endl;

    for (unsigned int i = 0 ; i < exp_.planners.size() ; ++i)
    {
        out << exp_.planners[i].name << std::endl;

        // get names of common properties
        std::vector<std::string> properties;
        for (std::map<std::string, std::string>::const_iterator mit = exp_.planners[i].common.begin() ;
             mit != exp_.planners[i].common.end() ; ++mit)
            properties.push_back(mit->first);
        std::sort(properties.begin(), properties.end());

        // print names & values of common properties
        out << properties.size() << " common properties" << std::endl;
        for (unsigned int k = 0 ; k < properties.size() ; ++k)
        {
            std::map<std::string, std::string>::const_iterator it = exp_.planners[i].common.find(properties[k]);
            out << it->first << " = " << it->second << std::endl;
        }

        // construct the list of all possible properties for all runs
        std::map<std::string, bool> propSeen;
        for (unsigned int j = 0 ; j < exp_.planners[i].runs.size() ; ++j)
            for (std::map<std::string, std::string>::const_iterator mit = exp_.planners[i].runs[j].begin() ;
                 mit != exp_.planners[i].runs[j].end() ; ++mit)
                propSeen[mit->first] = true;

        properties.clear();

        for (std::map<std::string, bool>::iterator it = propSeen.begin() ; it != propSeen.end() ; ++it)
            properties.push_back(it->first);
        std::sort(properties.begin(), properties.end());

        // print the property names
        out << properties.size() << " properties for each run" << std::endl;
        for (unsigned int j = 0 ; j < properties.size() ; ++j)
            out << properties[j] << std::endl;

        // print the data for each run
        out << exp_.planners[i].runs.size() << " runs" << std::endl;
        for (unsigned int j = 0 ; j < exp_.planners[i].runs.size() ; ++j)
        {
            for (unsigned int k = 0 ; k < properties.size() ; ++k)
            {
                std::map<std::string, std::string>::const_iterator it = exp_.planners[i].runs[j].find(properties[k]);
                if (it != exp_.planners[i].runs[j].end())
                    out << it->second;
                out << "; ";
            }
            out << std::endl;
        }

        out << '.' << std::endl;
    }
    return true;
}

namespace ompl
{

    static bool terminationCondition(MemUsage_t maxMem, const time::point &endTime)
    {
        if (time::now() < endTime && getProcessMemoryUsage() < maxMem)
            return false;
        return true;
    }

}

void ompl::Benchmark::benchmark(double maxTime, double maxMem, unsigned int runCount, bool displayProgress)
{
    // sanity checks
    if (gsetup_)
        gsetup_->setup();
    else
        csetup_->setup();

    if (!(gsetup_ ? gsetup_->getGoal() : csetup_->getGoal()))
    {
        msg_.error("No goal defined");
        return;
    }

    if (planners_.empty())
    {
        msg_.error("There are no planners to benchmark");
        return;
    }

    status_.running = true;
    exp_.totalDuration = 0.0;
    exp_.maxTime = maxTime;
    exp_.maxMem = maxMem;
    exp_.host = getHostname();

    exp_.startTime = time::now();

    // clear previous experimental data
    exp_.planners.clear();
    exp_.planners.resize(planners_.size());

    const base::ProblemDefinitionPtr &pdef = gsetup_ ? gsetup_->getProblemDefinition() : csetup_->getProblemDefinition();
    // set up all the planners
    for (unsigned int i = 0 ; i < planners_.size() ; ++i)
    {
        // configure the planner
        planners_[i]->setProblemDefinition(pdef);
        if (!planners_[i]->isSetup())
            planners_[i]->setup();
        exp_.planners[i].name = (gsetup_ ? "geometric_" : "control_") + planners_[i]->getName();
    }

    std::stringstream setupInfo;
    if (gsetup_)
        gsetup_->print(setupInfo);
    else
        csetup_->print(setupInfo);
    exp_.setupInfo = setupInfo.str();

    msg_.inform("Beginning benchmark");

    msg::OutputHandler *oh = msg::getOutputHandler();
    msg::noOutputHandler();

    boost::shared_ptr<boost::progress_display> progress;
    if (displayProgress)
    {
        std::cout << "Running experiment " << exp_.name << std::endl;
        progress.reset(new boost::progress_display(100, std::cout));
    }

    MemUsage_t memStart = getProcessMemoryUsage();

    for (unsigned int i = 0 ; i < planners_.size() ; ++i)
    {
        status_.activePlanner = exp_.planners[i].name;

        // run the planner
        for (unsigned int j = 0 ; j < runCount ; ++j)
        {
            status_.activeRun = j;
            status_.progressPercentage = (double)(100 * (runCount * i + j)) / (double)(planners_.size() * runCount);

            if (displayProgress)
                while (status_.progressPercentage > progress->count())
                    ++(*progress);

            // make sure all planning data structures are cleared
            planners_[i]->clear();
            if (gsetup_)
            {
                gsetup_->getGoal()->clearSolutionPath();
                gsetup_->getSpaceInformation()->getMotionValidator()->resetMotionCounter();
            }
            else
            {
                csetup_->getGoal()->clearSolutionPath();
                csetup_->getSpaceInformation()->getMotionValidator()->resetMotionCounter();
            }

            time::point timeStart = time::now();

            bool solved = false;

            try
            {
                solved = planners_[i]->solve(boost::bind(&terminationCondition,
                                                         memStart + (MemUsage_t)(maxMem * 1024 * 1024),
                                                         time::now() + time::seconds(maxTime)), 0.1);
            }
            catch(std::runtime_error &e)
            {
                std::cerr << std::endl << "There was an error executing planner " << status_.activePlanner <<  ", run = " << j << std::endl;
                std::cerr << "*** " << e.what() << std::endl;
            }

            double timeUsed = time::seconds(time::now() - timeStart);
            MemUsage_t memUsed = getProcessMemoryUsage();
            if (memStart < memUsed)
                memUsed -= memStart;
            else
                memUsed = 0;

            // store results
            try
            {
                RunProperties run;
                run["solved BOOLEAN"] = boost::lexical_cast<std::string>(solved);
                run["time REAL"] = boost::lexical_cast<std::string>(timeUsed);
                run["memory REAL"] = boost::lexical_cast<std::string>((double)memUsed / (1024.0 * 1024.0));
                if (gsetup_)
                {
                    run["valid segment fraction REAL"] = boost::lexical_cast<std::string>(gsetup_->getSpaceInformation()->getMotionValidator()->getValidMotionFraction());
                }
                else
                {
                    run["valid segment fraction REAL"] = boost::lexical_cast<std::string>(csetup_->getSpaceInformation()->getMotionValidator()->getValidMotionFraction());
                }

                if (solved)
                {
                    if (gsetup_)
                    {
                        run["approximate solution BOOLEAN"] = boost::lexical_cast<std::string>(gsetup_->getGoal()->isApproximate());
                        run["solution difference REAL"] = boost::lexical_cast<std::string>(gsetup_->getGoal()->getDifference());
                        run["solution length REAL"] = boost::lexical_cast<std::string>(gsetup_->getSolutionPath().length());
                        run["solution segments INTEGER"] = boost::lexical_cast<std::string>(gsetup_->getSolutionPath().states.size() - 1);
                        run["correct solution BOOLEAN"] = boost::lexical_cast<std::string>(gsetup_->getSolutionPath().check());

                        unsigned int factor = gsetup_->getStateManifold()->getValidSegmentCountFactor();
                        gsetup_->getStateManifold()->setValidSegmentCountFactor(factor * 4);
                        run["correct solution strict BOOLEAN"] = boost::lexical_cast<std::string>(gsetup_->getSolutionPath().check());
                        gsetup_->getStateManifold()->setValidSegmentCountFactor(factor);

                        // simplify solution
                        timeStart = time::now();
                        gsetup_->simplifySolution();
                        timeUsed = time::seconds(time::now() - timeStart);
                        run["simplification time REAL"] = boost::lexical_cast<std::string>(timeUsed);
                        run["simplified solution length REAL"] = boost::lexical_cast<std::string>(gsetup_->getSolutionPath().length());
                        run["simplified solution segments INTEGER"] = boost::lexical_cast<std::string>(gsetup_->getSolutionPath().states.size() - 1);
                        run["simplified correct solution BOOLEAN"] = boost::lexical_cast<std::string>(gsetup_->getSolutionPath().check());
                        gsetup_->getStateManifold()->setValidSegmentCountFactor(factor * 4);
                        run["simplified correct solution strict BOOLEAN"] = boost::lexical_cast<std::string>(gsetup_->getSolutionPath().check());
                        gsetup_->getStateManifold()->setValidSegmentCountFactor(factor);
                    }
                    else
                    {
                        run["approximate solution BOOLEAN"] = boost::lexical_cast<std::string>(csetup_->getGoal()->isApproximate());
                        run["solution difference REAL"] = boost::lexical_cast<std::string>(csetup_->getGoal()->getDifference());
                        run["solution length REAL"] = boost::lexical_cast<std::string>(csetup_->getSolutionPath().length());
                        run["solution segments INTEGER"] = boost::lexical_cast<std::string>(csetup_->getSolutionPath().states.size() - 1);
                        run["correct solution BOOLEAN"] = boost::lexical_cast<std::string>(csetup_->getSolutionPath().check());
                    }
                }

                base::PlannerData pd;
                planners_[i]->getPlannerData(pd);
                run["graph states INTEGER"] = boost::lexical_cast<std::string>(pd.states.size());
                unsigned long edges = 0;
                for (unsigned int k = 0 ; k < pd.edges.size() ; ++k)
                    edges += pd.edges[k].size();
                run["graph motions INTEGER"] = boost::lexical_cast<std::string>(edges);

                for (std::map<std::string, std::string>::const_iterator it = pd.properties.begin() ; it != pd.properties.end() ; ++it)
                    run[it->first] = it->second;

                exp_.planners[i].runs.push_back(run);
            }
            catch(std::runtime_error &e)
            {
                std::cerr << std::endl << "There was an error in the extraction of planner results: planner = " << status_.activePlanner << ", run = " << j << std::endl;
                std::cerr << "*** " << e.what() << std::endl;
            }
        }
    }

    status_.running = false;
    status_.progressPercentage = 100.0;
    if (displayProgress)
    {
        while (status_.progressPercentage > progress->count())
            ++(*progress);
        std::cout << std::endl;
    }

    exp_.totalDuration = time::seconds(time::now() - exp_.startTime);
    msg::useOutputHandler(oh);

    msg_.inform("Benchmark complete");
}
