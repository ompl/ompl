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

#include "ompl/tools/benchmark/Benchmark.h"
#include "ompl/tools/benchmark/MachineSpecs.h"
#include "ompl/util/Time.h"
#include <boost/lexical_cast.hpp>
#include <boost/progress.hpp>
#include <fstream>
#include <sstream>

/// @cond IGNORE
namespace ompl
{
    /** \brief Propose a name for a file in which results should be saved, based on the date and hostname of the experiment */
    static std::string getResultsFilename(const Benchmark::CompleteExperiment &exp)
    {
        return "ompl_" + exp.host + "_" + boost::posix_time::to_iso_extended_string(exp.startTime) + ".log";
    }

    /** \brief Propose a name for a file in which console output should be saved, based on the date and hostname of the experiment */
    static std::string getConsoleFilename(const Benchmark::CompleteExperiment &exp)
    {
        return "ompl_" + exp.host + "_" + boost::posix_time::to_iso_extended_string(exp.startTime) + ".console";
    }

    static bool terminationCondition(const machine::MemUsage_t maxMem, const time::point &endTime)
    {
        if (time::now() < endTime && machine::getProcessMemoryUsage() < maxMem)
            return false;
        return true;
    }

    class RunPlanner
    {
    public:

        RunPlanner(const Benchmark *benchmark, bool useThreads)
            : benchmark_(benchmark), timeUsed_(0.0), memUsed_(0), crashed_(false), useThreads_(useThreads)
        {
        }

        void run(const base::PlannerPtr &planner, const machine::MemUsage_t memStart, const machine::MemUsage_t maxMem, const double maxTime)
        {
            if (!useThreads_)
            {
                runThread(planner, memStart + maxMem, time::seconds(maxTime));
                return;
            }

            boost::thread t(boost::bind(&RunPlanner::runThread, this, planner, memStart + maxMem, time::seconds(maxTime)));

            // allow 25% more time than originally specified, in order to detect planner termination
            if (!t.timed_join(time::seconds(maxTime * 1.25)))
            {
                crashed_ = true;

                std::stringstream es;
                es << "Planner " << benchmark_->getStatus().activePlanner << " did not complete run " << benchmark_->getStatus().activeRun
                   << " within the specified amount of time (possible crash). Attempting to force termination of planning thread ..." << std::endl;
                std::cerr << es.str();
                msg_.error(es.str());

                t.interrupt();
                t.join();

                std::string m = "Planning thread cancelled";
                std::cerr << m << std::endl;
                msg_.error(m);
            }

            if (memStart < memUsed_)
                memUsed_ -= memStart;
            else
                memUsed_ = 0;
        }

        double getTimeUsed(void) const
        {
            return timeUsed_;
        }

        machine::MemUsage_t getMemUsed(void) const
        {
            return memUsed_;
        }

        bool crashed(void) const
        {
            return crashed_;
        }

    private:

        void runThread(const base::PlannerPtr &planner, const machine::MemUsage_t maxMem, const time::duration &maxDuration)
        {
            time::point timeStart = time::now();

            try
            {
                base::PlannerTerminationConditionFn ptc = boost::bind(&terminationCondition, maxMem, time::now() + maxDuration);
                planner->solve(ptc, 0.1);
            }
            catch(std::runtime_error &e)
            {
                std::stringstream es;
                es << "There was an error executing planner " << benchmark_->getStatus().activePlanner <<  ", run = " << benchmark_->getStatus().activeRun << std::endl;
                es << "*** " << e.what() << std::endl;
                std::cerr << es.str();
                msg_.error(es.str());
            }

            timeUsed_ = time::seconds(time::now() - timeStart);
            memUsed_ = machine::getProcessMemoryUsage();
        }

        const Benchmark    *benchmark_;
        double              timeUsed_;
        machine::MemUsage_t memUsed_;
        bool                crashed_;
        bool                useThreads_;
        msg::Interface      msg_;
    };

}
/// @endcond

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
        if (getResultsFilename(exp_) != std::string(filename))
            result = saveResultsToFile();

        msg_.error("Unable to write results to '%s'", filename);
    }
    return result;
}

bool ompl::Benchmark::saveResultsToFile(void) const
{
    std::string filename = getResultsFilename(exp_);
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

    out << exp_.seed << " is the random seed" << std::endl;
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

void ompl::Benchmark::benchmark(double maxTime, double maxMem, unsigned int runCount, bool displayProgress, bool useThreads)
{
    // sanity checks
    if (gsetup_)
    {
        if (!gsetup_->getSpaceInformation()->isSetup())
            gsetup_->getSpaceInformation()->setup();
    }
    else
    {
        if (!csetup_->getSpaceInformation()->isSetup())
            csetup_->getSpaceInformation()->setup();
    }

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
    exp_.host = machine::getHostname();
    exp_.seed = RNG::getSeed();

    exp_.startTime = time::now();

    msg_.inform("Configuring planners ...");

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

    msg_.inform("Done configuring planners.");
    msg_.inform("Saving planner setup information ...");

    std::stringstream setupInfo;
    if (gsetup_)
        gsetup_->print(setupInfo);
    else
        csetup_->print(setupInfo);
    setupInfo << std::endl << "Planner properties:" << std::endl;
    for (unsigned int i = 0 ; i < planners_.size() ; ++i)
    {
        planners_[i]->printProperties(setupInfo);
        planners_[i]->getParams(exp_.planners[i].common);
    }
    exp_.setupInfo = setupInfo.str();

    msg_.inform("Done saving information");

    msg_.inform("Beginning benchmark");
    msg::OutputHandler *oh = msg::getOutputHandler();
    msg::OutputHandlerFile ohf(getConsoleFilename(exp_).c_str());
    msg::useOutputHandler(&ohf);
    msg_.inform("Beginning benchmark");

    boost::shared_ptr<boost::progress_display> progress;
    if (displayProgress)
    {
        std::cout << "Running experiment " << exp_.name << "." << std::endl;
        std::cout << "Each planner will be executed " << runCount << " times for at most " << maxTime << " seconds. Memory is limited at " << maxMem << "MB." << std::endl;
        progress.reset(new boost::progress_display(100, std::cout));
    }

    machine::MemUsage_t memStart = machine::getProcessMemoryUsage();
    machine::MemUsage_t maxMemBytes = (machine::MemUsage_t)(maxMem * 1024 * 1024);

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

            msg_.inform("Preparing for run %d of %s", status_.activeRun, status_.activePlanner.c_str());

            // make sure all planning data structures are cleared
            try
            {
                planners_[i]->clear();
                if (gsetup_)
                {
                    gsetup_->getGoal()->clearSolutionPaths();
                    gsetup_->getSpaceInformation()->getMotionValidator()->resetMotionCounter();
                }
                else
                {
                    csetup_->getGoal()->clearSolutionPaths();
                    csetup_->getSpaceInformation()->getMotionValidator()->resetMotionCounter();
                }
            }
            catch(std::runtime_error &e)
            {
                std::stringstream es;
                es << "There was an error while preparing for run " << status_.activeRun << " of planner " << status_.activePlanner << std::endl;
                es << "*** " << e.what() << std::endl;
                std::cerr << es.str();
                msg_.error(es.str());
            }

            // execute pre-run event, if set
            try
            {
                if (preRun_)
                {
                    msg_.inform("Executing pre-run event for run %d of planner %s ...", status_.activeRun, status_.activePlanner.c_str());
                    preRun_(planners_[i]);
                    msg_.inform("Completed execution of pre-run event");
                }
            }
            catch(std::runtime_error &e)
            {
                std::stringstream es;
                es << "There was an error executing the pre-run event for run " << status_.activeRun << " of planner " << status_.activePlanner << std::endl;
                es << "*** " << e.what() << std::endl;
                std::cerr << es.str();
                msg_.error(es.str());
            }

            RunPlanner rp(this, useThreads);
            rp.run(planners_[i], memStart, maxMemBytes, maxTime);
            bool solved = gsetup_ ? gsetup_->haveSolutionPath() : csetup_->haveSolutionPath();

            // store results
            try
            {
                RunProperties run;

                run["crashed BOOLEAN"] = boost::lexical_cast<std::string>(rp.crashed());
                run["time REAL"] = boost::lexical_cast<std::string>(rp.getTimeUsed());
                run["memory REAL"] = boost::lexical_cast<std::string>((double)rp.getMemUsed() / (1024.0 * 1024.0));
                if (gsetup_)
                {
                    run["solved BOOLEAN"] = boost::lexical_cast<std::string>(gsetup_->haveExactSolutionPath());
                    run["valid segment fraction REAL"] = boost::lexical_cast<std::string>(gsetup_->getSpaceInformation()->getMotionValidator()->getValidMotionFraction());
                }
                else
                {
                    run["solved BOOLEAN"] = boost::lexical_cast<std::string>(csetup_->haveExactSolutionPath());
                    run["valid segment fraction REAL"] = boost::lexical_cast<std::string>(csetup_->getSpaceInformation()->getMotionValidator()->getValidMotionFraction());
                }

                if (solved)
                {
                    if (gsetup_)
                    {
                        run["approximate solution BOOLEAN"] = boost::lexical_cast<std::string>(gsetup_->getGoal()->isApproximate());
                        run["solution difference REAL"] = boost::lexical_cast<std::string>(gsetup_->getGoal()->getDifference());
                        run["solution length REAL"] = boost::lexical_cast<std::string>(gsetup_->getSolutionPath().length());
                        run["solution smoothness REAL"] = boost::lexical_cast<std::string>(gsetup_->getSolutionPath().smoothness());
                        run["solution clearance REAL"] = boost::lexical_cast<std::string>(gsetup_->getSolutionPath().clearance());
                        run["solution segments INTEGER"] = boost::lexical_cast<std::string>(gsetup_->getSolutionPath().states.size() - 1);
                        run["correct solution BOOLEAN"] = boost::lexical_cast<std::string>(gsetup_->getSolutionPath().check());

                        unsigned int factor = gsetup_->getStateSpace()->getValidSegmentCountFactor();
                        gsetup_->getStateSpace()->setValidSegmentCountFactor(factor * 4);
                        run["correct solution strict BOOLEAN"] = boost::lexical_cast<std::string>(gsetup_->getSolutionPath().check());
                        gsetup_->getStateSpace()->setValidSegmentCountFactor(factor);

                        // simplify solution
                        time::point timeStart = time::now();
                        gsetup_->simplifySolution();
                        double timeUsed = time::seconds(time::now() - timeStart);
                        run["simplification time REAL"] = boost::lexical_cast<std::string>(timeUsed);
                        run["simplified solution length REAL"] = boost::lexical_cast<std::string>(gsetup_->getSolutionPath().length());
                        run["simplified solution smoothness REAL"] = boost::lexical_cast<std::string>(gsetup_->getSolutionPath().smoothness());
                        run["simplified solution clearance REAL"] = boost::lexical_cast<std::string>(gsetup_->getSolutionPath().clearance());
                        run["simplified solution segments INTEGER"] = boost::lexical_cast<std::string>(gsetup_->getSolutionPath().states.size() - 1);
                        run["simplified correct solution BOOLEAN"] = boost::lexical_cast<std::string>(gsetup_->getSolutionPath().check());
                        gsetup_->getStateSpace()->setValidSegmentCountFactor(factor * 4);
                        run["simplified correct solution strict BOOLEAN"] = boost::lexical_cast<std::string>(gsetup_->getSolutionPath().check());
                        gsetup_->getStateSpace()->setValidSegmentCountFactor(factor);
                    }
                    else
                    {
                        run["approximate solution BOOLEAN"] = boost::lexical_cast<std::string>(csetup_->getGoal()->isApproximate());
                        run["solution difference REAL"] = boost::lexical_cast<std::string>(csetup_->getGoal()->getDifference());
                        run["solution length REAL"] = boost::lexical_cast<std::string>(csetup_->getSolutionPath().length());
                        run["solution clearance REAL"] = boost::lexical_cast<std::string>(csetup_->getSolutionPath().asGeometric().clearance());
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

                // execute post-run event, if set
                try
                {
                    if (postRun_)
                    {
                        msg_.inform("Executing post-run event for run %d of planner %s ...", status_.activeRun, status_.activePlanner.c_str());
                        postRun_(planners_[i], run);
                        msg_.inform("Completed execution of post-run event");
                    }
                }
                catch(std::runtime_error &e)
                {
                    std::stringstream es;
                    es << "There was an error in the execution of the post-run event for run " << status_.activeRun << " of planner " << status_.activePlanner << std::endl;
                    es << "*** " << e.what() << std::endl;
                    std::cerr << es.str();
                    msg_.error(es.str());
                }

                exp_.planners[i].runs.push_back(run);
            }
            catch(std::runtime_error &e)
            {
                std::stringstream es;
                es << "There was an error in the extraction of planner results: planner = " << status_.activePlanner << ", run = " << status_.activePlanner << std::endl;
                es << "*** " << e.what() << std::endl;
                std::cerr << es.str();
                msg_.error(es.str());
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

    msg_.inform("Benchmark complete");
    msg::useOutputHandler(oh);
    msg_.inform("Benchmark complete");
}
