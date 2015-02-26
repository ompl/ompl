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

/* Author: Ioan Sucan, Luis G. Torres */

#include "ompl/tools/benchmark/Benchmark.h"
#include "ompl/tools/benchmark/MachineSpecs.h"
#include "ompl/util/Time.h"
#include "ompl/config.h"
#include <boost/scoped_ptr.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/progress.hpp>
#include <boost/thread.hpp>
#include <fstream>
#include <sstream>

/// @cond IGNORE
namespace ompl
{
    namespace tools
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
                : benchmark_(benchmark), timeUsed_(0.0), memUsed_(0), useThreads_(useThreads)
            {
            }

            void run(const base::PlannerPtr &planner, const machine::MemUsage_t memStart, const machine::MemUsage_t maxMem, const double maxTime, const double timeBetweenUpdates)
            {
                if (!useThreads_)
                {
                    runThread(planner, memStart + maxMem, time::seconds(maxTime), time::seconds(timeBetweenUpdates));
                    return;
                }

                boost::thread t(boost::bind(&RunPlanner::runThread, this, planner, memStart + maxMem, time::seconds(maxTime), time::seconds(timeBetweenUpdates)));

                // allow 25% more time than originally specified, in order to detect planner termination
#if BOOST_VERSION < 105000
                // For older versions of boost, we have to use this
                // deprecated form of the timed join
                if (!t.timed_join(time::seconds(maxTime * 1.25)))
#else
                if (!t.try_join_for(boost::chrono::duration<double>(maxTime * 1.25)))
#endif
                {
                    status_ = base::PlannerStatus::CRASH;

                    std::stringstream es;
                    es << "Planner " << benchmark_->getStatus().activePlanner << " did not complete run " << benchmark_->getStatus().activeRun
                       << " within the specified amount of time (possible crash). Attempting to force termination of planning thread ..." << std::endl;
                    std::cerr << es.str();
                    OMPL_ERROR(es.str().c_str());

                    t.interrupt();
                    t.join();

                    std::string m = "Planning thread cancelled";
                    std::cerr << m << std::endl;
                    OMPL_ERROR(m.c_str());
                }

                if (memStart < memUsed_)
                    memUsed_ -= memStart;
                else
                    memUsed_ = 0;
            }

            double getTimeUsed() const
            {
                return timeUsed_;
            }

            machine::MemUsage_t getMemUsed() const
            {
                return memUsed_;
            }

            base::PlannerStatus getStatus() const
            {
                return status_;
            }

            const Benchmark::RunProgressData& getRunProgressData() const
            {
                return runProgressData_;
            }

        private:

            void runThread(const base::PlannerPtr &planner, const machine::MemUsage_t maxMem, const time::duration &maxDuration, const time::duration &timeBetweenUpdates)
            {
                time::point timeStart = time::now();

                try
                {
                    base::PlannerTerminationConditionFn ptc = boost::bind(&terminationCondition, maxMem, time::now() + maxDuration);
                    solved_ = false;
                    // Only launch the planner progress property
                    // collector if there is any data for it to report
                    //
                    // \TODO issue here is that at least one sample
                    // always gets taken before planner even starts;
                    // might be worth adding a short wait time before
                    // collector begins sampling
                    boost::scoped_ptr<boost::thread> t;
                    if (planner->getPlannerProgressProperties().size() > 0)
                        t.reset(new boost::thread(boost::bind(&RunPlanner::collectProgressProperties,                                                               this,
                                                              planner->getPlannerProgressProperties(),
                                                              timeBetweenUpdates)));
                    status_ = planner->solve(ptc, 0.1);
                    solvedFlag_.lock();
                    solved_ = true;
                    solvedCondition_.notify_all();
                    solvedFlag_.unlock();
                    if (t)
                        t->join(); // maybe look into interrupting even if planner throws an exception
                }
                catch(std::runtime_error &e)
                {
                    std::stringstream es;
                    es << "There was an error executing planner " << benchmark_->getStatus().activePlanner <<  ", run = " << benchmark_->getStatus().activeRun << std::endl;
                    es << "*** " << e.what() << std::endl;
                    std::cerr << es.str();
                    OMPL_ERROR(es.str().c_str());
                }

                timeUsed_ = time::seconds(time::now() - timeStart);
                memUsed_ = machine::getProcessMemoryUsage();
            }

            void collectProgressProperties(const base::Planner::PlannerProgressProperties& properties,
                                           const time::duration &timePerUpdate)
            {
                time::point timeStart = time::now();

                boost::unique_lock<boost::mutex> ulock(solvedFlag_);
                while (!solved_)
                {
                    if (solvedCondition_.timed_wait(ulock, time::now() + timePerUpdate))
                        return;
                    else
                    {
                        double timeInSeconds = time::seconds(time::now() - timeStart);
                        std::string timeStamp =
                            boost::lexical_cast<std::string>(timeInSeconds);
                        std::map<std::string, std::string> data;
                        data["time REAL"] = timeStamp;
                        for (base::Planner::PlannerProgressProperties::const_iterator item = properties.begin();
                             item != properties.end();
                             ++item)
                        {
                            data[item->first] = item->second();
                        }
                        runProgressData_.push_back(data);
                    }
                }
            }

            const Benchmark    *benchmark_;
            double              timeUsed_;
            machine::MemUsage_t memUsed_;
            base::PlannerStatus status_;
            bool                useThreads_;
            Benchmark::RunProgressData runProgressData_;

            // variables needed for progress property collection
            bool solved_;
            boost::mutex solvedFlag_;
            boost::condition_variable solvedCondition_;
        };

    }
}
/// @endcond

bool ompl::tools::Benchmark::saveResultsToFile(const char *filename) const
{
    bool result = false;

    std::ofstream fout(filename);
    if (fout.good())
    {
        result = saveResultsToStream(fout);
        OMPL_INFORM("Results saved to '%s'", filename);
    }
    else
    {
        // try to save to a different file, if we can
        if (getResultsFilename(exp_) != std::string(filename))
            result = saveResultsToFile();

        OMPL_ERROR("Unable to write results to '%s'", filename);
    }
    return result;
}

bool ompl::tools::Benchmark::saveResultsToFile() const
{
    std::string filename = getResultsFilename(exp_);
    return saveResultsToFile(filename.c_str());
}

bool ompl::tools::Benchmark::saveResultsToStream(std::ostream &out) const
{
    if (exp_.planners.empty())
    {
        OMPL_WARN("There is no experimental data to save");
        return false;
    }

    if (!out.good())
    {
        OMPL_ERROR("Unable to write to stream");
        return false;
    }

    out << "OMPL version " << OMPL_VERSION << std::endl;
    out << "Experiment " << (exp_.name.empty() ? "NO_NAME" : exp_.name) << std::endl;
    out << "Running on " << (exp_.host.empty() ? "UNKNOWN" : exp_.host) << std::endl;
    out << "Starting at " << boost::posix_time::to_iso_extended_string(exp_.startTime) << std::endl;
    out << "<<<|" << std::endl << exp_.setupInfo << "|>>>" << std::endl;
    out << "<<<|" << std::endl << exp_.cpuInfo << "|>>>" << std::endl;

    out << exp_.seed << " is the random seed" << std::endl;
    out << exp_.maxTime << " seconds per run" << std::endl;
    out << exp_.maxMem << " MB per run" << std::endl;
    out << exp_.runCount << " runs per planner" << std::endl;
    out << exp_.totalDuration << " seconds spent to collect the data" << std::endl;

    // change this if more enum types are added
    out << "1 enum type" << std::endl;
    out << "status";
    for (unsigned int i = 0 ; i < base::PlannerStatus::TYPE_COUNT ; ++i)
        out << '|' << base::PlannerStatus(static_cast<base::PlannerStatus::StatusType>(i)).asString();
    out << std::endl;

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

        // print the run progress data if it was reported
        if (exp_.planners[i].runsProgressData.size() > 0)
        {
            // Print number of progress properties
            out << exp_.planners[i].progressPropertyNames.size() << " progress properties for each run" << std::endl;
            // Print progress property names
            for (std::vector<std::string>::const_iterator iter =
                     exp_.planners[i].progressPropertyNames.begin();
                 iter != exp_.planners[i].progressPropertyNames.end();
                 ++iter)
            {
                out << *iter << std::endl;
            }
            // Print progress properties for each run
            out << exp_.planners[i].runsProgressData.size() << " runs" << std::endl;
            for (std::size_t r = 0; r < exp_.planners[i].runsProgressData.size(); ++r)
            {
                // For each time point
                for (std::size_t t = 0; t < exp_.planners[i].runsProgressData[r].size(); ++t)
                {
                    // Print each of the properties at that time point
                    for (std::map<std::string, std::string>::const_iterator iter =
                             exp_.planners[i].runsProgressData[r][t].begin();
                         iter != exp_.planners[i].runsProgressData[r][t].end();
                         ++iter)
                    {
                        out << iter->second << ",";
                    }

                    // Separate time points by semicolons
                    out << ";";
                }

                // Separate runs by newlines
                out << std::endl;
            }
        }

        out << '.' << std::endl;
    }
    return true;
}

void ompl::tools::Benchmark::benchmark(const Request &req)
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
        OMPL_ERROR("No goal defined");
        return;
    }

    if (planners_.empty())
    {
        OMPL_ERROR("There are no planners to benchmark");
        return;
    }

    status_.running = true;
    exp_.totalDuration = 0.0;
    exp_.maxTime = req.maxTime;
    exp_.maxMem = req.maxMem;
    exp_.runCount = req.runCount;
    exp_.host = machine::getHostname();
    exp_.cpuInfo = machine::getCPUInfo();
    exp_.seed = RNG::getSeed();

    exp_.startTime = time::now();

    OMPL_INFORM("Configuring planners ...");

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
        OMPL_INFORM("Configured %s", exp_.planners[i].name.c_str());
    }

    OMPL_INFORM("Done configuring planners.");
    OMPL_INFORM("Saving planner setup information ...");

    std::stringstream setupInfo;
    if (gsetup_)
        gsetup_->print(setupInfo);
    else
        csetup_->print(setupInfo);
    setupInfo << std::endl << "Properties of benchmarked planners:" << std::endl;
    for (unsigned int i = 0 ; i < planners_.size() ; ++i)
        planners_[i]->printProperties(setupInfo);

    exp_.setupInfo = setupInfo.str();

    OMPL_INFORM("Done saving information");

    OMPL_INFORM("Beginning benchmark");
    msg::OutputHandler *oh = msg::getOutputHandler();
    boost::scoped_ptr<msg::OutputHandlerFile> ohf;
    if (req.saveConsoleOutput)
    {
        ohf.reset(new msg::OutputHandlerFile(getConsoleFilename(exp_).c_str()));
        msg::useOutputHandler(ohf.get());
    }
    else
        msg::noOutputHandler();
    OMPL_INFORM("Beginning benchmark");

    boost::scoped_ptr<boost::progress_display> progress;
    if (req.displayProgress)
    {
        std::cout << "Running experiment " << exp_.name << "." << std::endl;
        std::cout << "Each planner will be executed " << req.runCount << " times for at most " << req.maxTime << " seconds. Memory is limited at "
                  << req.maxMem << "MB." << std::endl;
        progress.reset(new boost::progress_display(100, std::cout));
    }

    machine::MemUsage_t memStart = machine::getProcessMemoryUsage();
    machine::MemUsage_t maxMemBytes = (machine::MemUsage_t)(req.maxMem * 1024 * 1024);

    for (unsigned int i = 0 ; i < planners_.size() ; ++i)
    {
        status_.activePlanner = exp_.planners[i].name;
        // execute planner switch event, if set
        try
        {
            if (plannerSwitch_)
            {
                OMPL_INFORM("Executing planner-switch event for planner %s ...", status_.activePlanner.c_str());
                plannerSwitch_(planners_[i]);
                OMPL_INFORM("Completed execution of planner-switch event");
            }
        }
        catch(std::runtime_error &e)
        {
            std::stringstream es;
            es << "There was an error executing the planner-switch event for planner " << status_.activePlanner << std::endl;
            es << "*** " << e.what() << std::endl;
            std::cerr << es.str();
            OMPL_ERROR(es.str().c_str());
        }
        if (gsetup_)
            gsetup_->setup();
        else
            csetup_->setup();
        planners_[i]->params().getParams(exp_.planners[i].common);
        planners_[i]->getSpaceInformation()->params().getParams(exp_.planners[i].common);

        // Add planner progress property names to struct
        exp_.planners[i].progressPropertyNames.push_back("time REAL");
        base::Planner::PlannerProgressProperties::const_iterator iter;
        for (iter = planners_[i]->getPlannerProgressProperties().begin();
             iter != planners_[i]->getPlannerProgressProperties().end();
             ++iter)
        {
            exp_.planners[i].progressPropertyNames.push_back(iter->first);
        }
        std::sort(exp_.planners[i].progressPropertyNames.begin(),
                  exp_.planners[i].progressPropertyNames.end());

        // run the planner
        for (unsigned int j = 0 ; j < req.runCount ; ++j)
        {
            status_.activeRun = j;
            status_.progressPercentage = (double)(100 * (req.runCount * i + j)) / (double)(planners_.size() * req.runCount);

            if (req.displayProgress)
                while (status_.progressPercentage > progress->count())
                    ++(*progress);

            OMPL_INFORM("Preparing for run %d of %s", status_.activeRun, status_.activePlanner.c_str());

            // make sure all planning data structures are cleared
            try
            {
                planners_[i]->clear();
                if (gsetup_)
                {
                    gsetup_->getProblemDefinition()->clearSolutionPaths();
                    gsetup_->getSpaceInformation()->getMotionValidator()->resetMotionCounter();
                }
                else
                {
                    csetup_->getProblemDefinition()->clearSolutionPaths();
                    csetup_->getSpaceInformation()->getMotionValidator()->resetMotionCounter();
                }
            }
            catch(std::runtime_error &e)
            {
                std::stringstream es;
                es << "There was an error while preparing for run " << status_.activeRun << " of planner " << status_.activePlanner << std::endl;
                es << "*** " << e.what() << std::endl;
                std::cerr << es.str();
                OMPL_ERROR(es.str().c_str());
            }

            // execute pre-run event, if set
            try
            {
                if (preRun_)
                {
                    OMPL_INFORM("Executing pre-run event for run %d of planner %s ...", status_.activeRun, status_.activePlanner.c_str());
                    preRun_(planners_[i]);
                    OMPL_INFORM("Completed execution of pre-run event");
                }
            }
            catch(std::runtime_error &e)
            {
                std::stringstream es;
                es << "There was an error executing the pre-run event for run " << status_.activeRun << " of planner " << status_.activePlanner << std::endl;
                es << "*** " << e.what() << std::endl;
                std::cerr << es.str();
                OMPL_ERROR(es.str().c_str());
            }

            RunPlanner rp(this, req.useThreads);
            rp.run(planners_[i], memStart, maxMemBytes, req.maxTime, req.timeBetweenUpdates);
            bool solved = gsetup_ ? gsetup_->haveSolutionPath() : csetup_->haveSolutionPath();

            // store results
            try
            {
                RunProperties run;

                run["time REAL"] = boost::lexical_cast<std::string>(rp.getTimeUsed());
                run["memory REAL"] = boost::lexical_cast<std::string>((double)rp.getMemUsed() / (1024.0 * 1024.0));
                run["status ENUM"] = boost::lexical_cast<std::string>((int)static_cast<base::PlannerStatus::StatusType>(rp.getStatus()));
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
                        run["approximate solution BOOLEAN"] = boost::lexical_cast<std::string>(gsetup_->getProblemDefinition()->hasApproximateSolution());
                        run["solution difference REAL"] = boost::lexical_cast<std::string>(gsetup_->getProblemDefinition()->getSolutionDifference());
                        run["solution length REAL"] = boost::lexical_cast<std::string>(gsetup_->getSolutionPath().length());
                        run["solution smoothness REAL"] = boost::lexical_cast<std::string>(gsetup_->getSolutionPath().smoothness());
                        run["solution clearance REAL"] = boost::lexical_cast<std::string>(gsetup_->getSolutionPath().clearance());
                        run["solution segments INTEGER"] = boost::lexical_cast<std::string>(gsetup_->getSolutionPath().getStateCount() - 1);
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
                        run["simplified solution segments INTEGER"] = boost::lexical_cast<std::string>(gsetup_->getSolutionPath().getStateCount() - 1);
                        run["simplified correct solution BOOLEAN"] = boost::lexical_cast<std::string>(gsetup_->getSolutionPath().check());
                        gsetup_->getStateSpace()->setValidSegmentCountFactor(factor * 4);
                        run["simplified correct solution strict BOOLEAN"] = boost::lexical_cast<std::string>(gsetup_->getSolutionPath().check());
                        gsetup_->getStateSpace()->setValidSegmentCountFactor(factor);
                    }
                    else
                    {
                        run["approximate solution BOOLEAN"] = boost::lexical_cast<std::string>(csetup_->getProblemDefinition()->hasApproximateSolution());
                        run["solution difference REAL"] = boost::lexical_cast<std::string>(csetup_->getProblemDefinition()->getSolutionDifference());
                        run["solution length REAL"] = boost::lexical_cast<std::string>(csetup_->getSolutionPath().length());
                        run["solution clearance REAL"] = boost::lexical_cast<std::string>(csetup_->getSolutionPath().asGeometric().clearance());
                        run["solution segments INTEGER"] = boost::lexical_cast<std::string>(csetup_->getSolutionPath().getControlCount());
                        run["correct solution BOOLEAN"] = boost::lexical_cast<std::string>(csetup_->getSolutionPath().check());
                    }
                }

                base::PlannerData pd (gsetup_ ? gsetup_->getSpaceInformation() : csetup_->getSpaceInformation());
                planners_[i]->getPlannerData(pd);
                run["graph states INTEGER"] = boost::lexical_cast<std::string>(pd.numVertices());
                run["graph motions INTEGER"] = boost::lexical_cast<std::string>(pd.numEdges());

                for (std::map<std::string, std::string>::const_iterator it = pd.properties.begin() ; it != pd.properties.end() ; ++it)
                    run[it->first] = it->second;

                // execute post-run event, if set
                try
                {
                    if (postRun_)
                    {
                        OMPL_INFORM("Executing post-run event for run %d of planner %s ...", status_.activeRun, status_.activePlanner.c_str());
                        postRun_(planners_[i], run);
                        OMPL_INFORM("Completed execution of post-run event");
                    }
                }
                catch(std::runtime_error &e)
                {
                    std::stringstream es;
                    es << "There was an error in the execution of the post-run event for run " << status_.activeRun << " of planner " << status_.activePlanner << std::endl;
                    es << "*** " << e.what() << std::endl;
                    std::cerr << es.str();
                    OMPL_ERROR(es.str().c_str());
                }

                exp_.planners[i].runs.push_back(run);

                // Add planner progress data from the planner progress
                // collector if there was anything to report
                if (planners_[i]->getPlannerProgressProperties().size() > 0)
                {
                    exp_.planners[i].runsProgressData.push_back(rp.getRunProgressData());
                }
            }
            catch(std::runtime_error &e)
            {
                std::stringstream es;
                es << "There was an error in the extraction of planner results: planner = " << status_.activePlanner << ", run = " << status_.activePlanner << std::endl;
                es << "*** " << e.what() << std::endl;
                std::cerr << es.str();
                OMPL_ERROR(es.str().c_str());
            }
        }
    }

    status_.running = false;
    status_.progressPercentage = 100.0;
    if (req.displayProgress)
    {
        while (status_.progressPercentage > progress->count())
            ++(*progress);
        std::cout << std::endl;
    }

    exp_.totalDuration = time::seconds(time::now() - exp_.startTime);

    OMPL_INFORM("Benchmark complete");
    msg::useOutputHandler(oh);
    OMPL_INFORM("Benchmark complete");
}
