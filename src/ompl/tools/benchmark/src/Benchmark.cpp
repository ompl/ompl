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
#include "ompl/util/String.h"
#include <boost/scoped_ptr.hpp>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <fstream>
#include <sstream>

/// @cond IGNORE
namespace ompl
{
    namespace tools
    {
        /** \brief Propose a name for a file in which results should be saved, based on the date and hostname of the
         * experiment */
        static std::string getResultsFilename(const Benchmark::CompleteExperiment &exp)
        {
            return "ompl_" + exp.host + "_" + time::as_string(exp.startTime) + ".log";
        }

        /** \brief Propose a name for a file in which console output should be saved, based on the date and hostname of
         * the experiment */
        static std::string getConsoleFilename(const Benchmark::CompleteExperiment &exp)
        {
            return "ompl_" + exp.host + "_" + time::as_string(exp.startTime) + ".console";
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
            RunPlanner(const Benchmark *benchmark)
              : benchmark_(benchmark), timeUsed_(0.0), memUsed_(0)
            {
            }

            void run(const base::PlannerPtr &planner, const machine::MemUsage_t memStart,
                     const machine::MemUsage_t maxMem, const double maxTime, const double timeBetweenUpdates)
            {
                runThread(planner, memStart + maxMem, time::seconds(maxTime), time::seconds(timeBetweenUpdates));
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

            const Benchmark::RunProgressData &getRunProgressData() const
            {
                return runProgressData_;
            }

        private:
            void runThread(const base::PlannerPtr &planner, const machine::MemUsage_t maxMem,
                           const time::duration &maxDuration, const time::duration &timeBetweenUpdates)
            {
                time::point timeStart = time::now();

                try
                {
                    const time::point endtime = time::now() + maxDuration;
                    base::PlannerTerminationConditionFn ptc([maxMem, endtime]
                                                            {
                                                                return terminationCondition(maxMem, endtime);
                                                            });
                    solved_ = false;
                    // Only launch the planner progress property
                    // collector if there is any data for it to report
                    //
                    // \todo issue here is that at least one sample
                    // always gets taken before planner even starts;
                    // might be worth adding a short wait time before
                    // collector begins sampling
                    boost::scoped_ptr<std::thread> t;
                    if (planner->getPlannerProgressProperties().size() > 0)
                        t.reset(new std::thread([this, &planner, timeBetweenUpdates]
                                                {
                                                    collectProgressProperties(planner->getPlannerProgressProperties(),
                                                                              timeBetweenUpdates);
                                                }));
                    status_ = planner->solve(ptc, 0.1);
                    solvedFlag_.lock();
                    solved_ = true;
                    solvedCondition_.notify_all();
                    solvedFlag_.unlock();
                    if (t)
                        t->join();  // maybe look into interrupting even if planner throws an exception
                }
                catch (std::runtime_error &e)
                {
                    std::stringstream es;
                    es << "There was an error executing planner " << benchmark_->getStatus().activePlanner
                       << ", run = " << benchmark_->getStatus().activeRun << std::endl;
                    es << "*** " << e.what() << std::endl;
                    std::cerr << es.str();
                    OMPL_ERROR(es.str().c_str());
                }

                timeUsed_ = time::seconds(time::now() - timeStart);
                memUsed_ = machine::getProcessMemoryUsage();
            }

            void collectProgressProperties(const base::Planner::PlannerProgressProperties &properties,
                                           const time::duration &timePerUpdate)
            {
                time::point timeStart = time::now();

                std::unique_lock<std::mutex> ulock(solvedFlag_);
                while (!solved_)
                {
                    if (solvedCondition_.wait_for(ulock, timePerUpdate) == std::cv_status::no_timeout)
                        return;
                    else
                    {
                        double timeInSeconds = time::seconds(time::now() - timeStart);
                        std::string timeStamp = ompl::toString(timeInSeconds);
                        std::map<std::string, std::string> data;
                        data["time REAL"] = timeStamp;
                        for (const auto &property : properties)
                        {
                            data[property.first] = property.second();
                        }
                        runProgressData_.push_back(data);
                    }
                }
            }

            const Benchmark *benchmark_;
            double timeUsed_;
            machine::MemUsage_t memUsed_;
            base::PlannerStatus status_;
            Benchmark::RunProgressData runProgressData_;

            // variables needed for progress property collection
            bool solved_;
            std::mutex solvedFlag_;
            std::condition_variable solvedCondition_;
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

    out << exp_.parameters.size() << " experiment properties" << std::endl;
    for (const auto &parameter : exp_.parameters)
        out << parameter.first << " = " << parameter.second << std::endl;

    out << "Running on " << (exp_.host.empty() ? "UNKNOWN" : exp_.host) << std::endl;
    out << "Starting at " << time::as_string(exp_.startTime) << std::endl;
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
    for (unsigned int i = 0; i < base::PlannerStatus::TYPE_COUNT; ++i)
        out << '|' << base::PlannerStatus(static_cast<base::PlannerStatus::StatusType>(i)).asString();
    out << std::endl;

    out << exp_.planners.size() << " planners" << std::endl;

    for (const auto &planner : exp_.planners)
    {
        out << planner.name << std::endl;

        // get names of common properties
        std::vector<std::string> properties;
        for (auto &property : planner.common)
            properties.push_back(property.first);
        std::sort(properties.begin(), properties.end());

        // print names & values of common properties
        out << properties.size() << " common properties" << std::endl;
        for (auto &property : properties)
        {
            auto it = planner.common.find(property);
            out << it->first << " = " << it->second << std::endl;
        }

        // construct the list of all possible properties for all runs
        std::map<std::string, bool> propSeen;
        for (auto &run : planner.runs)
            for (auto &property : run)
                propSeen[property.first] = true;

        properties.clear();

        for (auto &it : propSeen)
            properties.push_back(it.first);
        std::sort(properties.begin(), properties.end());

        // print the property names
        out << properties.size() << " properties for each run" << std::endl;
        for (auto &property : properties)
            out << property << std::endl;

        // print the data for each run
        out << planner.runs.size() << " runs" << std::endl;
        for (auto &run : planner.runs)
        {
            for (auto &property : properties)
            {
                auto it = run.find(property);
                if (it != run.end())
                    out << it->second;
                out << "; ";
            }
            out << std::endl;
        }

        // print the run progress data if it was reported
        if (planner.runsProgressData.size() > 0)
        {
            // Print number of progress properties
            out << planner.progressPropertyNames.size() << " progress properties for each run" << std::endl;
            // Print progress property names
            for (const auto &progPropName : planner.progressPropertyNames)
            {
                out << progPropName << std::endl;
            }
            // Print progress properties for each run
            out << planner.runsProgressData.size() << " runs" << std::endl;
            for (const auto &r : planner.runsProgressData)
            {
                // For each time point
                for (const auto &t : r)
                {
                    // Print each of the properties at that time point
                    for (const auto &iter : t)
                    {
                        out << iter.second << ",";
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

    const base::ProblemDefinitionPtr &pdef =
        gsetup_ ? gsetup_->getProblemDefinition() : csetup_->getProblemDefinition();
    // set up all the planners
    for (unsigned int i = 0; i < planners_.size(); ++i)
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
    for (auto &planner : planners_)
        planner->printProperties(setupInfo);

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

    boost::scoped_ptr<ompl::time::ProgressDisplay> progress;
    if (req.displayProgress)
    {
        std::cout << "Running experiment " << exp_.name << "." << std::endl;
        if (req.runCount)
            std::cout << "Each planner will be executed " << req.runCount << " times for at most " << req.maxTime << " seconds.";
        else
            std::cout << "Each planner will be executed as many times as possible within " << req.maxTime << " seconds.";
        std::cout << " Memory is limited at " << req.maxMem << "MB." << std::endl;
        progress.reset(new ompl::time::ProgressDisplay);
    }

    machine::MemUsage_t memStart = machine::getProcessMemoryUsage();
    auto maxMemBytes = (machine::MemUsage_t)(req.maxMem * 1024 * 1024);

    for (unsigned int i = 0; i < planners_.size(); ++i)
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
        catch (std::runtime_error &e)
        {
            std::stringstream es;
            es << "There was an error executing the planner-switch event for planner " << status_.activePlanner
               << std::endl;
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
        exp_.planners[i].progressPropertyNames.emplace_back("time REAL");
        for (const auto &property : planners_[i]->getPlannerProgressProperties())
        {
            exp_.planners[i].progressPropertyNames.push_back(property.first);
        }
        std::sort(exp_.planners[i].progressPropertyNames.begin(), exp_.planners[i].progressPropertyNames.end());

        // run the planner
        double maxTime = req.maxTime;
        unsigned int j = 0;
        while (true)
        {
            status_.activeRun = j;
            status_.progressPercentage = req.runCount ?
                (double)(100 * (req.runCount * i + j)) / (double)(planners_.size() * req.runCount) :
                (double)(100 * i) / (double)(planners_.size());

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
            catch (std::runtime_error &e)
            {
                std::stringstream es;
                es << "There was an error while preparing for run " << status_.activeRun << " of planner "
                   << status_.activePlanner << std::endl;
                es << "*** " << e.what() << std::endl;
                std::cerr << es.str();
                OMPL_ERROR(es.str().c_str());
            }

            // execute pre-run event, if set
            try
            {
                if (preRun_)
                {
                    OMPL_INFORM("Executing pre-run event for run %d of planner %s ...", status_.activeRun,
                                status_.activePlanner.c_str());
                    preRun_(planners_[i]);
                    OMPL_INFORM("Completed execution of pre-run event");
                }
            }
            catch (std::runtime_error &e)
            {
                std::stringstream es;
                es << "There was an error executing the pre-run event for run " << status_.activeRun << " of planner "
                   << status_.activePlanner << std::endl;
                es << "*** " << e.what() << std::endl;
                std::cerr << es.str();
                OMPL_ERROR(es.str().c_str());
            }

            RunPlanner rp(this);
            rp.run(planners_[i], memStart, maxMemBytes, maxTime, req.timeBetweenUpdates);
            bool solved = gsetup_ ? gsetup_->haveSolutionPath() : csetup_->haveSolutionPath();

            // store results
            try
            {
                RunProperties run;

                run["time REAL"] = ompl::toString(rp.getTimeUsed());
                run["memory REAL"] = ompl::toString((double)rp.getMemUsed() / (1024.0 * 1024.0));
                run["status ENUM"] = std::to_string((int)static_cast<base::PlannerStatus::StatusType>(rp.getStatus()));
                if (gsetup_)
                {
                    run["solved BOOLEAN"] = std::to_string(gsetup_->haveExactSolutionPath());
                    run["valid segment fraction REAL"] =
                        ompl::toString(gsetup_->getSpaceInformation()->getMotionValidator()->getValidMotionFraction());
                }
                else
                {
                    run["solved BOOLEAN"] = std::to_string(csetup_->haveExactSolutionPath());
                    run["valid segment fraction REAL"] =
                        ompl::toString(csetup_->getSpaceInformation()->getMotionValidator()->getValidMotionFraction());
                }

                if (solved)
                {
                    if (gsetup_)
                    {
                        run["approximate solution BOOLEAN"] =
                            std::to_string(gsetup_->getProblemDefinition()->hasApproximateSolution());
                        run["solution difference REAL"] =
                            ompl::toString(gsetup_->getProblemDefinition()->getSolutionDifference());
                        run["solution length REAL"] = ompl::toString(gsetup_->getSolutionPath().length());
                        run["solution smoothness REAL"] = ompl::toString(gsetup_->getSolutionPath().smoothness());
                        run["solution clearance REAL"] = ompl::toString(gsetup_->getSolutionPath().clearance());
                        run["solution segments INTEGER"] =
                            std::to_string(gsetup_->getSolutionPath().getStateCount() - 1);
                        run["correct solution BOOLEAN"] = std::to_string(gsetup_->getSolutionPath().check());

                        unsigned int factor = gsetup_->getStateSpace()->getValidSegmentCountFactor();
                        gsetup_->getStateSpace()->setValidSegmentCountFactor(factor * 4);
                        run["correct solution strict BOOLEAN"] = std::to_string(gsetup_->getSolutionPath().check());
                        gsetup_->getStateSpace()->setValidSegmentCountFactor(factor);

                        if (req.simplify)
                        {
                            // simplify solution
                            time::point timeStart = time::now();
                            gsetup_->simplifySolution();
                            double timeUsed = time::seconds(time::now() - timeStart);
                            run["simplification time REAL"] = ompl::toString(timeUsed);
                            run["simplified solution length REAL"] =
                                ompl::toString(gsetup_->getSolutionPath().length());
                            run["simplified solution smoothness REAL"] =
                                ompl::toString(gsetup_->getSolutionPath().smoothness());
                            run["simplified solution clearance REAL"] =
                                ompl::toString(gsetup_->getSolutionPath().clearance());
                            run["simplified solution segments INTEGER"] =
                                std::to_string(gsetup_->getSolutionPath().getStateCount() - 1);
                            run["simplified correct solution BOOLEAN"] =
                                std::to_string(gsetup_->getSolutionPath().check());
                            gsetup_->getStateSpace()->setValidSegmentCountFactor(factor * 4);
                            run["simplified correct solution strict BOOLEAN"] =
                                std::to_string(gsetup_->getSolutionPath().check());
                            gsetup_->getStateSpace()->setValidSegmentCountFactor(factor);
                        }
                    }
                    else
                    {
                        run["approximate solution BOOLEAN"] =
                            std::to_string(csetup_->getProblemDefinition()->hasApproximateSolution());
                        run["solution difference REAL"] =
                            ompl::toString(csetup_->getProblemDefinition()->getSolutionDifference());
                        run["solution length REAL"] = ompl::toString(csetup_->getSolutionPath().length());
                        run["solution clearance REAL"] =
                            ompl::toString(csetup_->getSolutionPath().asGeometric().clearance());
                        run["solution segments INTEGER"] = std::to_string(csetup_->getSolutionPath().getControlCount());
                        run["correct solution BOOLEAN"] = std::to_string(csetup_->getSolutionPath().check());
                    }
                }

                base::PlannerData pd(gsetup_ ? gsetup_->getSpaceInformation() : csetup_->getSpaceInformation());
                planners_[i]->getPlannerData(pd);
                run["graph states INTEGER"] = std::to_string(pd.numVertices());
                run["graph motions INTEGER"] = std::to_string(pd.numEdges());

                for (const auto &prop : pd.properties)
                    run[prop.first] = prop.second;

                // execute post-run event, if set
                try
                {
                    if (postRun_)
                    {
                        OMPL_INFORM("Executing post-run event for run %d of planner %s ...", status_.activeRun,
                                    status_.activePlanner.c_str());
                        postRun_(planners_[i], run);
                        OMPL_INFORM("Completed execution of post-run event");
                    }
                }
                catch (std::runtime_error &e)
                {
                    std::stringstream es;
                    es << "There was an error in the execution of the post-run event for run " << status_.activeRun
                       << " of planner " << status_.activePlanner << std::endl;
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
            catch (std::runtime_error &e)
            {
                std::stringstream es;
                es << "There was an error in the extraction of planner results: planner = " << status_.activePlanner
                   << ", run = " << status_.activePlanner << std::endl;
                es << "*** " << e.what() << std::endl;
                std::cerr << es.str();
                OMPL_ERROR(es.str().c_str());
            }

            ++j;
            if (req.runCount == 0)
            {
                maxTime -= rp.getTimeUsed();
                if (maxTime < 0.)
                    break;
            }
            else
            {
                if (j >= req.runCount)
                    break;
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
