/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, University of Stuttgart
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
 *   * Neither the name of the University of Stuttgart nor the names
 *     of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written
 *     permission.
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

/* Author: Andreas Orthey */

#include <ompl/base/SpaceInformation.h>
#include <ompl/tools/benchmark/Benchmark.h>

namespace ot = ompl::tools;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void printBenchmarkResults(const ot::Benchmark &b)
{
    ot::Benchmark::CompleteExperiment experiment = b.getRecordedExperimentData();

    std::vector<double> meanTime;
    std::vector<std::string> plannerName;
    std::map<double, std::pair<std::string, int>> plannerTimes;

    for (unsigned int k = 0; k < experiment.planners.size(); k++)
    {
        ot::Benchmark::PlannerExperiment pk = experiment.planners.at(k);
        std::vector<ot::Benchmark::RunProperties> runs = pk.runs;

        unsigned int N = runs.size();
        double time = 0;
        double percentSuccess = 0.0;
        for (unsigned int j = 0; j < N; j++)
        {
            ot::Benchmark::RunProperties run = runs.at(j);
            double timeJrun = std::atof(run["time REAL"].c_str());
            time += timeJrun;
            if (timeJrun < experiment.maxTime)
                percentSuccess++;
        }

        time = time / (double)N;
        percentSuccess = 100.0 * (percentSuccess / (double)N);
        pk.name.erase(0, 10);

        plannerTimes[time] = std::make_pair(pk.name, percentSuccess);
    }

    std::cout << "Finished Benchmark (Runtime:" << experiment.maxTime << ", RunCount:" << experiment.runCount << ")"
              << std::endl;
    std::cout << "Placement <Rank> <Time (in Seconds)> <Success (in Percentage)>" << std::endl;
    unsigned int ctr = 1;
    std::cout << std::string(80, '-') << std::endl;
    for (auto const &p : plannerTimes)
    {
        std::cout << "Place <" << ctr << "> Time: <" << p.first << "> \%Success: <" << p.second.second << "> ("
                  << p.second.first << ")" << (ctr < 2 ? " <-- Winner" : "") << std::endl;
        ctr++;
    }
    std::cout << std::string(80, '-') << std::endl;
}

void printEstimatedTimeToCompletion(unsigned int numberPlanners, unsigned int run_count, unsigned int runtime_limit)
{
    std::cout << std::string(80, '-') << std::endl;
    double worst_case_time_estimate_in_seconds = numberPlanners * run_count * runtime_limit;
    double worst_case_time_estimate_in_minutes = worst_case_time_estimate_in_seconds / 60.0;
    double worst_case_time_estimate_in_hours = worst_case_time_estimate_in_minutes / 60.0;
    std::cout << "Number of Planners           : " << numberPlanners << std::endl;
    std::cout << "Number of Runs Per Planner   : " << run_count << std::endl;
    std::cout << "Time Per Run (s)             : " << runtime_limit << std::endl;
    std::cout << "Worst-case time requirement  : ";

    if (worst_case_time_estimate_in_hours < 1)
    {
        if (worst_case_time_estimate_in_minutes < 1)
        {
            std::cout << worst_case_time_estimate_in_seconds << "s" << std::endl;
        }
        else
        {
            std::cout << worst_case_time_estimate_in_minutes << "m" << std::endl;
        }
    }
    else
    {
        std::cout << worst_case_time_estimate_in_hours << "h" << std::endl;
    }
    std::cout << std::string(80, '-') << std::endl;
}
