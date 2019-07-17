#include <ompl/base/SpaceInformation.h>
#include <ompl/tools/benchmark/Benchmark.h>

namespace ot = ompl::tools;
namespace ob = ompl::base;
namespace og = ompl::geometric;
void PrintBenchmarkResults(const ot::Benchmark &b)
{
    ot::Benchmark::CompleteExperiment experiment = b.getRecordedExperimentData();

    std::vector<double> meanTime;
    std::vector<std::string> plannerName;
    std::map<double, std::pair<std::string, int>> plannerTimes;

    for(uint k = 0; k < experiment.planners.size(); k++)
    {
        ot::Benchmark::PlannerExperiment pk = experiment.planners.at(k);
        std::vector<ot::Benchmark::RunProperties> runs = pk.runs;

        uint N = runs.size();
        double time = 0;
        double percentSuccess = 0.0;
        for(uint j = 0; j < N; j++)
        {
            ot::Benchmark::RunProperties run = runs.at(j);
            double timeJrun = std::atof(run["time REAL"].c_str());
            time += timeJrun;
            if(timeJrun < experiment.maxTime)
            {
              percentSuccess++;
            }

        }

        time = time / (double)N;
        percentSuccess = 100.0*(percentSuccess / (double)N);
        pk.name.erase(0,10);

        plannerTimes[time] = std::make_pair(pk.name, percentSuccess);
    }

    std::cout << "Finished Benchmark (Runtime:" << experiment.maxTime 
      << ", RunCount:" << experiment.runCount << ")" << std::endl;
    std::cout << "Placement <Rank> <Time (in Seconds)> <Success (in Percentage)>" << std::endl;
    uint ctr = 1;
    std::cout << std::string(80, '-') << std::endl;
    for (auto const &p : plannerTimes)
    {
        std::cout << "Place <" << ctr++ << "> Time: <" << p.first 
          << "> \%Success: <" <<  p.second.second 
          << "> (" <<  p.second.first << ")" 
          << (ctr<2?" <-- Winner":"")<< std::endl;
    }
    std::cout << std::string(80, '-') << std::endl;
}



