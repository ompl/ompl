/**
 * @file MotionBenchmakerDemoMain.cpp
 *
 * @brief Example program demonstrating how to use the Motion Benchmaker
 *        with OMPL and VAMP for evaluating motion planning on MBM problems.
 *
 * This example shows how to:
 * 1. Load Motion Benchmark Maker problems from a json file
 * 2. Run planning benchmarks using OMPL planners with VAMP collision checking
 * 3. Collect and print statistics on the results, or run benchmark with OMPL and save to .log
 *
 * Usage:
 *   ./demo_MotionBenchmakerDemo --problem-file <problem_file.json> --robot <robot_name>
 *
 * Example:
 *   ./demo_MotionBenchmakerDemo --problem-file problems.json --robot panda
 *
 * Usage for benchmark
 *   ./demo_MotionBenchmakerDemo --problem-file problems.json --robot panda --benchmark 100
 */

#include "MotionBenchmakerDemo.h"
#include <iostream>
#include <string>

#include <vector>
#include <optional>
#include <boost/program_options.hpp>
#include <vamp/collision/shapes.hh>
#include <vamp/collision/capt.hh>
#include <vamp/collision/attachments.hh>

namespace po = boost::program_options;

int main(int argc, char **argv)
{
    po::options_description desc("Options");

    std::string problemFile;
    std::string robotName;
    std::string plannerName;
    unsigned int benchmarkTrials = 0;
    double timeoutSeconds = 5.0;

    desc.add_options()("help", "show help message")(
        "problem-file", po::value<std::string>(&problemFile)->default_value("problems.json"),
        "Problem file in JSON format")("robot", po::value<std::string>(&robotName)->default_value("panda"),
                                       "Robot name (e.g., panda)")(
        "planner", po::value<std::string>(&plannerName)->default_value("RRTConnect"),
        "Planner name (e.g., RRTConnect, RRT, KPIECE1)")("benchmark",
                                                         po::value<unsigned int>(&benchmarkTrials)->default_value(0),
                                                         "Benchmark Planners for a specified number of trials")(
        "timeout", po::value<double>(&timeoutSeconds)->default_value(5.0),
        "Timeout in seconds for each planning attempt");

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help"))
    {
        std::cout << desc << std::endl;
        return 0;
    }

    std::cout << "Motion Benchmaker Demo" << std::endl;
    std::cout << "======================" << std::endl;
    std::cout << "Problem File: " << problemFile << std::endl;
    std::cout << "Robot: " << robotName << std::endl;
    std::cout << "Planner: " << plannerName << std::endl;
    std::cout << "Trials: " << benchmarkTrials << std::endl;
    std::cout << std::endl;

    try
    {
        // Create the benchmarker
        MotionBenchmakerDemo benchmarker(robotName, problemFile, plannerName);

        // Get list of problems
        const auto &problemNames = benchmarker.getProblemNames();
        std::cout << "Available problems: ";
        for (size_t i = 0; i < problemNames.size(); ++i)
        {
            if (i > 0)
                std::cout << ", ";
            std::cout << problemNames[i];
        }
        std::cout << std::endl << std::endl;

        // Run benchmarks on all problems
        std::cout << "Running benchmarks..." << std::endl;
        auto results = benchmarker.benchmarkAll(benchmarkTrials, timeoutSeconds, true);

        // Print statistics for each problem
        if (benchmarkTrials > 0)
        {
            std::cout << "Benchmark Results Saved, use python script to transfer to .db" << std::endl;
        }
        else
        {
            for (const auto &[problemName, problemResults] : results)
            {
                MotionBenchmakerDemo::printStatistics(problemName, problemResults);
            }
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
