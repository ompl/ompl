/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, Rice University
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
 *   * Neither the name of Rice University nor the names of its
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

/* Author: Sahruday Patti */

/// @file VAMPAllPlannersDemo.cpp
/// @brief Demonstrates VAMP-accelerated planning with all major OMPL planners.
///
/// This demo shows that VAMP's SIMD-accelerated collision detection works with
/// every geometric planner in OMPL, not just a handful. It uses VAMPSetup to
/// configure the planning problem and runs each planner on a sphere-cage
/// environment around a Panda arm.
///
/// Usage:
///   demo_VAMPAllPlanners                      Run all planners once
///   demo_VAMPAllPlanners --benchmark 20       Benchmark all planners (20 runs each)
///   demo_VAMPAllPlanners --planner RRTConnect  Run a single planner

#include <iostream>
#include <vector>
#include <array>
#include <map>
#include <string>
#include <chrono>
#include <functional>
#include <iomanip>

#include <boost/program_options.hpp>

#include <ompl/vamp/VAMPSetup.h>

// All geometric planners
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/RRTsharp.h>
#include <ompl/geometric/planners/rrt/RRTXstatic.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/rrt/LBTRRT.h>
#include <ompl/geometric/planners/rrt/LazyLBTRRT.h>
#include <ompl/geometric/planners/rrt/LazyRRT.h>
#include <ompl/geometric/planners/rrt/TRRT.h>
#include <ompl/geometric/planners/rrt/BiTRRT.h>
#include <ompl/geometric/planners/rrt/ATRRT.h>
#include <ompl/geometric/planners/rrt/SORRTstar.h>
#include <ompl/geometric/planners/rrt/STRRTstar.h>
#include <ompl/geometric/planners/rrt/AORRTC.h>

#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/prm/LazyPRM.h>
#include <ompl/geometric/planners/prm/LazyPRMstar.h>
#include <ompl/geometric/planners/prm/SPARS.h>
#include <ompl/geometric/planners/prm/SPARStwo.h>

#include <ompl/geometric/planners/informedtrees/BITstar.h>
#include <ompl/geometric/planners/informedtrees/ABITstar.h>
#include <ompl/geometric/planners/informedtrees/AITstar.h>
#include <ompl/geometric/planners/informedtrees/EITstar.h>

#include <ompl/geometric/planners/lazyinformedtrees/BLITstar.h>

#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/kpiece/BKPIECE1.h>
#include <ompl/geometric/planners/kpiece/LBKPIECE1.h>

#include <ompl/geometric/planners/est/EST.h>
#include <ompl/geometric/planners/est/BiEST.h>
#include <ompl/geometric/planners/est/ProjEST.h>

#include <ompl/geometric/planners/sbl/SBL.h>

#include <ompl/geometric/planners/fmt/FMT.h>
#include <ompl/geometric/planners/fmt/BFMT.h>

#include <ompl/geometric/planners/pdst/PDST.h>
#include <ompl/geometric/planners/stride/STRIDE.h>
#include <ompl/geometric/planners/sst/SST.h>

#include <ompl/geometric/planners/rlrt/RLRT.h>
#include <ompl/geometric/planners/rlrt/BiRLRT.h>

#include <vamp/collision/factory.hh>
#include <vamp/robots/panda.hh>

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace ot = ompl::tools;
namespace po = boost::program_options;

using Robot = vamp::robots::Panda;

using PlannerFactory = std::function<ob::PlannerPtr(const ob::SpaceInformationPtr &)>;

/// All supported planner factories, grouped by category.
auto getAllPlannerFactories() -> std::map<std::string, PlannerFactory>
{
    return {
        // RRT family
        {"RRTConnect", [](const ob::SpaceInformationPtr &si) { return std::make_shared<og::RRTConnect>(si); }},
        {"RRT", [](const ob::SpaceInformationPtr &si) { return std::make_shared<og::RRT>(si); }},
        {"RRTstar", [](const ob::SpaceInformationPtr &si) { return std::make_shared<og::RRTstar>(si); }},
        {"RRTsharp", [](const ob::SpaceInformationPtr &si) { return std::make_shared<og::RRTsharp>(si); }},
        {"RRTXstatic", [](const ob::SpaceInformationPtr &si) { return std::make_shared<og::RRTXstatic>(si); }},
        {"InformedRRTstar",
         [](const ob::SpaceInformationPtr &si) { return std::make_shared<og::InformedRRTstar>(si); }},
        {"LBTRRT", [](const ob::SpaceInformationPtr &si) { return std::make_shared<og::LBTRRT>(si); }},
        {"LazyRRT", [](const ob::SpaceInformationPtr &si) { return std::make_shared<og::LazyRRT>(si); }},
        {"TRRT", [](const ob::SpaceInformationPtr &si) { return std::make_shared<og::TRRT>(si); }},
        {"BiTRRT", [](const ob::SpaceInformationPtr &si) { return std::make_shared<og::BiTRRT>(si); }},
        {"AORRTC", [](const ob::SpaceInformationPtr &si) { return std::make_shared<og::AORRTC>(si); }},

        // PRM family
        {"PRM", [](const ob::SpaceInformationPtr &si) { return std::make_shared<og::PRM>(si); }},
        {"PRMstar", [](const ob::SpaceInformationPtr &si) { return std::make_shared<og::PRMstar>(si); }},
        {"LazyPRM", [](const ob::SpaceInformationPtr &si) { return std::make_shared<og::LazyPRM>(si); }},
        {"LazyPRMstar", [](const ob::SpaceInformationPtr &si) { return std::make_shared<og::LazyPRMstar>(si); }},
        {"SPARS", [](const ob::SpaceInformationPtr &si) { return std::make_shared<og::SPARS>(si); }},
        {"SPARStwo", [](const ob::SpaceInformationPtr &si) { return std::make_shared<og::SPARStwo>(si); }},

        // Informed trees
        {"BITstar", [](const ob::SpaceInformationPtr &si) { return std::make_shared<og::BITstar>(si); }},
        {"ABITstar", [](const ob::SpaceInformationPtr &si) { return std::make_shared<og::ABITstar>(si); }},
        {"AITstar", [](const ob::SpaceInformationPtr &si) { return std::make_shared<og::AITstar>(si); }},
        {"EITstar", [](const ob::SpaceInformationPtr &si) { return std::make_shared<og::EITstar>(si); }},
        {"BLITstar", [](const ob::SpaceInformationPtr &si) { return std::make_shared<og::BLITstar>(si); }},

        // KPIECE family
        {"KPIECE1", [](const ob::SpaceInformationPtr &si) { return std::make_shared<og::KPIECE1>(si); }},
        {"BKPIECE1", [](const ob::SpaceInformationPtr &si) { return std::make_shared<og::BKPIECE1>(si); }},
        {"LBKPIECE1", [](const ob::SpaceInformationPtr &si) { return std::make_shared<og::LBKPIECE1>(si); }},

        // EST family
        {"EST", [](const ob::SpaceInformationPtr &si) { return std::make_shared<og::EST>(si); }},
        {"BiEST", [](const ob::SpaceInformationPtr &si) { return std::make_shared<og::BiEST>(si); }},
        {"ProjEST", [](const ob::SpaceInformationPtr &si) { return std::make_shared<og::ProjEST>(si); }},

        // Other
        {"SBL", [](const ob::SpaceInformationPtr &si) { return std::make_shared<og::SBL>(si); }},
        {"FMT", [](const ob::SpaceInformationPtr &si) { return std::make_shared<og::FMT>(si); }},
        {"BFMT", [](const ob::SpaceInformationPtr &si) { return std::make_shared<og::BFMT>(si); }},
        {"PDST", [](const ob::SpaceInformationPtr &si) { return std::make_shared<og::PDST>(si); }},
        {"STRIDE", [](const ob::SpaceInformationPtr &si) { return std::make_shared<og::STRIDE>(si); }},
        {"SST", [](const ob::SpaceInformationPtr &si) { return std::make_shared<og::SST>(si); }},
    };
}

/// Build a sphere-cage environment around the robot.
auto buildSphereCageEnvironment() -> vamp::collision::Environment<float>
{
    vamp::collision::Environment<float> env;

    constexpr float radius = 0.2f;
    const std::vector<std::array<float, 3>> obstacles = {
        {0.55f, 0.f, 0.25f},  {0.35f, 0.35f, 0.25f},  {0.f, 0.55f, 0.25f},   {-0.55f, 0.f, 0.25f},
        {-0.35f, -0.35f, 0.25f}, {0.f, -0.55f, 0.25f}, {0.35f, -0.35f, 0.25f}, {0.35f, 0.35f, 0.8f},
        {0.f, 0.55f, 0.8f},   {-0.35f, 0.35f, 0.8f},  {-0.55f, 0.f, 0.8f},   {-0.35f, -0.35f, 0.8f},
        {0.f, -0.55f, 0.8f},  {0.35f, -0.35f, 0.8f},
    };

    for (const auto &obs : obstacles)
        env.spheres.emplace_back(vamp::collision::factory::sphere::array(obs, radius));
    env.sort();

    return env;
}

/// Run a single planner and report its result.
auto runPlanner(ompl::vamp::VAMPSetup<Robot> &setup, const std::string &name, const PlannerFactory &factory,
                double timeout) -> bool
{
    setup.clear();
    setup.setStartGoal({0., -0.785, 0., -2.356, 0., 1.571, 0.785}, {2.35, 1., 0., -0.8, 0., 2.5, 0.785});
    setup.setPlanner(factory(setup.getSpaceInformation()));

    auto t0 = std::chrono::steady_clock::now();
    ob::PlannerStatus status = setup.solve(timeout);
    auto t1 = std::chrono::steady_clock::now();
    double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

    bool solved = status == ob::PlannerStatus::EXACT_SOLUTION;
    std::cout << std::left << std::setw(22) << name << (solved ? "SOLVED" : "FAILED") << "  " << std::fixed
              << std::setprecision(2) << std::setw(10) << ms << " ms";

    if (solved)
        std::cout << "  (" << setup.getSolutionPath().getStateCount() << " states)";
    std::cout << std::endl;

    return solved;
}

/// Run all planners once, reporting results.
void runAllPlanners(double timeout)
{
    auto env = buildSphereCageEnvironment();
    ompl::vamp::VAMPSetup<Robot> setup(env);

    auto factories = getAllPlannerFactories();

    std::cout << "\n=== VAMP All-Planner Demo (" << Robot::name << ", " << Robot::dimension << " DOF) ===" << std::endl;
    std::cout << "SIMD width: " << vamp::FloatVectorWidth << std::endl;
    std::cout << "Timeout: " << timeout << " s per planner" << std::endl;
    std::cout << "Planners: " << factories.size() << std::endl;
    std::cout << std::string(60, '-') << std::endl;

    unsigned int solved = 0;
    unsigned int total = 0;
    for (const auto &[name, factory] : factories)
    {
        if (runPlanner(setup, name, factory, timeout))
            ++solved;
        ++total;
    }

    std::cout << std::string(60, '-') << std::endl;
    std::cout << "Results: " << solved << "/" << total << " planners found solutions" << std::endl;
}

/// Run a single named planner.
void runSinglePlanner(const std::string &plannerName, double timeout)
{
    auto factories = getAllPlannerFactories();
    auto it = factories.find(plannerName);
    if (it == factories.end())
    {
        std::cerr << "Unknown planner: " << plannerName << std::endl;
        std::cerr << "Available planners:" << std::endl;
        for (const auto &[name, _] : factories)
            std::cerr << "  " << name << std::endl;
        return;
    }

    auto env = buildSphereCageEnvironment();
    ompl::vamp::VAMPSetup<Robot> setup(env);

    std::cout << "\n=== VAMP Single Planner: " << plannerName << " (" << Robot::name << ") ===" << std::endl;
    runPlanner(setup, plannerName, it->second, timeout);

    if (setup.getSimpleSetup().haveSolutionPath())
    {
        std::cout << "\nSolution path:" << std::endl;
        setup.getSolutionPath().print(std::cout);
    }
}

/// Benchmark all planners using OMPL's Benchmark tool.
void benchmarkAllPlanners(unsigned int runs, double timeout)
{
    auto env = buildSphereCageEnvironment();
    ompl::vamp::VAMPSetup<Robot> setup(env);
    setup.setStartGoal({0., -0.785, 0., -2.356, 0., 1.571, 0.785}, {2.35, 1., 0., -0.8, 0., 2.5, 0.785});

    auto factories = getAllPlannerFactories();

    std::cout << "\n=== VAMP Benchmark (" << Robot::name << ", " << factories.size() << " planners, " << runs
              << " runs) ===" << std::endl;

    std::string logfile = setup.benchmark(factories, runs, timeout, "VAMP_AllPlanners_" + std::string(Robot::name));

    std::cout << "Results: " << logfile << std::endl;
    std::cout << "Convert: ompl_benchmark_statistics.py " << logfile << " -d results.db" << std::endl;
    std::cout << "Visualize: upload results.db to plannerarena.org" << std::endl;
}

int main(int argc, char **argv)
{
    po::options_description desc("VAMP All-Planner Demo");

    int benchmarkRuns = 0;
    std::string plannerName;
    double timeout = 5.0;

    desc.add_options()("help", "Show help")("benchmark", po::value<int>(&benchmarkRuns)->default_value(0),
                                            "Benchmark all planners with N runs each")(
        "planner", po::value<std::string>(&plannerName), "Run a single planner by name")(
        "timeout", po::value<double>(&timeout)->default_value(5.0), "Planning timeout in seconds");

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help"))
    {
        std::cout << desc << std::endl;
        std::cout << "\nAvailable planners:" << std::endl;
        for (const auto &[name, _] : getAllPlannerFactories())
            std::cout << "  " << name << std::endl;
        return 0;
    }

    if (benchmarkRuns > 0)
        benchmarkAllPlanners(static_cast<unsigned int>(benchmarkRuns), timeout);
    else if (!plannerName.empty())
        runSinglePlanner(plannerName, timeout);
    else
        runAllPlanners(timeout);

    return 0;
}
