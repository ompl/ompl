#include "MotionBenchmakerDemo.h"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/PlannerTerminationCondition.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/fmt/FMT.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/kpiece/LBKPIECE1.h>
#include <ompl/vamp/VampStateValidityChecker.h>
#include <ompl/vamp/VampMotionValidator.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/vamp/VampStateSpace.h>
#include <vamp/robots/panda.hh>
#include <vamp/vector.hh>
#include <ompl/tools/benchmark/Benchmark.h>


#include <iostream>
#include <iomanip>
#include <numeric>
#include <algorithm>
#include <cmath>
#include <fstream>

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace ot = ompl::tools;


MotionBenchmakerDemo::MotionBenchmakerDemo(const std::string& robotName,
                                          const std::string& problemFile,
                                          const std::string& plannerName)
    : robotName_(robotName), plannerName_(plannerName)
{
    loadProblemsFromJSON(problemFile);
    initializeStateSpace();
}

void MotionBenchmakerDemo::loadProblemsFromJSON(const std::string& filename)
{
    using boost::property_tree::ptree;
    ptree pt;
    
    try {
        std::ifstream file(filename);
        if (!file.is_open()) {
            throw std::runtime_error("Failed to open JSON file: " + filename);
        }
        boost::property_tree::json_parser::read_json(file, pt);
    } catch (const std::exception& e) {
        throw std::runtime_error(std::string("Failed to load JSON file: ") + e.what());
    }

    // Check if problems key exists
    auto problemsIt = pt.find("problems");
    if (problemsIt == pt.not_found()) {
        throw std::runtime_error("JSON file does not contain 'problems' key");
    }

    // Iterate through each problem type (e.g., "problem_type_1", "problem_type_2")
    for (const auto& problemEntry : pt.get_child("problems")) {
        std::string problemName = problemEntry.first;
        problemNames_.push_back(problemName);

        const auto& problemInstances = problemEntry.second;
        std::vector<ptree> instances;

        // Each problem type contains an array of problem instances
        for (const auto& instanceEntry : problemInstances) {
            instances.push_back(instanceEntry.second);
        }
        
        problems_[problemName] = instances;
    }

    std::cout << "Loaded " << problemNames_.size() << " problem types from " << filename << std::endl;
}

void MotionBenchmakerDemo::initializeStateSpace()
{
    // Create state space
    using Robot = vamp::robots::Panda;
    space_ = std::make_shared<ompl::vamp::VampStateSpace<Robot>>();
}

std::shared_ptr<ompl::base::Planner> MotionBenchmakerDemo::createPlanner(
    const std::shared_ptr<ompl::base::SpaceInformation>& si) const
{
    std::shared_ptr<ompl::base::Planner> planner;

    if (plannerName_ == "RRTConnect" || plannerName_ == "rrtc") {
        planner = std::make_shared<og::RRTConnect>(si);
    } else if (plannerName_ == "RRTstar" || plannerName_ == "rrtstar") {
        planner = std::make_shared<og::RRTstar>(si);
    } else if (plannerName_ == "FMT" || plannerName_ == "fmt") {
        planner = std::make_shared<og::FMT>(si);
    } else if (plannerName_ == "PRM" || plannerName_ == "prm") {
        planner = std::make_shared<og::PRM>(si);
    }
     else if (plannerName_ == "KPIECE1" || plannerName_ == "kpiece1") {
        planner = std::make_shared<og::KPIECE1>(si);
    }
    else if (plannerName_ == "RRT" || plannerName_ == "rrt") {
        planner = std::make_shared<og::RRT>(si);
    } else if (plannerName_ == "LBKPIECE1" || plannerName_ == "lbkpiece1") {
        planner = std::make_shared<og::LBKPIECE1>(si);
    }
    else {
        std::cerr << "Unknown planner '" << plannerName_ << "', using RRTConnect" << std::endl;
        planner = std::make_shared<og::RRTConnect>(si);
    }

    return planner;
}

bool MotionBenchmakerDemo::setupProblem(
    const std::string& problemName,
    const boost::property_tree::ptree& problemData)
{

    // Check if problem is valid
    bool valid = problemData.get<bool>("valid", true);
    if (!valid) {
        return false;
    }

    // Build environment from problem data
    vamp::collision::Environment<float> env_float;
    bool is_box = problemName == "box" ? true : false;
    
    // Load spheres
    if (problemData.find("sphere") != problemData.not_found()) {
        for (const auto& sphereEntry : problemData.get_child("sphere")) {
            const auto& s = sphereEntry.second;
            std::array<float, 3> center;
            size_t i = 0;
            const auto& posChild = (s.find("center") != s.not_found()) ? 
                s.get_child("center") : s.get_child("position");
            for (const auto& coord : posChild) {
                if (i < 3) center[i++] = std::stof(coord.second.data());
            }
            float radius = s.get<float>("radius");
            env_float.spheres.emplace_back(
                vamp::collision::factory::sphere::array(center, radius));
        }
    }
    
    // Load cylinders (convert to capsules)
    if (problemData.find("cylinder") != problemData.not_found()) {
        for (const auto& cylEntry : problemData.get_child("cylinder")) {
            const auto& c = cylEntry.second;
            std::array<float, 3> center, orientation;
            size_t i = 0;
            const auto& posChild = (c.find("center") != c.not_found()) ? 
                c.get_child("center") : c.get_child("position");
            for (const auto& coord : posChild) {
                if (i < 3) center[i++] = std::stof(coord.second.data());
            }
            i = 0;
            for (const auto& euler : c.get_child("orientation_euler_xyz")) {
                if (i < 3) orientation[i++] = std::stof(euler.second.data());
            }
            float radius = c.get<float>("radius");
            float length = c.get<float>("length");
            if (is_box) {
                std::array<float, 3> halfExtents = {radius, radius, length / 2.0};
                env_float.cuboids.emplace_back(
                    vamp::collision::factory::cuboid::array(center, orientation, halfExtents));
            } 
            else {
                env_float.capsules.emplace_back(
                    vamp::collision::factory::capsule::center::array(center, orientation, radius, length));
            }
        }
    }
    
    // Load boxes
    if (problemData.find("box") != problemData.not_found()) {
        for (const auto& boxEntry : problemData.get_child("box")) {
            const auto& b = boxEntry.second;
            std::array<float, 3> center, orientation, halfExtents;
            size_t i = 0;
            const auto& posChild = (b.find("center") != b.not_found()) ? 
                b.get_child("center") : b.get_child("position");
            for (const auto& coord : posChild) {
                if (i < 3) center[i++] = std::stof(coord.second.data());
            }
            i = 0;
            for (const auto& euler : b.get_child("orientation_euler_xyz")) {
                if (i < 3) orientation[i++] = std::stof(euler.second.data());
            }
            i = 0;
            for (const auto& extent : b.get_child("half_extents")) {
                if (i < 3) halfExtents[i++] = std::stof(extent.second.data());
            }
            env_float.cuboids.emplace_back(
                vamp::collision::factory::cuboid::array(center, orientation, halfExtents));
        }
    }

    using Environment = vamp::collision::Environment<vamp::FloatVector<vamp::FloatVectorWidth>>;
    currentEnv_ = std::make_shared<Environment>(env_float);

    using Robot = vamp::robots::Panda;
    auto space_info = ss_->getSpaceInformation();

    // Add VAMP-based validators using the environment
    space_info->setStateValidityChecker(
        std::make_shared<ompl::vamp::VampStateValidityChecker<Robot>>(space_info, *currentEnv_));
    space_info->setMotionValidator(
        std::make_shared<ompl::vamp::VampMotionValidator<Robot>>(space_info, *currentEnv_));

    // Set start and goal configurations
    ob::ScopedState<> start(space_);
    ob::ScopedState<> goal(space_);

    // Get start configuration
    std::vector<double> startVec;
    if (problemData.find("start") != problemData.not_found()) {
        for (const auto& val : problemData.get_child("start")) {
            startVec.push_back(std::stod(val.second.data()));
        }
    }
    
    for (size_t i = 0; i < startVec.size() && i < space_->getDimension(); ++i) {
        start[i] = startVec[i];
    }

    // Get goal configuration
    std::vector<double> goalVec;
    if (problemData.find("goal") != problemData.not_found()) {
        for (const auto& val : problemData.get_child("goal")) {
            goalVec.push_back(std::stod(val.second.data()));
        }
    } else if (problemData.find("goals") != problemData.not_found()) {
        // Use first goal from goals array
        auto goalsChild = problemData.get_child("goals");
        if (goalsChild.size() > 0) {
            auto firstGoal = goalsChild.begin()->second;
            for (const auto& val : firstGoal) {
                goalVec.push_back(std::stod(val.second.data()));
            }
        }
    }
    
    for (size_t i = 0; i < goalVec.size() && i < space_->getDimension(); ++i) {
        goal[i] = goalVec[i];
    }

    // print start vs goal
    // std::cout << "Start: ";
    // for (size_t i = 0; i < startVec.size(); ++i) {
    //     std::cout << start[i] << " ";
    // }
    // std::cout << std::endl;

    // std::cout << "Goal: ";
    // for (size_t i = 0; i < goalVec.size(); ++i) {
    //     std::cout << goal[i] << " ";
    // }
    // std::cout << std::endl;

    // Set the start and goal states
    ss_->setStartAndGoalStates(start, goal);
    return true;
}


PlanningResult MotionBenchmakerDemo::solveInstance(
    const std::string& problemName,
    const boost::property_tree::ptree& problemData,
    double timeoutSeconds)
{
    PlanningResult result;
    ss_ = std::make_shared<og::SimpleSetup>(space_);
    
    auto space_info = ss_->getSpaceInformation();
    
    if (!setupProblem(problemName, problemData)) {
        return result;
    }
    space_info = ss_->getSpaceInformation();
    // Setup benchmarking
    auto planner = createPlanner(space_info);
    ss_->setPlanner(planner);

    // Plan
    auto startTime = std::chrono::steady_clock::now();
    ob::PlannerStatus status = ss_->solve(
        ob::timedPlannerTerminationCondition(timeoutSeconds)
    );
    auto endTime = std::chrono::steady_clock::now();

    result.planningTime = std::chrono::duration_cast<std::chrono::nanoseconds>(
        endTime - startTime
    );

    if (status == ob::PlannerStatus::EXACT_SOLUTION ||
        status == ob::PlannerStatus::APPROXIMATE_SOLUTION) {
        result.solved = true;
        auto path = ss_->getSolutionPath();
        result.pathVertices = path.getStateCount();
        result.pathCost = path.length();
    }

    // Get planner data for iteration count
    ob::PlannerData plannerData(space_info);
    ss_->getPlanner()->getPlannerData(plannerData);
    result.planningIterations = plannerData.numVertices();

    // Clean up
    ss_.reset();
    
    return result;
}

PlanningResult MotionBenchmakerDemo::benchmarkInstance(
    const std::string& problemName,
    const boost::property_tree::ptree& problemData,
    unsigned int benchmarkTrials,
    double timeoutSeconds)
{
    PlanningResult result;
    ss_ = std::make_shared<og::SimpleSetup>(space_);
    
    if (!setupProblem(problemName, problemData)) {
        return result;
    }
    auto space_info = ss_->getSpaceInformation();
    // setup benchmark
    double memoryLimit = 4096;

    std::string benchmarkName = "MBM-" + problemName;
    ot::Benchmark b(*ss_, benchmarkName);
    ot::Benchmark::Request request(timeoutSeconds, memoryLimit, benchmarkTrials);

    b.addPlanner(std::make_shared<og::RRTConnect>(space_info));
    b.addPlanner(std::make_shared<og::RRT>(space_info));
    b.addPlanner(std::make_shared<og::KPIECE1>(space_info));    
    b.addPlanner(std::make_shared<og::LBKPIECE1>(space_info));

    b.benchmark(request);
    b.saveResultsToFile("vamp_mbm_benchmark.log");
    result.solved = true;
    ss_.reset();
    
    return result;
}


std::vector<PlanningResult> MotionBenchmakerDemo::benchmarkProblem(
    const std::string& problemName,
    unsigned int benchmarkTrials,
    double timeoutSeconds)
{
    std::vector<PlanningResult> allResults;

    if (problems_.find(problemName) == problems_.end()) {
        std::cerr << "Problem '" << problemName << "' not found" << std::endl;
        return allResults;
    }

    const auto& instances = problems_[problemName];
    std::cout << "Benchmarking '" << problemName << "' with " << instances.size()
              << " instances..." << std::endl;

    for (size_t i = 0; i < instances.size(); ++i) {
        std::cout << "  Instance " << i + 1 << "/" << instances.size() << "... ";
        std::cout.flush();

        if (benchmarkTrials == 0) {
            auto result = solveInstance(problemName, instances[i], timeoutSeconds);
            allResults.push_back(result);
        } 
        else {
            auto result = benchmarkInstance(problemName, instances[i], benchmarkTrials, timeoutSeconds);
            allResults.push_back(result);
            break;
        }
    }

    return allResults;
}

std::map<std::string, std::vector<PlanningResult>> MotionBenchmakerDemo::benchmarkAll(
    unsigned int benchmarkTrials,
    double timeoutSeconds,
    bool print_failures)
{
    std::map<std::string, std::vector<PlanningResult>> allResults;

    for (const auto& problemName : problemNames_) {

        // if (problemName != "table_under_pick") {
        //     continue;
        // }

        auto results = benchmarkProblem(problemName, benchmarkTrials, timeoutSeconds);
        allResults[problemName] = results;

        if (print_failures) {
            unsigned int failures = 0;
            for (const auto& result : results) {
                if (!result.solved) {
                    failures++;
                }
            }
            if (failures > 0) {
                std::cout << "    Failed: " << failures << "/" << results.size() << std::endl;
            }
        }

    }

    return allResults;
}

void MotionBenchmakerDemo::printStatistics(const std::string& problemName,
                                          const std::vector<PlanningResult>& results)
{
    if (results.empty()) {
        std::cout << "No results for problem '" << problemName << "'" << std::endl;
        return;
    }

    // Count successes
    unsigned int successes = 0;
    std::vector<double> planningTimes;
    std::vector<double> pathCosts;
    std::vector<unsigned int> iterations;
    std::vector<unsigned int> vertices;

    for (const auto& result : results) {
        if (result.solved) {
            successes++;
            planningTimes.push_back(result.planningTime.count() / 1e9);  // Convert to seconds
            pathCosts.push_back(result.pathCost);
            iterations.push_back(result.planningIterations);
            vertices.push_back(result.pathVertices);
        }
    }

    std::cout << "\n=== Statistics for '" << problemName << "' ===" << std::endl;
    std::cout << "Success Rate: " << successes << "/" << results.size() << std::endl;

    if (successes > 0) {
        // Calculate statistics
        auto minMaxTime = std::minmax_element(planningTimes.begin(), planningTimes.end());
        double meanTime = std::accumulate(planningTimes.begin(), planningTimes.end(), 0.0) /
                         planningTimes.size();

        auto minMaxCost = std::minmax_element(pathCosts.begin(), pathCosts.end());
        double meanCost = std::accumulate(pathCosts.begin(), pathCosts.end(), 0.0) /
                         pathCosts.size();

        std::cout << std::fixed << std::setprecision(4);
        std::cout << "Planning Time (s): min=" << *minMaxTime.first
                  << ", max=" << *minMaxTime.second
                  << ", mean=" << meanTime << std::endl;

        std::cout << "Path Cost (m): min=" << *minMaxCost.first
                  << ", max=" << *minMaxCost.second
                  << ", mean=" << meanCost << std::endl;

        unsigned int meanIter = std::accumulate(iterations.begin(), iterations.end(), 0u) /
                               iterations.size();
        unsigned int meanVert = std::accumulate(vertices.begin(), vertices.end(), 0u) /
                               vertices.size();

        std::cout << "Mean Iterations: " << meanIter << std::endl;
        std::cout << "Mean Path Vertices: " << meanVert << std::endl;
    }
    std::cout << std::endl;
}

