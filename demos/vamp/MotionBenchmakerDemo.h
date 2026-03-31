#ifndef MOTION_BENCHMAKER_DEMO_H_
#define MOTION_BENCHMAKER_DEMO_H_

#include <string>
#include <vector>
#include <map>
#include <memory>
#include <chrono>

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/Planner.h>
#include <ompl/geometric/SimpleSetup.h>
#include <boost/property_tree/ptree.hpp>
#include <vamp/collision/environment.hh>

/**
 * @brief Results from a single planning trial.
 */
struct PlanningResult
{
    bool solved = false;
    double planningTime = 0.0;
    double simplificationTime = 0.0;
    unsigned int planningIterations = 0;
    unsigned int pathVertices = 0;
    double pathCost = 0.0;
};

/**
 * @brief Configuration for running Motion Benchmark Maker benchmarks with OMPL and VAMP.
 */
class MotionBenchmakerDemo
{
public:
    /**
     * @brief Initialize with a robot type and JSON problem file.
     *
     * @param robotName The VAMP robot name (e.g., "panda")
     * @param problemFile Path to the JSON file containing problem definitions
     * @param plannerName The OMPL planner to use (e.g., "RRTConnect", "RRTstar")
     */
    MotionBenchmakerDemo(const std::string& robotName,
                        const std::string& problemFile,
                        const std::string& plannerName = "RRTConnect");

    /**
     * @brief Run a benchmark on a specific problem.
     *
     * @param problemName Name of the problem to benchmark
     * @param numTrials Number of times to solve each problem instance
     * @param timeoutSeconds Timeout for each planning attempt
     *
     * @return Vector of results for each trial
     */
    std::vector<PlanningResult> benchmarkProblem(const std::string& problemName,
                                                 unsigned int numTrials = 1,
                                                 double timeoutSeconds = 5.0);

    /**
     * @brief Run benchmarks on all problems in the loaded file.
     *
     * @param numTrials Number of times to solve each problem instance
     * @param timeoutSeconds Timeout for each planning attempt
     * @param print_failures Whether to print failure information
     *
     * @return Map from problem name to list of results
     */
    std::map<std::string, std::vector<PlanningResult>> benchmarkAll(
        unsigned int numTrials = 1,
        double timeoutSeconds = 5.0,
        bool print_failures = false);

    /**
     * @brief Get the list of available problem names.
     */
    const std::vector<std::string>& getProblemNames() const
    {
        return problemNames_;
    }

    /**
     * @brief Print summary statistics for a set of results.
     */
    static void printStatistics(const std::string& problemName,
                               const std::vector<PlanningResult>& results);

private:
    std::string robotName_;
    std::string plannerName_;
    std::vector<std::string> problemNames_;
    std::map<std::string, std::vector<boost::property_tree::ptree>> problems_;
    std::shared_ptr<ompl::base::StateSpace> space_;
    std::shared_ptr<ompl::geometric::SimpleSetup> ss_;
    
    // Environment for collision checking
    using Environment = vamp::collision::Environment<vamp::FloatVector<vamp::FloatVectorWidth>>;
    std::shared_ptr<Environment> currentEnv_;

    /**
     * @brief Load problems from a JSON file (MBM format).
     */
    void loadProblemsFromJSON(const std::string& filename);

    /**
     * @brief Initialize the space information and planner for the given robot.
     */
    void initializeStateSpace();

    /**
     * @brief Set up a planning problem in the given simple setup.
     */
    bool setupProblem(const std::string& problemName,
                      const boost::property_tree::ptree& problemData);

    /**
     * @brief Run a single planning trial on a problem.
     */
    PlanningResult solveInstance(const std::string& problemName,
                               const boost::property_tree::ptree& problemData,
                               double timeoutSeconds);
    /**
     * @brief Benchmark an instance using OMPL's benchmark.
     */
    PlanningResult benchmarkInstance(
                                const std::string& problemName,
                                const boost::property_tree::ptree& problemData,
                                unsigned int benchmarkTrials,
                                double timeoutSeconds);

    /**
     * @brief Create an OMPL planner instance by name.
     */
    std::shared_ptr<ompl::base::Planner> createPlanner(
        const std::shared_ptr<ompl::base::SpaceInformation>& si) const;
};

#endif
