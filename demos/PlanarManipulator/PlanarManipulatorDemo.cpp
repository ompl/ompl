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

/* Author: Ryan Luna */

#include <fstream>

#include "boost/program_options.hpp"
#include "PolyWorld.h"
#include "PlanarManipulatorPolyWorld.h"
#include "PlanarManipulator.h"
#include "PlanarManipulatorStateSpace.h"
#include "PlanarManipulatorStateValidityChecker.h"
#include "PlanarManipulatorIKGoal.h"
#include "PlanarManipulatorTSRRTConfig.h"
#include "PlanarManipulatorXXLDecomposition.h"

#include <ompl/geometric/SimpleSetup.h>

// planners
#include <ompl/geometric/planners/xxl/XXL.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/stride/STRIDE.h>
#include <ompl/geometric/planners/rrt/TSRRT.h>
#include <ompl/geometric/planners/rlrt/RLRT.h>

// bi-directional
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/kpiece/BKPIECE1.h>
#include <ompl/geometric/planners/rlrt/BiRLRT.h>

#include <ompl/tools/benchmark/Benchmark.h>

// Input arguments to this binary.
struct Arguments
{
    int numLinks;
    int numRuns;
    double timeout;
    std::string problem;
    bool viz;
};

// Minimal setup for a planar manipulation problem.
struct Problem
{
    Problem(int links, const std::string& problemName, PolyWorld &&world, const Eigen::Affine2d &baseFrame, const Eigen::Affine2d &goalFrame)
      : name(problemName), manipulator(links, 1.0 / links), world(std::move(world)), goalFrame(goalFrame)
    {
        manipulator.setBaseFrame(baseFrame);
    }

    std::string name;
    PlanarManipulator manipulator;
    PolyWorld world;
    Eigen::Affine2d goalFrame;
};

// Creates a problem from the input arguments.
Problem CreateProblem(const Arguments &args)
{
    if (args.problem == "corridor")
    {
        Eigen::Affine2d baseFrame;
        Eigen::Affine2d goalFrame;
        PolyWorld world = createCorridorProblem(args.numLinks, baseFrame, goalFrame);
        return Problem(args.numLinks, args.problem, std::move(world), baseFrame, goalFrame);
    }
    else if (args.problem == "constricted")
    {
        Eigen::Affine2d baseFrame;
        Eigen::Affine2d goalFrame;
        PolyWorld world = createConstrictedProblem(args.numLinks, baseFrame, goalFrame);
        return Problem(args.numLinks, args.problem, std::move(world), baseFrame, goalFrame);
    }
    else
    {
        throw ompl::Exception("Unknown problem name: " + args.problem);
    }
}

// Initialize OMPL for the given planar manipulator problem.
ompl::geometric::SimpleSetupPtr setupOMPL(Problem &problem)
{
    const unsigned int numLinks = problem.manipulator.getNumLinks();
    // Create the state space for the manipulator.
    ompl::base::StateSpacePtr space(new PlanarManipulatorStateSpace(numLinks));
    ompl::base::RealVectorBounds bounds(numLinks);
    bounds.setLow(-M_PI);
    bounds.setHigh(M_PI);

    // Bound the joints of the manipulator between [-PI, PI]
    space->as<PlanarManipulatorStateSpace>()->setBounds(bounds);
    problem.manipulator.setBounds(bounds.low, bounds.high);

    ompl::geometric::SimpleSetupPtr setup(new ompl::geometric::SimpleSetup(space));

    // Create the collision checker.
    setup->setStateValidityChecker(std::make_shared<PlanarManipulatorCollisionChecker>(
        setup->getSpaceInformation(), problem.manipulator, problem.world));

    // Increase motion validator resolution.
    setup->getSpaceInformation()->setStateValidityCheckingResolution(0.001);

    // Set the start and goal.
    ompl::base::State *start = setup->getStateSpace()->allocState();
    double *start_angles = start->as<PlanarManipulatorStateSpace::StateType>()->values;
    for (size_t i = 0; i < numLinks; ++i)
        start_angles[i] = 1e-7;  // zero is dangerous for numerical reasons.
    setup->getProblemDefinition()->addStartState(start);
    setup->getStateSpace()->freeState(start);

    ompl::base::GoalPtr goal(new PlanarManipulatorIKGoal(setup->getSpaceInformation(), problem.goalFrame,
                                                         &problem.manipulator,
                                                         false));  // orientation is not fixed
    goal->as<PlanarManipulatorIKGoal>()->setThreshold(1e-3);
    setup->setGoal(goal);

    return setup;
}

// Returns the bounds on the reachable part of the world for the chain.
ompl::base::RealVectorBounds GetReachableWorkspaceBounds(Point origin, double chainLength, const Problem &problem)
{
    const auto xBounds = problem.world.xBounds();
    const auto yBounds = problem.world.yBounds();

    // Clip the world bounds based on reachable workspace of the chain.
    double xMin = std::max(origin.first - chainLength, xBounds.first);
    double xMax = std::min(origin.first + chainLength, xBounds.second);
    double yMin = std::max(origin.second - chainLength, yBounds.first);
    double yMax = std::min(origin.second + chainLength, yBounds.second);
    ompl::base::RealVectorBounds reachable_bounds(2);
    reachable_bounds.setLow(0, xMin);
    reachable_bounds.setHigh(0, xMax);
    reachable_bounds.setLow(1, yMin);
    reachable_bounds.setHigh(1, yMax);

    return reachable_bounds;
}

// Returns the task-space projection for the planar manipulator when planning
// using TSRRT.  Projects the end-effector position into the workspace.
ompl::geometric::TaskSpaceConfigPtr getTaskSpaceConfig(const Problem &problem)
{
    const PlanarManipulator &manip = problem.manipulator;
    unsigned int numLinks = manip.getNumLinks();
    double linkLength = 1.0 / numLinks;
    double chainLength = numLinks * linkLength;

    const Eigen::Affine2d &baseFrame = manip.getBaseFrame();
    Point origin = {baseFrame.translation()(0), baseFrame.translation()(1)};

    const ompl::base::RealVectorBounds reachable_bounds = GetReachableWorkspaceBounds(origin, chainLength, problem);

    ompl::geometric::TaskSpaceConfigPtr task_space_ptr(new PlanarManipTaskSpaceConfig(&manip, reachable_bounds));
    return task_space_ptr;
}

// Returns the XXL decomposition for the planar manipulator.  Splits the
// 2D workspace into a grid with numXYSlices in the x and y dimensions.
// Projects the end-effector position of the chain into the decomposition.
// For chains with more than six links, the midpoint is also projected.
ompl::geometric::XXLDecompositionPtr getXXLDecomp(const ompl::base::SpaceInformationPtr &si, const Problem &problem,
                                                  int numXYSlices)
{
    const PlanarManipulator &manip = problem.manipulator;
    unsigned int numLinks = manip.getNumLinks();
    double linkLength = 1.0 / numLinks;  // TODO: pass this in
    double chainLength = numLinks * linkLength;

    // Creating decomposition for XXL
    const Eigen::Affine2d &baseFrame = manip.getBaseFrame();
    Point origin = {baseFrame.translation()(0), baseFrame.translation()(1)};

    const ompl::base::RealVectorBounds reachable_bounds = GetReachableWorkspaceBounds(origin, chainLength, problem);

    std::vector<int> xySlices(2, numXYSlices);
    const int thetaSlices = 1;
    // Select point(s) on the manipulator for projection.
    // For short chains, we only pick the end-effector position.
    std::vector<int> projLinks;
    if (numLinks > 6)
    {
        // midpoint
        projLinks.push_back((numLinks / 2) - 1);
    }
    // end-effector.
    projLinks.push_back(numLinks - 1);

    ompl::geometric::XXLDecompositionPtr decomp(new PMXXLDecomposition(
        si, &manip, reachable_bounds, xySlices, thetaSlices, projLinks, true));  // diagonal edges
    return decomp;
}

// Computes the Cartesian distance traveled by each joint in the chain
// on the solution path that is computed.
void postRunEvent(const ompl::base::PlannerPtr &planner, ompl::tools::Benchmark::RunProperties &run,
                  const PlanarManipulator *manip)
{
    if (!planner->getProblemDefinition()->hasSolution())
        return;

    double cartesianDist = 0.0;
    const auto &path = static_cast<const ompl::geometric::PathGeometric &>(
        *(planner->getProblemDefinition()->getSolutionPath().get()));
    for (size_t i = 0; i < path.getStateCount() - 1; ++i)
    {
        std::vector<Eigen::Affine2d> startFrames, endFrames;
        manip->FK(path.getState(i)->as<PlanarManipulatorStateSpace::StateType>()->values, startFrames);
        manip->FK(path.getState(i + 1)->as<PlanarManipulatorStateSpace::StateType>()->values, endFrames);

        for (size_t j = 1; j < endFrames.size(); ++j)
            cartesianDist += (endFrames[j].translation() - startFrames[j].translation()).norm();
    }

    run["Cartesian Distance REAL"] = boost::lexical_cast<std::string>(cartesianDist);
}

void BenchmarkProblem(ompl::geometric::SimpleSetupPtr setup, const Problem &problem, int runs, double timeout)
{
    ompl::base::PlannerPtr kpiece(new ompl::geometric::KPIECE1(setup->getSpaceInformation()));
    ompl::base::PlannerPtr rrt(new ompl::geometric::RRT(setup->getSpaceInformation()));
    ompl::base::PlannerPtr rlrt(new ompl::geometric::RLRT(setup->getSpaceInformation()));
    ompl::base::PlannerPtr stride(new ompl::geometric::STRIDE(setup->getSpaceInformation()));
    ompl::base::PlannerPtr tsrrt(new ompl::geometric::TSRRT(setup->getSpaceInformation(), getTaskSpaceConfig(problem)));

    ompl::base::PlannerPtr rrtc(new ompl::geometric::RRTConnect(setup->getSpaceInformation()));
    ompl::base::PlannerPtr bkpiece(new ompl::geometric::BKPIECE1(setup->getSpaceInformation()));
    ompl::base::PlannerPtr birlrt(new ompl::geometric::BiRLRT(setup->getSpaceInformation()));

    const int numLinks = problem.manipulator.getNumLinks();
    const int xySlices = std::max(2, numLinks / 3);
    ompl::base::PlannerPtr xxl(new ompl::geometric::XXL(setup->getSpaceInformation(),
                                                        getXXLDecomp(setup->getSpaceInformation(), problem, xySlices)));
    ompl::base::PlannerPtr xxl1(new ompl::geometric::XXL(
        setup->getSpaceInformation(), getXXLDecomp(setup->getSpaceInformation(), problem, /*xySlices*/ 1)));
    xxl1->setName("XXL1");

    std::string name ="PlanarManipulator - " + problem.name;
    ompl::tools::Benchmark benchmark(*setup, name);

    benchmark.addPlanner(rrt);
    benchmark.addPlanner(rrtc);
    benchmark.addPlanner(rlrt);
    benchmark.addPlanner(birlrt);
    benchmark.addPlanner(stride);
    benchmark.addPlanner(kpiece);
    benchmark.addPlanner(bkpiece);
    benchmark.addPlanner(xxl);
    benchmark.addPlanner(xxl1);
    benchmark.addPlanner(tsrrt);

    benchmark.setPostRunEvent([&](const ompl::base::PlannerPtr &planner, ompl::tools::Benchmark::RunProperties &run) {
        postRunEvent(planner, run, &problem.manipulator);
    });
    benchmark.addExperimentParameter("num_links", "INTEGER", boost::lexical_cast<std::string>(numLinks));
    benchmark.addExperimentParameter("cells", "INTEGER", boost::lexical_cast<std::string>(xySlices));

    double memoryLimit = 8192.0;  // MB. XXL requires an XXL amount of memory
    ompl::tools::Benchmark::Request request(timeout, memoryLimit, runs);
    benchmark.benchmark(request);

    benchmark.saveResultsToFile();
}

void WriteVisualization(const Problem &problem, const ompl::geometric::PathGeometric &path, int xySlices)
{
    const int numLinks = problem.manipulator.getNumLinks();

    const char *world_file = "world.yaml";
    OMPL_INFORM("Writing world to %s", world_file);
    problem.world.writeWorld(world_file);

    const double linkLength = 1.0 / numLinks;
    const Eigen::Affine2d &basePose = problem.manipulator.getBaseFrame();

    const char *path_file = "manipulator_path.txt";
    OMPL_INFORM("Writing path to %s", path_file);
    std::ofstream fout;

    // This is a proprietary format for the python visualization script.
    fout.open(path_file);
    // Preamble.
    fout << numLinks << " " << linkLength << " " << basePose.translation()(0) << " " << basePose.translation()(1) << " "
         << xySlices << std::endl;
    // Write each state on the interpolated path.
    for (size_t i = 0; i < path.getStateCount(); ++i)
    {
        const double *angles = path.getState(i)->as<PlanarManipulatorStateSpace::StateType>()->values;
        for (size_t j = 0; j < problem.manipulator.getNumLinks(); ++j)
            fout << angles[j] << " ";
        fout << std::endl;
    }
    fout.close();
}

void SolveProblem(ompl::geometric::SimpleSetupPtr setup, const Problem &problem, double timeout, bool write_viz_out)
{
    // Solve the problem with XXL.
    const int numLinks = problem.manipulator.getNumLinks();
    // The number of grid cells in each dimension of the workspace decomposition.
    const int xySlices = std::max(2, numLinks / 3);
    ompl::base::PlannerPtr xxl(new ompl::geometric::XXL(setup->getSpaceInformation(),
                                                        getXXLDecomp(setup->getSpaceInformation(), problem, xySlices)));
    setup->setPlanner(xxl);

    // SOLVE!
    ompl::base::PlannerStatus status = setup->solve(timeout);

    if (status == ompl::base::PlannerStatus::EXACT_SOLUTION ||
        status == ompl::base::PlannerStatus::APPROXIMATE_SOLUTION)
    {
        ompl::geometric::PathGeometric &pgeo = setup->getSolutionPath();
        OMPL_INFORM("Solution path has %d states", pgeo.getStateCount());

        if (write_viz_out)
        {
            pgeo.interpolate(250);
            WriteVisualization(problem, pgeo, xySlices);
        }
    }
    else
    {
        OMPL_WARN("Planning failed");
    }
}

void PlanarManipulatorPlanning(const Arguments &args)
{
    ompl::msg::setLogLevel(ompl::msg::LOG_INFO);

    Problem problem = CreateProblem(args);
    ompl::geometric::SimpleSetupPtr setup = setupOMPL(problem);

    if (args.numRuns == 1)
        SolveProblem(setup, problem, args.timeout, args.viz);
    else
        BenchmarkProblem(setup, problem, args.numRuns, args.timeout);
}

int main(int argc, char **argv)
{
    Arguments args;

    // Read args
    namespace po = boost::program_options;
    po::options_description desc("Allowed options");
    // clang-format off
    desc.add_options()
        ("help,?", "Show this message")
        ("links,l", po::value<int>(&args.numLinks)->default_value(10),
            "Set the number of links in the chain")
        ("runs,r", po::value<int>(&args.numRuns)->default_value(1),
            "The number of times to execute the query.  >1 implies benchmarking")
        ("timeout,t", po::value<double>(&args.timeout)->default_value(60.0),
            "The maximum time (seconds) before failure is declared")
        ("problem,p", po::value<std::string>(&args.problem)->default_value("corridor"),
            "The name of the problem [corridor,constricted] to solve")
        ("viz,v", po::bool_switch(&args.viz)->default_value(false),
            "Write visualization output to disk.  Only works when runs = 1");
    //  clang-format on

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help"))
    {
        std::cout << desc << std::endl;
        return 0;
    }
    PlanarManipulatorPlanning(args);
}
