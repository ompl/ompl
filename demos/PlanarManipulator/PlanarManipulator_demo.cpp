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

#include "boost/program_options.hpp"

#include <fstream>
#include <sstream>

#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/xxl/XXL.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/est/EST.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/stride/STRIDE.h>
#include <ompl/geometric/planners/prm/LazyPRM.h>
#include <ompl/geometric/planners/rrt/TSRRT.h>
#include <ompl/geometric/planners/rlrt/RLRT.h>

// bi directional
#include <ompl/geometric/planners/sbl/SBL.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/kpiece/LBKPIECE1.h>
#include <ompl/geometric/planners/kpiece/BKPIECE1.h>
#include <ompl/geometric/planners/rlrt/BiRLRT.h>

#include <ompl/tools/benchmark/Benchmark.h>

#include "PlanarManipulator.h"
#include "PlanarManipulatorStateSpace.h"
#include "PlanarManipulatorIKGoal.h"
#include "PlanarManipulatorXXLDecomposition.h"
#include "PlanarManipulatorTSRRTConfig.h"
#include "BoundedPlanarManipulatorStateSpace.h"
#include "LinearMotionValidator.h"
#include "PMLinearMotionValidator.h"
#include "PlanarManipulator_demo.h"

struct Arguments
{
    int numLinks;
    int numRuns;
    double timeout;
    std::string name;
    bool cartesianInterpolate;
    bool verbose;
    double bounds;
};

ompl::geometric::SimpleSetupPtr createProblem(PlanarManipulator* manipulator, bool cartesianInterpolator, double bound = M_PI)
{
    ompl::base::StateSpacePtr space(new BoundedPlanarManipulatorStateSpace(manipulator));
    ompl::base::RealVectorBounds bounds(manipulator->getNumLinks());
    bounds.setLow(-bound);
    bounds.setHigh(bound);

    space->as<BoundedPlanarManipulatorStateSpace>()->setBounds(bounds);
    manipulator->setBounds(bounds.low, bounds.high);

    if (cartesianInterpolator)
        space->as<BoundedPlanarManipulatorStateSpace>()->useCartesianInterpolator(true);

    ompl::geometric::SimpleSetupPtr setup(new ompl::geometric::SimpleSetup(space));

    // Custom motion validator required for Cartesian interpolation scheme and to prevent self intersection.
    ompl::base::MotionValidatorPtr mv(new PMLinearMotionValidator(setup->getSpaceInformation(), manipulator));
    setup->getSpaceInformation()->setMotionValidator(mv);

    // Increase resolution to mitigate collisions
    setup->getSpaceInformation()->setStateValidityCheckingResolution(0.001);

    return setup;
}

std::pair<int,bool> getGoalInfo(unsigned int step)
{
    int goalSeg = 0;
    bool normNegY = false;
    switch(step)
    {
        case 0:
            goalSeg = 4;
            normNegY = false;  // we want end to point 'up'
            break;
        case 1:
            goalSeg = 39;
            normNegY = false;
            break;
        case 2:
            goalSeg = 20;
            normNegY = true;  // end should point 'down'
            break;
        case 3:
            goalSeg = 28;
            normNegY = true;
            break;
        default:
            OMPL_ERROR("STEP NUMBER = %u", step);
            throw ompl::Exception("Unknown step number");
    }

    return std::make_pair(goalSeg, normNegY);
}

void setStartAndGoal(ompl::geometric::SimpleSetupPtr setup, const PlanarManipulator* manip, const Environment& env, const double* init, unsigned int step)
{
    ompl::base::ScopedState<> start(setup->getStateSpace());
    for(unsigned int i = 0; i < manip->getNumLinks(); ++i)
        start[i] = init[i];
    setup->setStartState(start);

    std::pair<int, bool> goalInfo = getGoalInfo(step);

    const Segment& seg = env[goalInfo.first];
    Eigen::Affine2d goalPose = Eigen::Affine2d::Identity();
    goalPose.translation()(0) = (seg.second.first + seg.first.first) / 2.0;
    goalPose.translation()(1) = (seg.second.second + seg.first.second) / 2.0;

    double dx = seg.second.first - seg.first.first;
    double dy = seg.second.second - seg.first.second;
    double normX = -dy, normY = dx;
    // Correct sign on normal vector, depending on the desired direction
    if ((goalInfo.second && normY > 0.0) || (!goalInfo.second && normY < 0.0))
    {
        normX = -normX;
        normY = -normY;
    }

    double angle = atan2(normY, normX);
    goalPose.rotate(angle);

    // Translate the end position slightly away from the wall to avoid issues with collision checker
    Eigen::Vector2d p(goalPose.translation());
    Eigen::Vector2d v(normX, normY);
    v.normalize();
    p = p - 0.01 * v;
    goalPose.translation() = p;

    ompl::base::GoalPtr goal(new PlanarManipulatorIKGoal(setup->getSpaceInformation(), goalPose, manip, false));  // true means fixed orientation
    goal->as<PlanarManipulatorIKGoal>()->setThreshold(1e-3); // set higher to avoid issues with numerical precision
    setup->setGoal(goal);
}

void postRunEvent(const ompl::base::PlannerPtr &planner, ompl::tools::Benchmark::RunProperties &run, const PlanarManipulator* manip)
{
    if(planner->getProblemDefinition()->hasSolution())
    {
        double cartesianDist = 0.0;
        const ompl::geometric::PathGeometric& path = static_cast<const ompl::geometric::PathGeometric&>(*(planner->getProblemDefinition()->getSolutionPath().get()));
        for(size_t i = 0; i < path.getStateCount()-1; ++i)
        {
            std::vector<Eigen::Affine2d> startFrames, endFrames;
            manip->FK(path.getState(i  )->as<PlanarManipulatorStateSpace::StateType>()->values, startFrames);
            manip->FK(path.getState(i+1)->as<PlanarManipulatorStateSpace::StateType>()->values, endFrames);

            for(size_t j = 1; j < endFrames.size(); ++j)
                cartesianDist += (endFrames[j].translation() - startFrames[j].translation()).norm();
        }

        run["Cartesian Distance REAL"] = boost::lexical_cast<std::string>(cartesianDist);
    }
}

void planarManipulatorPlanning(const Arguments& args, PlanarManipulator& manipulator, const PolyWorld& world, const Eigen::Affine2d& goalPose)
{
    std::pair<double, double> xBounds = world.getXBounds(); //std::make_pair(0, 1.25);
    std::pair<double, double> yBounds = world.getYBounds();

    int numLinks = args.numLinks;
    double linkLength = 1.0 / numLinks;

    world.writeWorld("test_world.yaml");

    bool useCartInterpolator = args.cartesianInterpolate;
    ompl::geometric::SimpleSetupPtr setup = createProblem(&manipulator, useCartInterpolator, args.bounds);

    ompl::base::ScopedState<> start(setup->getStateSpace());
    for(unsigned int i = 0; i < manipulator.getNumLinks(); ++i)
        start[i] = 1e-7;  // zero is dangerous, since the chain will lie on a decomposition boundary.
    setup->setStartState(start);

    ompl::base::GoalPtr goal(new PlanarManipulatorIKGoal(setup->getSpaceInformation(), goalPose, &manipulator, false)); // false means orientation is not fixed
    goal->as<PlanarManipulatorIKGoal>()->setThreshold(1e-3); // set higher to avoid issues with numerical precision
    setup->setGoal(goal);

    setup->setStateValidityChecker(std::make_shared<IsValidPolyWorld>(setup->getSpaceInformation(), &manipulator, &world));

    // Planners we might care aboot
    int xySlices = std::max(2, numLinks/3);  // at least two cells in each dimension
    ompl::geometric::XXLDecompositionPtr xxldecomp = getXXLDecomp(setup->getSpaceInformation(), &manipulator, xySlices, 1, xBounds, yBounds);
    ompl::base::PlannerPtr xxl(new ompl::geometric::XXL(setup->getSpaceInformation(), xxldecomp));

    ompl::geometric::XXLDecompositionPtr xxl1decomp = getXXLDecomp(setup->getSpaceInformation(),
                                                                   &manipulator, /*xySlices=*/1,
                                                                   /*thetaslices=*/1, xBounds, yBounds);
    ompl::base::PlannerPtr xxl1(new ompl::geometric::XXL(setup->getSpaceInformation(), xxl1decomp));
    xxl1->setName("XXL1");

    ompl::base::PlannerPtr kpiece(new ompl::geometric::KPIECE1(setup->getSpaceInformation()));
    ompl::base::PlannerPtr rrt(new ompl::geometric::RRT(setup->getSpaceInformation()));
    //ompl::base::PlannerPtr prm(new ompl::geometric::PRM(setup->getSpaceInformation()));
    //ompl::base::PlannerPtr lazyprm(new ompl::geometric::LazyPRM(setup->getSpaceInformation()));
    ompl::base::PlannerPtr stride(new ompl::geometric::STRIDE(setup->getSpaceInformation()));
    ompl::base::PlannerPtr rlrt(new ompl::geometric::RLRT(setup->getSpaceInformation()));
    ompl::base::PlannerPtr birlrt(new ompl::geometric::BiRLRT(setup->getSpaceInformation()));

    //ompl::base::PlannerPtr randomWalk(new ompl::geometric::RandomWalk(setup->getSpaceInformation()));
    ompl::base::PlannerPtr rrtc(new ompl::geometric::RRTConnect(setup->getSpaceInformation()));
    ompl::base::PlannerPtr bkpiece(new ompl::geometric::BKPIECE1(setup->getSpaceInformation()));

    double chainLength = numLinks * linkLength;
    const Eigen::Affine2d& basePose = manipulator.getBaseFrame();
    Point2D origin = std::make_pair(basePose.translation()(0), basePose.translation()(1));
    double xMin = std::max(origin.first - chainLength, xBounds.first);
    double xMax = std::min(origin.first + chainLength, xBounds.second);
    double yMin = std::max(origin.second - chainLength, yBounds.first);
    double yMax = std::min(origin.second + chainLength, yBounds.second);

    ompl::base::RealVectorBounds tsBounds(2);
    tsBounds.low[0] = xMin;
    tsBounds.low[1] = yMin;
    tsBounds.high[0] = xMax;
    tsBounds.high[1] = yMax;

    ompl::geometric::TaskSpaceConfigPtr task_space_ptr(new PlanarManipTaskSpaceConfig(&manipulator, tsBounds));
    ompl::base::PlannerPtr tsrrt(new ompl::geometric::TSRRT(setup->getSpaceInformation(), task_space_ptr));

    if (args.verbose)
    {
        OMPL_INFORM("Planar manipulator %s", (args.numRuns > 1 ? "benchmarking " : "planning"));
        OMPL_INFORM("      Links: %d", args.numLinks);
        OMPL_INFORM(" Jnt limits: [-%f, %f]", args.bounds, args.bounds);
        OMPL_INFORM("     # Runs: %d", args.numRuns);
        OMPL_INFORM("     Slices: %d", xySlices);
        OMPL_INFORM("  Cartesian: %d", args.cartesianInterpolate);
        OMPL_INFORM("    Timeout: %f", args.timeout);
        OMPL_INFORM("       Name: %s", args.name.c_str());
    }


    // One planning call
    if (args.numRuns == 1)
    {
        setup->setPlanner(xxl);
        //setup->setPlanner(xxl1);
        // setup->setPlanner(rrt);
        // setup->setPlanner(rrtc);
        // setup->setPlanner(kpiece);
        // setup->setPlanner(bkpiece);
        //setup->setPlanner(tsrrt);

        // setup->setup();
        // setup->print();

        ompl::base::PlannerStatus status = setup->solve(args.timeout);

        if (status == ompl::base::PlannerStatus::EXACT_SOLUTION
            || status == ompl::base::PlannerStatus::APPROXIMATE_SOLUTION)
        {
            // Write path to file
            ompl::geometric::PathGeometric& pgeo = setup->getSolutionPath();

            if (useCartInterpolator)
            {
                std::cout << "original solution path has " << pgeo.getStateCount() << " states" << std::endl;

                // just create a new path.  way easier
                ompl::geometric::PathGeometric newpath(setup->getSpaceInformation());
                newpath.append(pgeo.getState(0));

                ompl::base::State* xstate = setup->getSpaceInformation()->allocState();
                for(size_t i = 1; i < pgeo.getStateCount(); ++i)
                {
                    int intermediateStates = std::max((int) (30 * setup->getSpaceInformation()->distance(pgeo.getState(i-1), pgeo.getState(i))), 30);
                    double dt = 1.0 / (intermediateStates + 1);
                    double t = dt;
                    while (t < (1.0 - dt/2.0))
                    {
                        setup->getSpaceInformation()->getStateSpace()->interpolate(pgeo.getState(i-1), pgeo.getState(i), t, xstate);
                        newpath.append(xstate);
                        t += dt;
                    }
                    newpath.append(pgeo.getState(i));
                }

                pgeo = newpath;

                setup->getSpaceInformation()->freeState(xstate);
            }
            else
                pgeo.interpolate(500);

            if (!pgeo.check())
                std::cerr << "ERROR: Interpolated path is not valid" << std::endl;
            for(size_t i = 0; i < pgeo.getStateCount(); ++i)
            {
                if (!setup->getSpaceInformation()->satisfiesBounds(pgeo.getState(i)))
                    std::cerr << "ERROR: State " << i << " / " << pgeo.getStateCount()-1 << " does NOT satisfy joint limits" << std::endl;
            }

            std::cout << "Solution path has " << pgeo.getStateCount() << " states" << std::endl;

            std::ofstream fout;
            std::stringstream str;
            str << "solution_" << numLinks << "_" << xySlices << ".txt";
            fout.open(str.str().c_str());
            fout << numLinks << " " << linkLength << " " << basePose.translation()(0) << " " << basePose.translation()(1) << " " << acos(basePose.matrix()(0,0)) << " " << xySlices << std::endl;//" " << args.dbDir << " " << args.dbFile << std::endl;
            for(size_t i = 0; i < pgeo.getStateCount(); ++i)
            {
                const double* angles = pgeo.getState(i)->as<PlanarManipulatorStateSpace::StateType>()->values;
                for(size_t j = 0; j < manipulator.getNumLinks(); ++j)
                    fout << angles[j] << " ";
                fout << std::endl;
            }
            fout.close();
        }
    }
    else // benchmark
    {
        int numRuns = args.numRuns;
        double timeout = args.timeout; // secs
        double memoryLimit = 8192.0;  // MB.  set high because XXL requires an XXL amount of memory

        std::string name = args.name;//"climbing_benchmark";

        ompl::tools::Benchmark::Request request(timeout, memoryLimit, numRuns);
        if (useCartInterpolator)
            request.simplify = false;

        ompl::tools::Benchmark benchmark(*(setup.get()), name);
        benchmark.addPlanner(rrt);
        benchmark.addPlanner(tsrrt);
        benchmark.addPlanner(xxl);
        benchmark.addPlanner(stride);
        benchmark.addPlanner(kpiece);
        benchmark.addPlanner(bkpiece);
        benchmark.addPlanner(rrt);
        benchmark.addPlanner(rrtc);
        benchmark.addPlanner(rlrt);
        benchmark.addPlanner(birlrt);
        benchmark.addPlanner(xxl1);

        benchmark.setPostRunEvent([manipulator](const ompl::base::PlannerPtr &planner,
                                                ompl::tools::Benchmark::RunProperties &run)
                                                { postRunEvent(planner, run, &manipulator); });

        benchmark.addExperimentParameter("num_links", "INTEGER", boost::lexical_cast<std::string>(numLinks));
        benchmark.addExperimentParameter("cells", "INTEGER", boost::lexical_cast<std::string>(xySlices));

        benchmark.benchmark(request);
        // TOTAL HACK FOR THE OUTPUT FILENAME:
        const std::string output_dir = "/planarresults/";
        benchmark.saveResultsToFile(std::string(output_dir + name + ".log").c_str());
    }
}

int main(int argc, char** argv)
{
    Arguments args;
    char environment;

    // Read args
    namespace po = boost::program_options;
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help,?",                                                                          "Show this message")
        ("links,l",   po::value<int>(&args.numLinks)->      default_value(10),              "Set the number of links in the chain")
        ("runs,r",    po::value<int>(&args.numRuns)->       default_value(1),               "The number of times to execute the query.  >1 implies benchmarking")
        ("timeout,t", po::value<double>(&args.timeout)->    default_value(60),              "The maximum time (seconds) before failure is declared")
        ("name,n",    po::value<std::string>(&args.name)->  default_value("PM"),            "The name of this experiment")
        ("cartInt,c", po::value<bool>(&args.cartesianInterpolate)->default_value(true),     "Enable/disable cartesian space interpolation")
        ("verbose,v", po::value<bool>(&args.verbose)->      default_value(true),            "Turn on/off verbose output")
        ("env,e",     po::value<char>(&environment)->       default_value('r'),             "Select an environment")
        ("bounds,b",  po::value<double>(&args.bounds)->     default_value(3.14),            "The (symmetric) bounds on the joint angles (0,pi)");

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help"))
        std::cout << desc << std::endl;
    else
    {
        std::pair<double, double> xBounds = std::make_pair(0, 1.25);
        std::pair<double, double> yBounds = std::make_pair(0, 1.25);
        PolyWorld world("simpleworld", xBounds, yBounds);

        PlanarManipulator manipulator(args.numLinks, 1.0 / args.numLinks);

        // where the chain starts
        Eigen::Affine2d basePose = Eigen::Affine2d::Identity();

        // where we want the chain to end up
        Eigen::Affine2d goalPose = Eigen::Affine2d::Identity();
        switch(environment)
        {
            // Rectangle world
            case 'r':
            case 'R':
            {
                double gap = (1.0 / args.numLinks) * (1.1 * M_PI + log(args.numLinks)/(double)args.numLinks);
                double w = ((xBounds.second - xBounds.first) * 0.50) - gap;
                goalPose.translation()(0) = (w + gap) * 0.95;  // problem becomes impossible when this value
                //goalPose.translation()(1) = 0.50;
                goalPose.translation()(1) = std::min(4.0 * gap, 0.50);  // need to move y down.  as num links increases, the obstacle gets bigger and eventually the goal position is infeasible to reach
                goalPose.rotate(0);

                // set base pose
                basePose.translation()(1) = gap/2.0;

                createRectangleEnvironment(args.numLinks, 1.0 / args.numLinks, world);

                if (args.verbose)
                    OMPL_INFORM("Planning in the rectangle world");
                break;
            }

            // Clutter world
            case 'c':
            case 'C':
            {
                createClutterEnvironment(args.numLinks, 1.0 / args.numLinks, world);

                double gap = (1.0 / args.numLinks) * (1.1 * M_PI + log(args.numLinks)/(double)args.numLinks);
                double y1 = 0.25;
                double height = 0.05;
                double y2 = y1 + height + 0.75*gap;

                goalPose.translation()(0) = 0.25 + gap - (gap / 4.0);
                goalPose.translation()(1) = y2;
                goalPose.rotate(0);

                // set base pose
                basePose.translation()(1) = gap/2.0;

                if (args.verbose)
                    OMPL_INFORM("Planning in the clutter world");

                break;
            }

            // Left-side constricted world
            case 'l':
            case 'L':
            {
                double gap = (log10(args.numLinks) / (double)args.numLinks) * 2.0;
                //double gap = 2.0 / args.numLinks;
                OMPL_INFORM("GAP: %.3f  Links: %d", gap, args.numLinks);

                //goalPose.translation()(0) = 0.5 + 1.5 * gap;
                goalPose.translation()(0) = 0.75;
                goalPose.translation()(1) = 0.5 + 1.5 * gap;
                goalPose.rotate(0);

                // set base pose
                basePose.translation()(1) = 0.50;

                // No extra homotopies at 18-20, but not too hard for bidirectional planners.
                // createConstrictedEnvironment2(gap, world);
                // if (args.verbose) OMPL_INFORM("Planning in the constricted world 2");

                createConstrictedEnvironment3(gap, args.numLinks, world);
                if (args.verbose) OMPL_INFORM("Planning in the constricted world 3");
                break;
            }
            case 't':
            case 'T':
            {
                const double goal_y = createTunnelEnvironment(args.numLinks, world);

                basePose.translation()(0) = 0.0;
                basePose.translation()(1) = 0.4;

                goalPose.translation()(0) = 0.75;
                goalPose.translation()(1) = goal_y;

                break;
            }

            default:
            {
                std::cerr << "Unknown environment entry \"" << environment << "\"" << std::endl;
                throw;
            }
        }

        // std::cout << "Base pose: " << std::endl;
        // std::cout << basePose.matrix() << std::endl;
        manipulator.setBaseFrame(basePose);
        //createRectangleEnvironment(args.numLinks, 1.0 / args.numLinks, world);
        //createClutterEnvironment(args.numLinks, 1.0 / args.numLinks, world);

        planarManipulatorPlanning(args, manipulator, world, goalPose);
    }
}
