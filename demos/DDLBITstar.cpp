/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2026, Open Motion Planning Library contributors
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
 *   * Neither the name of the copyright holder nor the names of its
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

/* Author: Elias (OMPL contribution) */
#include "PlannerComparisonGeometry.h"
#include <ompl/base/ScopedState.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/informedtrees/BITstar.h>
#include <ompl/geometric/planners/informedtrees/DDLBITstar.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/util/RandomNumbers.h>
#include <ompl/util/Console.h>
#include <chrono>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <string>
#include <algorithm>
#include <array>
#include <limits>
#include <vector>

int main(int argc, char **argv)
{
    if (argc != 5)
    {
        std::cerr << "Usage: " << argv[0] << " RRTstar|RRTConnect|InformedRRTstar|BITstar|DDLBITstar"
                  << " open|passage|narrow|maze|surround|clutter|passage6d|blocked seed seconds\n";
        return 1;
    }
    try
    {
        const std::string plannerName(argv[1]), scenario(argv[2]);
        if ((plannerName != "BITstar" && plannerName != "DDLBITstar" && plannerName != "RRTstar" &&
             plannerName != "InformedRRTstar" && plannerName != "RRTConnect") ||
            (scenario != "open" && scenario != "passage" && scenario != "clutter" && scenario != "blocked" &&
             scenario != "narrow" && scenario != "maze" && scenario != "surround" && scenario != "passage6d"))
            throw std::invalid_argument("Unknown planner or scenario");
        const auto seed = static_cast<unsigned int>(std::stoul(argv[3]));
        const double seconds = std::stod(argv[4]);
        if (!std::isfinite(seconds) || seconds <= 0.0)
            throw std::invalid_argument("Time budget must be finite and positive");
        ompl::RNG::setSeed(seed);
        ompl::msg::setLogLevel(ompl::msg::LOG_ERROR);
        namespace ob = ompl::base;
        namespace og = ompl::geometric;
        const unsigned int dimension = scenario == "passage6d" ? 6u : 2u;
        auto space = std::make_shared<ob::RealVectorStateSpace>(dimension);
        ob::RealVectorBounds bounds(dimension);
        bounds.setLow(-1.0);
        bounds.setHigh(1.0);
        space->setBounds(bounds);
        og::SimpleSetup setup(space);
        std::size_t validityChecks = 0;
        const PlannerComparisonWorld world(scenario);
        const auto validConfiguration = [&world](const ob::State *state) { return world.valid(state); };
        std::size_t motionChecks = 0;
        setup.setStateValidityChecker(
            [&](const ob::State *state)
            {
                ++validityChecks;
                return validConfiguration(state);
            });
        setup.getSpaceInformation()->setMotionValidator(
            std::make_shared<PlannerComparisonMotionValidator>(setup.getSpaceInformation(), world, motionChecks));
        setup.getSpaceInformation()->setStateValidityCheckingResolution(0.001);
        ob::ScopedState<> start(space), goal(space);
        for (unsigned int i = 0; i < dimension; ++i)
            start[i] = goal[i] = 0.0;
        start[0] = -0.9;
        goal[0] = 0.9;
        if (dimension == 6)
            for (unsigned int i = 2; i < dimension; ++i)
            {
                start[i] = -0.5;
                goal[i] = 0.5;
            }
        if (scenario == "surround")
        {
            start[0] = goal[0] = 0.0;
            start[1] = -0.25;
            goal[1] = -0.85;
        }
        std::shared_ptr<ob::Planner> planner;
        std::shared_ptr<og::DDLBITstar> ddl;
        std::shared_ptr<og::BITstar> bit;
        if (plannerName == "DDLBITstar")
        {
            ddl = std::make_shared<og::DDLBITstar>(setup.getSpaceInformation());
            // Rescale Table 1's 500-unit map parameters to the width-2 domain.
            ddl->setGridResolution(0.06);
            ddl->setBoundaryResolution(0.008);
            ddl->setBoundaryStandardDeviation(0.04);
            bit = ddl;
            planner = bit;
        }
        else if (plannerName == "BITstar")
        {
            bit = std::make_shared<og::BITstar>(setup.getSpaceInformation());
            planner = bit;
        }
        else if (plannerName == "RRTConnect")
        {
            auto connect = std::make_shared<og::RRTConnect>(setup.getSpaceInformation());
            connect->setRange(0.1);
            planner = connect;
        }
        else
        {
            std::shared_ptr<og::RRTstar> rrt = plannerName == "RRTstar" ?
                                                   std::make_shared<og::RRTstar>(setup.getSpaceInformation()) :
                                                   std::make_shared<og::InformedRRTstar>(setup.getSpaceInformation());
            rrt->setRange(0.1);
            rrt->setGoalBias(0.1);
            rrt->setRewireFactor(1.1);
            planner = rrt;
        }
        if (bit)
        {
            bit->setSamplesPerBatch(50);
            bit->setRewireFactor(1.1);
        }
        setup.setStartAndGoalStates(start, goal);
        setup.setOptimizationObjective(
            std::make_shared<ob::PathLengthOptimizationObjective>(setup.getSpaceInformation()));
        setup.setPlanner(planner);
        setup.setup();
        double firstSolution = -1.0;
        double firstCost = std::numeric_limits<double>::infinity();
        std::vector<std::pair<double, double>> improvements;
        const auto begin = std::chrono::steady_clock::now();
        setup.getProblemDefinition()->setIntermediateSolutionCallback(
            [&](const ob::Planner *, const std::vector<const ob::State *> &, const ob::Cost cost)
            {
                const auto time = std::chrono::duration<double>(std::chrono::steady_clock::now() - begin).count();
                if (firstSolution < 0.0)
                {
                    firstSolution = time;
                    firstCost = cost.value();
                }
                improvements.emplace_back(time, cost.value());
            });
        const auto deadline = begin + std::chrono::duration<double>(seconds);
        setup.solve(
            ob::PlannerTerminationCondition([deadline] { return std::chrono::steady_clock::now() >= deadline; }));
        const auto elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - begin).count();
        const auto planningChecks = validityChecks;
        const auto planningMotionChecks = motionChecks;
        const bool exact = setup.haveExactSolutionPath();
        double finalCost = exact ? setup.getSolutionPath().length() : std::numeric_limits<double>::infinity();
        if (exact && firstSolution < 0.0)
        {
            // RRT-Connect returns its first path without intermediate callbacks.
            firstSolution = elapsed;
            firstCost = finalCost;
            improvements.emplace_back(elapsed, finalCost);
        }
        bool valid = exact && setup.getSolutionPath().check();
        if (valid)
        {
            // Independently recheck returned edges at an absolute step ten times finer
            // than the configured 2D checking resolution, using the geometric predicate.
            const auto &states = setup.getSolutionPath().getStates();
            ob::ScopedState<> check(space);
            for (std::size_t edge = 1; edge < states.size() && valid; ++edge)
            {
                valid = world.segmentValid(states[edge - 1], states[edge]);
                const auto segments = std::max(1u, static_cast<unsigned int>(std::ceil(
                                                       space->distance(states[edge - 1], states[edge]) / 0.00025)));
                for (unsigned int i = 0; i <= segments && valid; ++i)
                {
                    space->interpolate(states[edge - 1], states[edge], static_cast<double>(i) / segments, check.get());
                    valid = space->satisfiesBounds(check.get()) && validConfiguration(check.get());
                }
            }
        }
        ob::PlannerData data(setup.getSpaceInformation());
        planner->getPlannerData(data);
        std::cout << std::setprecision(10) << scenario << ',' << plannerName << ',' << seed << ',' << exact << ','
                  << valid << ',' << firstSolution << ',' << finalCost << ',' << data.numVertices() << ','
                  << (bit ? bit->numBatches() : 0) << ',' << (ddl ? ddl->numBoundaryPoints() : 0) << ','
                  << (ddl ? ddl->numBoundarySamples() : 0) << ',' << (ddl ? ddl->numDensityRejections() : 0) << ','
                  << (ddl ? ddl->numGridRebuilds() : 0) << ',' << elapsed << ',' << planningChecks << ',' << firstCost;
        for (double checkpoint : {0.05, 0.1, 0.25, 0.5})
        {
            double cost = std::numeric_limits<double>::infinity();
            if (checkpoint <= seconds)
                for (const auto &improvement : improvements)
                    if (improvement.first <= checkpoint)
                        cost = std::min(cost, improvement.second);
            std::cout << ',' << cost;
        }
        std::cout << ',' << planningMotionChecks << '\n';
        return exact && !valid ? 2 : 0;
    }
    catch (const std::exception &error)
    {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
