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
#include <ompl/base/ScopedState.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/informedtrees/BITstar.h>
#include <ompl/geometric/planners/informedtrees/DDLBITstar.h>
#include <ompl/util/RandomNumbers.h>
#include <ompl/util/Console.h>
#include <chrono>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <string>

int main(int argc, char **argv)
{
    if (argc != 5)
    {
        std::cerr << "Usage: " << argv[0] << " BITstar|DDLBITstar open|passage|clutter|blocked seed seconds\n";
        return 1;
    }
    try
    {
        const std::string plannerName(argv[1]), scenario(argv[2]);
        if ((plannerName != "BITstar" && plannerName != "DDLBITstar") ||
            (scenario != "open" && scenario != "passage" && scenario != "clutter" && scenario != "blocked"))
            throw std::invalid_argument("Unknown planner or scenario");
        const auto seed = static_cast<unsigned int>(std::stoul(argv[3]));
        const double seconds = std::stod(argv[4]);
        if (!std::isfinite(seconds) || seconds <= 0.0)
            throw std::invalid_argument("Time budget must be finite and positive");
        ompl::RNG::setSeed(seed);
        ompl::msg::setLogLevel(ompl::msg::LOG_ERROR);
        namespace ob = ompl::base;
        namespace og = ompl::geometric;
        auto space = std::make_shared<ob::RealVectorStateSpace>(2);
        ob::RealVectorBounds bounds(2);
        bounds.setLow(-1.0);
        bounds.setHigh(1.0);
        space->setBounds(bounds);
        og::SimpleSetup setup(space);
        setup.setStateValidityChecker(
            [scenario](const ob::State *state)
            {
                const auto *s = state->as<ob::RealVectorStateSpace::StateType>();
                const double x = s->values[0], y = s->values[1];
                if (scenario == "open")
                    return true;
                if (scenario == "passage")
                    return std::abs(x) > 0.12 || (y > 0.45 && y < 0.7);
                if (scenario == "blocked")
                    return std::abs(x) > 0.12;
                for (double cx : {-0.45, 0.0, 0.45})
                    for (double cy : {-0.45, 0.0, 0.45})
                        if ((x - cx) * (x - cx) + (y - cy) * (y - cy) < 0.15 * 0.15)
                            return false;
                return true;
            });
        setup.getSpaceInformation()->setStateValidityCheckingResolution(0.002);
        ob::ScopedState<> start(space), goal(space);
        start[0] = -0.8;
        start[1] = 0.0;
        goal[0] = 0.8;
        goal[1] = 0.0;
        std::shared_ptr<og::BITstar> planner;
        std::shared_ptr<og::DDLBITstar> ddl;
        if (plannerName == "DDLBITstar")
        {
            ddl = std::make_shared<og::DDLBITstar>(setup.getSpaceInformation());
            ddl->setGridResolution(0.15);
            ddl->setBoundaryResolution(0.025);
            ddl->setBoundaryStandardDeviation(0.08);
            planner = ddl;
        }
        else
            planner = std::make_shared<og::BITstar>(setup.getSpaceInformation());
        planner->setSamplesPerBatch(50);
        planner->setRewireFactor(1.1);
        setup.setStartAndGoalStates(start, goal);
        setup.setPlanner(planner);
        setup.setup();
        double firstSolution = -1.0;
        const auto begin = std::chrono::steady_clock::now();
        setup.getProblemDefinition()->setIntermediateSolutionCallback(
            [&](const ob::Planner *, const std::vector<const ob::State *> &, const ob::Cost)
            {
                if (firstSolution < 0.0)
                    firstSolution = std::chrono::duration<double>(std::chrono::steady_clock::now() - begin).count();
            });
        setup.solve(seconds);
        const auto elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - begin).count();
        const bool exact = setup.haveExactSolutionPath();
        const bool valid = exact && setup.getSolutionPath().check();
        ob::PlannerData data(setup.getSpaceInformation());
        planner->getPlannerData(data);
        std::cout << std::setprecision(10) << scenario << ',' << plannerName << ',' << seed << ',' << exact << ','
                  << valid << ',' << firstSolution << ',' << planner->bestCost().value() << ',' << data.numVertices()
                  << ',' << planner->numBatches() << ',' << (ddl ? ddl->numBoundaryPoints() : 0) << ','
                  << (ddl ? ddl->numBoundarySamples() : 0) << ',' << (ddl ? ddl->numDensityRejections() : 0) << ','
                  << (ddl ? ddl->numGridRebuilds() : 0) << ',' << elapsed << '\n';
        return exact && !valid ? 2 : 0;
    }
    catch (const std::exception &error)
    {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
