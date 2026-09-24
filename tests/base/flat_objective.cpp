/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2026, Rice University
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
 *   * Neither the name of the Rice University nor the names of its
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

/* Author: Clayton W. Ramsey */

#define BOOST_TEST_MODULE "FlatEffortObjective"
#include <boost/test/unit_test.hpp>

#include "ompl/base/ScopedState.h"
#include "ompl/base/SpaceInformation.h"
#include "ompl/base/objectives/FlatEffortObjective.h"
#include "ompl/base/spaces/FlatStateSpace.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "ompl/geometric/SimpleSetup.h"
#include "ompl/geometric/planners/kpiece/LBKPIECE1.h"
#include "ompl/geometric/planners/rrt/RRT.h"
#include "ompl/geometric/planners/rrt/RRTConnect.h"
#include "ompl/geometric/planners/rrt/RRTstar.h"
#include "ompl/util/Console.h"

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

using namespace ompl::base;
namespace og = ompl::geometric;

namespace
{
    constexpr unsigned int ORDER = 2;
    constexpr unsigned int DIMENSION = 2;

    /** \brief A flat space over a bounded Euclidean output with a box bound on the velocity. */
    std::shared_ptr<FlatStateSpace> makeSpace()
    {
        auto output = std::make_shared<RealVectorStateSpace>(DIMENSION);
        output->setBounds(-1., 1.);
        auto space = std::make_shared<FlatStateSpace>(output, ORDER);
        space->setDerivativeBound(1, 2.);
        space->setup();
        return space;
    }

    /** \brief A square hole in the middle of the flat output plane. */
    class BoxObstacle : public StateValidityChecker
    {
    public:
        BoxObstacle(const SpaceInformationPtr &si, double half) : StateValidityChecker(si), half_(half)
        {
        }

        bool isValid(const State *state) const override
        {
            const double *values =
                state->as<FlatStateSpace::StateType>()->output()->as<RealVectorStateSpace::StateType>()->values;
            return std::abs(values[0]) > half_ || std::abs(values[1]) > half_;
        }

    private:
        double half_;
    };

    /** \brief Write a flat output and a velocity into \e state. */
    void setState(const FlatStateSpace *space, State *state, std::initializer_list<double> output,
                  std::initializer_list<double> velocity)
    {
        Eigen::MatrixXd flat(ORDER, space->getOutputDimension());
        flat.row(0) = Eigen::RowVectorXd::Map(output.begin(), static_cast<Eigen::Index>(output.size()));
        flat.row(1) = Eigen::RowVectorXd::Map(velocity.begin(), static_cast<Eigen::Index>(velocity.size()));
        space->fromFlatState(flat, state);
    }

    /** \brief Keeps every warning OMPL emits while it's alive. */
    class WarningLog : public ompl::msg::OutputHandler
    {
    public:
        WarningLog()
        {
            ompl::msg::useOutputHandler(this);
        }

        ~WarningLog() override
        {
            ompl::msg::restorePreviousOutputHandler();
        }

        void log(const std::string &text, ompl::msg::LogLevel level, const char *, int) override
        {
            if (level >= ompl::msg::LOG_WARN)
                warnings.push_back(text);
        }

        /** \brief Whether any warning so far contains \e fragment. */
        bool mentions(const std::string &fragment) const
        {
            return std::any_of(warnings.begin(), warnings.end(), [&fragment](const std::string &warning)
                               { return warning.find(fragment) != std::string::npos; });
        }

        std::vector<std::string> warnings;
    };

    /** \brief The warning the guard emits for a planner that leaves an edge checked the other way round. */
    const std::string REVERSAL_WARNING = "doesn't check every edge in the direction";

    /** \brief The warning the guard emits for an optimizing planner with no flat objective. */
    const std::string OBJECTIVE_WARNING = "without a FlatEffortObjective";

    /** \brief A problem with a box in the way, set up but with no planner yet. */
    std::shared_ptr<og::SimpleSetup> makeProblem(const std::shared_ptr<FlatStateSpace> &space)
    {
        auto setup = std::make_shared<og::SimpleSetup>(space);
        setup->setStateValidityChecker(std::make_shared<BoxObstacle>(setup->getSpaceInformation(), 0.3));

        ScopedState<> start(space), goal(space);
        setState(space.get(), start.get(), {-0.8, -0.8}, {0., 0.});
        setState(space.get(), goal.get(), {0.8, 0.8}, {0., 0.});
        setup->setStartAndGoalStates(start, goal, 0.1);
        return setup;
    }
}  // namespace

BOOST_AUTO_TEST_CASE(GoingNowhereCostsNothing)
{
    auto space = makeSpace();
    auto si = std::make_shared<SpaceInformation>(space);
    FlatEffortObjective objective(si);

    ScopedState<> state(space);
    setState(space.get(), state.get(), {0.2, -0.4}, {1., 0.5});

    // A flat state carrying a velocity has a polynomial that loops back around to it, and nobody has to
    // fly that loop to be where they already are.
    BOOST_CHECK(space->steer(state.get(), state.get()).has_value());
    BOOST_CHECK_EQUAL(objective.motionCost(state.get(), state.get()).value(), 0.);
    BOOST_CHECK_EQUAL(space->steeringCost(state.get(), state.get()), 0.);
}

BOOST_AUTO_TEST_CASE(TheHeuristicNeverExceedsGoingAround)
{
    auto space = makeSpace();
    auto si = std::make_shared<SpaceInformation>(space);
    FlatEffortObjective objective(si);

    StateSamplerPtr sampler = space->allocStateSampler();
    ScopedState<> from(space), through(space), to(space);

    for (unsigned int trial = 0; trial < 2000u; ++trial)
    {
        sampler->sampleUniform(from.get());
        sampler->sampleUniform(through.get());
        sampler->sampleUniform(to.get());

        // Going by way of a third flat state is one way of getting there, so the heuristic has to come in
        // at or under what that way costs.
        const double direct = objective.motionCostHeuristic(from.get(), to.get()).value();
        const double around = objective.motionCost(from.get(), through.get()).value() +
                              objective.motionCost(through.get(), to.get()).value();
        BOOST_REQUIRE_LE(direct, around * (1. + 1e-9));

        // Nothing is in the way, so the heuristic comes out at the cost of the edge itself.
        BOOST_REQUIRE_EQUAL(direct, objective.motionCost(from.get(), to.get()).value());
        BOOST_REQUIRE_EQUAL(direct, objective.motionCostBestEstimate(from.get(), to.get()).value());

        // make sure the steering cost matches
        const std::optional<FlatMotion> motion = space->steer(from.get(), to.get());
        BOOST_REQUIRE(motion.has_value());
        BOOST_REQUIRE_CLOSE(direct, space->getSteering().cost(*motion), 1e-9);
        BOOST_REQUIRE_CLOSE(direct, space->steeringCost(from.get(), to.get()), 1e-9);
    }
}

BOOST_AUTO_TEST_CASE(TheGuardFiresForAReversingPlanner)
{
    auto space = makeSpace();
    auto si = std::make_shared<SpaceInformation>(space);
    si->setStateValidityChecker(std::make_shared<BoxObstacle>(si, 0.3));
    si->setup();

    {
        WarningLog log;
        og::LBKPIECE1(si).setup();
        BOOST_CHECK(log.mentions(REVERSAL_WARNING));
    }

    {
        WarningLog log;
        og::RRT(si).setup();
        BOOST_CHECK(!log.mentions(REVERSAL_WARNING));
    }

    {
        // RRTConnect grows its goal tree backwards and checks those edges in the direction the path runs
        // through them, so its paths hold up and it has nothing to answer for here.
        WarningLog log;
        og::RRTConnect(si).setup();
        BOOST_CHECK(!log.mentions(REVERSAL_WARNING));
    }
}

BOOST_AUTO_TEST_CASE(TheGuardFiresForAnOptimizingPlannerWithoutTheObjective)
{
    {
        auto space = makeSpace();
        auto setup = makeProblem(space);
        setup->setPlanner(std::make_shared<og::RRTstar>(setup->getSpaceInformation()));

        WarningLog log;
        setup->setup();
        BOOST_CHECK(log.mentions(OBJECTIVE_WARNING));
    }

    {
        auto space = makeSpace();
        auto setup = makeProblem(space);
        setup->setPlanner(std::make_shared<og::RRTstar>(setup->getSpaceInformation()));
        setup->setOptimizationObjective(std::make_shared<FlatEffortObjective>(setup->getSpaceInformation()));

        WarningLog log;
        setup->setup();
        BOOST_CHECK(!log.mentions(OBJECTIVE_WARNING));
    }

    {
        // RRT takes whatever path it finds first, so it has nothing to optimize and nothing to warn about.
        auto space = makeSpace();
        auto setup = makeProblem(space);
        setup->setPlanner(std::make_shared<og::RRT>(setup->getSpaceInformation()));

        WarningLog log;
        setup->setup();
        BOOST_CHECK(!log.mentions(OBJECTIVE_WARNING));
    }
}
