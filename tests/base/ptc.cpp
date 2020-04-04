/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2013, Willow Garage
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
*   * Neither the name of Willow Garage nor the names of its
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

/* Author: Ioan Sucan */

#define BOOST_TEST_MODULE "PlannerTerminationCondition"
#include <boost/test/unit_test.hpp>
#include <iostream>
#include <thread>

#include "ompl/base/spaces/SO2StateSpace.h"
#include "ompl/base/PlannerTerminationCondition.h"
#include "ompl/base/terminationconditions/IterationTerminationCondition.h"
#include "ompl/base/terminationconditions/CostConvergenceTerminationCondition.h"
#include "ompl/base/objectives/PathLengthOptimizationObjective.h"
#include "ompl/base/objectives/MaximizeMinClearanceObjective.h"
#include "ompl/util/Time.h"

using namespace ompl;

BOOST_AUTO_TEST_CASE(TestSimpleTermination)
{
  static const double dt = 0.1;
  const base::PlannerTerminationCondition &ptc = base::timedPlannerTerminationCondition(dt);
  BOOST_CHECK(!ptc);
  BOOST_CHECK(!ptc());
  std::this_thread::sleep_for(ompl::time::seconds(dt + 0.01));
  BOOST_CHECK(ptc);
  BOOST_CHECK(ptc());

  const base::PlannerTerminationCondition &ptc_long = base::timedPlannerTerminationCondition(100.0 * dt);
  BOOST_CHECK(!ptc_long);
  BOOST_CHECK(!ptc_long());
  ptc_long.terminate();
  BOOST_CHECK(ptc_long);
  BOOST_CHECK(ptc_long());
}

BOOST_AUTO_TEST_CASE(TestThreadedTermination)
{
  static const double dt = 0.2;
  static const double interval = 0.005;
  const base::PlannerTerminationCondition &ptc = base::timedPlannerTerminationCondition(dt, interval);
  BOOST_CHECK(!ptc);
  BOOST_CHECK(!ptc());
  std::this_thread::sleep_for(ompl::time::seconds(dt + interval * 3.0));
  BOOST_CHECK(ptc);
  BOOST_CHECK(ptc());

  const base::PlannerTerminationCondition &ptc_long = base::timedPlannerTerminationCondition(100.0 * dt, interval);
  BOOST_CHECK(!ptc_long);
  BOOST_CHECK(!ptc_long());
  ptc_long.terminate();
  BOOST_CHECK(ptc_long);
  BOOST_CHECK(ptc_long());
}

BOOST_AUTO_TEST_CASE(TestIterationTermination)
{
  base::IterationTerminationCondition iptc(10);
  const base::PlannerTerminationCondition ptc(iptc);
  BOOST_CHECK(!ptc);
  BOOST_CHECK(!ptc());
  for (unsigned int i = 0; i < 8; ++i)
    BOOST_CHECK(!ptc());
  BOOST_CHECK(ptc);
  BOOST_CHECK(ptc());
  BOOST_CHECK(iptc.getTimesCalled() == 12);
}

BOOST_AUTO_TEST_CASE(TestCostConvergenceTermination)
{
  auto space = std::make_shared<base::SO2StateSpace>();
  auto si = std::make_shared<base::SpaceInformation>(space);
  auto pdef = std::make_shared<base::ProblemDefinition>(si);
  std::vector<base::Cost> costs(10, base::Cost(10.));
  std::vector<const base::State *> dummy;

  // convergence after 5 iterations
  pdef->setOptimizationObjective(std::make_shared<base::PathLengthOptimizationObjective>(si));
  {
    base::CostConvergenceTerminationCondition ptc(pdef, 5, 1.);
    BOOST_CHECK(!ptc);
    BOOST_CHECK(!ptc());
    for (unsigned int i = 0; i < 10; ++i)
    {
      BOOST_CHECK(i<5 ? !ptc() : ptc());
      pdef->getIntermediateSolutionCallback()(nullptr, dummy, costs[i]);
    }

    BOOST_CHECK(ptc);
    BOOST_CHECK(ptc());
  }

  // convergence after 10 iterations
  costs[9] = base::Cost(9.);
  {
    base::CostConvergenceTerminationCondition ptc(pdef, 10, .1);
    BOOST_CHECK(!ptc);
    BOOST_CHECK(!ptc());
    for (const auto &c: costs)
    {
      BOOST_CHECK(!ptc());
      pdef->getIntermediateSolutionCallback()(nullptr, dummy, c);
    }

    BOOST_CHECK(ptc);
    BOOST_CHECK(ptc());
  }

  // no convergence after 10 iterations
  costs[9] = base::Cost(0.);
  {
    base::CostConvergenceTerminationCondition ptc(pdef, 10, .1);
    BOOST_CHECK(!ptc);
    BOOST_CHECK(!ptc());
    for (const auto &c: costs)
    {
      BOOST_CHECK(!ptc());
      pdef->getIntermediateSolutionCallback()(nullptr, dummy, c);
    }

    BOOST_CHECK(!ptc);
    BOOST_CHECK(!ptc());
  }

  // convergence after 10 iterations
  costs[9] = base::Cost(11.);
  pdef->setOptimizationObjective(std::make_shared<base::MaximizeMinClearanceObjective>(si));
  {
    base::CostConvergenceTerminationCondition ptc(pdef, 10, .1);
    BOOST_CHECK(!ptc);
    BOOST_CHECK(!ptc());
    for (const auto &c: costs)
    {
      BOOST_CHECK(!ptc());
      pdef->getIntermediateSolutionCallback()(nullptr, dummy, c);
    }

    BOOST_CHECK(ptc);
    BOOST_CHECK(ptc());
  }

  // no convergence after 10 iterations
  costs[9] = base::Cost(20.);
  pdef->setOptimizationObjective(std::make_shared<base::MaximizeMinClearanceObjective>(si));
  {
    base::CostConvergenceTerminationCondition ptc(pdef, 10, .1);
    BOOST_CHECK(!ptc);
    BOOST_CHECK(!ptc());
    for (const auto &c: costs)
    {
      BOOST_CHECK(!ptc());
      pdef->getIntermediateSolutionCallback()(nullptr, dummy, c);
    }

    BOOST_CHECK(!ptc);
    BOOST_CHECK(!ptc());
  }
}
