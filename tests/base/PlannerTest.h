/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2011, Rice University
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

/* Author: Ioan Sucan */

#include "ompl/base/Planner.h"
#include <boost/test/unit_test.hpp>

namespace ompl
{

    namespace base
    {

        /** \brief Encapsulate basic tests for planners. This class
            should be used for every planner included with ompl, to
            ensure basic functionality works. */
        class PlannerTest
        {
        public:

            /** \brief Construct a testing setup for planner \e planner */
            PlannerTest(const PlannerPtr &planner) : planner_(planner)
            {
            }

            ~PlannerTest()
            {
            }

            /** \brief Test that solve() and clear() work as expected */
            void testSolveAndClear()
            {
                planner_->clear();

                bool solved = planner_->solve(timedPlannerTerminationCondition(1.0));
                BOOST_CHECK(solved);

                planner_->clear();

                solved = planner_->solve(timedPlannerTerminationCondition(1.0));
                BOOST_CHECK(solved);

                solved = planner_->solve(timedPlannerTerminationCondition(1.0));
                BOOST_CHECK(solved);

                planner_->clear();

                planner_->solve(plannerAlwaysTerminatingCondition());
                planner_->solve(0.001);
                planner_->solve(0.01);
                solved = planner_->solve(0.1);
                if (!solved)
                    solved = planner_->solve(timedPlannerTerminationCondition(1.0));
                BOOST_CHECK(solved);
                planner_->clear();
                planner_->clear();
                planner_->clear();
                planner_->getProblemDefinition()->isStraightLinePathValid();
            }

            /** \brief Call all tests for the planner */
            void test()
            {
                testSolveAndClear();
            }

        private:

            PlannerPtr planner_;

        };
    }
}
