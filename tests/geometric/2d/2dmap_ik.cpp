/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, Rice University
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

#define BOOST_TEST_MODULE "GeometricPlanningIK"
#include <boost/test/unit_test.hpp>
#include "ompl/util/DisableCompilerWarning.h"
OMPL_PUSH_DISABLE_CLANG_WARNING(-Wunused-function)
OMPL_PUSH_DISABLE_GCC_WARNING(-Wunused-function)
#include "2DmapSetup.h"
OMPL_POP_CLANG
#include <iostream>

#include "ompl/geometric/GeneticSearch.h"
#include "ompl/util/Time.h"

using namespace ompl;

BOOST_AUTO_TEST_CASE(SimpleIK)
{
    msg::setLogLevel(msg::LOG_ERROR);

    /* load environment */
    Environment2D env;
    boost::filesystem::path path(TEST_RESOURCES_DIR);
    path = path / "env1.txt";
    loadEnvironment(path.string().c_str(), env);

    if (env.width * env.height == 0)
    {
        BOOST_FAIL( "The environment has a 0 dimension. Cannot continue" );
    }

    /* instantiate space information */
    base::SpaceInformationPtr si = geometric::spaceInformation2DMap(env);

    /* set the goal state; the memory for this is automatically cleaned by SpaceInformation */
    base::GoalState goal(si);
    base::ScopedState<base::RealVectorStateSpace> gstate(si);
    gstate->values[0] = env.goal.first;
    gstate->values[1] = env.goal.second;
    goal.setState(gstate);
    goal.setThreshold(0.1);

    geometric::GeneticSearch gaik(si);
    gaik.setRange(5.0);
    base::ScopedState<base::RealVectorStateSpace> found(si);
    double time = 0.0;

    const int N = 100;
    for (int i = 0 ; i < N ; ++i)
    {
        ompl::time::point startTime = ompl::time::now();
        bool solved = gaik.solve(1.0, goal, found.get());
        ompl::time::duration elapsed = ompl::time::now() - startTime;
        time += ompl::time::seconds(elapsed);
        BOOST_CHECK(solved);
        BOOST_CHECK(si->distance(found.get(), gstate.get()) < goal.getThreshold());
    }
    time = time / (double)N;
    BOOST_CHECK(time < 0.01);
}
