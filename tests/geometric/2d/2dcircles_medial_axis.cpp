/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2018, Rice University
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

/* Authors: Bryce Willey */

#define BOOST_TEST_MODULE "GeometricPlanningMedialAxisSampling"
#include <boost/test/unit_test.hpp>

#include "2DcirclesSetup.h"
#include <iostream>

#include "ompl/base/samplers/GradientMedialAxisStateSampler.h"

namespace ompl
{
class TestPlanner
{
public:
    TestPlanner()
    {

    }
};
}

BOOST_AUTO_TEST_CASE(geometric_medialAxis)
{
    Circles2D circles;
    boost::filesystem::path path(TEST_RESOURCES_DIR);
    circles.loadCircles((path / "circle_obstacles.txt").string());
    circles.loadQueries((path / "circles_queries.txt").string());

    //std::cout << "Max dist: " << circles.getMaxDistance(0.02) << std::endl;

    // Get a statespace /sampler.

    ompl::base::SpaceInformationPtr si = ompl::geometric::spaceInformation2DCircles(circles);
    ompl::base::ValidStateSamplerAllocator alloc = [](const ompl::base::SpaceInformation *space_info)
    {
        return std::make_shared<ompl::base::GradientMedialAxisStateSampler>(space_info);
    };
    si->setValidStateSamplerAllocator(alloc);
    ompl::base::ValidStateSamplerPtr gradientSampler = si->allocValidStateSampler();

    // Do like 20 different samples, see what happens.
    ompl::base::State *state = si->allocState();
    int total = 200;
    int worked = 0;
    for (int i = 0; i < total; i++)
    {
        std::cerr << i << std::endl;
        bool success = gradientSampler->sample(state);
        worked = (success) ? worked + 1 : worked;
    }
 
    std::cout << "Gradient sampling worked for " << worked << " of " << total << 
                 " times, or " << (1.0 * worked) / (1.0 * total) * 100 << "%" << std::endl;
}