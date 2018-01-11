/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2012, Willow Garage
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
*   * Neither the name of the Willow Garage nor the names of its
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

#define BOOST_TEST_MODULE "StateStorage"
#include <boost/test/unit_test.hpp>
#include "ompl/base/StateStorage.h"
#include "ompl/base/ScopedState.h"
#include "ompl/base/spaces/SE3StateSpace.h"
#include "ompl/base/spaces/SE2StateSpace.h"

using namespace ompl;

// define a convenience macro
#define BOOST_OMPL_EXPECT_NEAR(a, b, diff) BOOST_CHECK_SMALL((a) - (b), diff)

struct Metadata
{
    Metadata() = default;

    int   tag1{0};
    float tag2{0.5f};

    template<typename Archive>
    void serialize(Archive & ar, const unsigned int /*version*/)
    {
        ar & tag1;
        ar & tag2;
    }
};


BOOST_AUTO_TEST_CASE(Store)
{
    auto space(std::make_shared<base::SE3StateSpace>());
    base::RealVectorBounds bounds(3);
    bounds.setLow(-1);
    bounds.setHigh(1);
    space->setBounds(bounds);
    space->setup();

    base::StateStorage ss(space);
    base::ScopedState<> s(space);
    base::State *x = space->allocState();
    for (int i = 0 ; i < 1000 ; ++i)
    {
        s.random();
        space->copyState(x, s.get());
        ss.addState(x);
    }
    space->freeState(x);
    ss.store("tmp_states");

    base::StateStorageWithMetadata<Metadata> ssm(space);
    ssm.generateSamples(3);
    ssm.getMetadata(0).tag1 = 2;
    ssm.getMetadata(1).tag2 = 1.0f;
    ssm.store("tmp_states_wm");
}

BOOST_AUTO_TEST_CASE(Load)
{
    auto space(std::make_shared<base::SE3StateSpace>());
    base::RealVectorBounds bounds(3);
    bounds.setLow(-1);
    bounds.setHigh(1);
    space->setBounds(bounds);
    space->setup();

    base::StateStorage ss(space);
    ss.load("tmp_states");

    base::StateStorageWithMetadata<Metadata> ssm(space);
    ssm.load("tmp_states_wm");
    BOOST_CHECK_EQUAL(ssm.getMetadata(0).tag1, 2);
    BOOST_OMPL_EXPECT_NEAR(ssm.getMetadata(1).tag2, 1.0, 1e-5);
}
