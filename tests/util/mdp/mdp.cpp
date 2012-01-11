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

#include <gtest/gtest.h>
#include "ompl/util/MDP.h"
#include <vector>

using namespace ompl;

TEST(MDP, Simple1)
{
    MDP mdp;
    mdp.setStateCount(3);
    mdp.setActionCount(2);

    // moving from 0:
    mdp.addPossibleAction(0, 0, 0, 0.4, -50.0);
    mdp.addPossibleAction(0, 1, 0, 0.5, 10.0);
    mdp.addPossibleAction(0, 2, 0, 0.1, 1.0);

    //    mdp.addPossibleAction(0, 0, 1, 0.0, 1.0);
    mdp.addPossibleAction(0, 1, 1, 0.9, 1.0);
    mdp.addPossibleAction(0, 2, 1, 0.1, 1.0);

    // moving from 1:
    mdp.addPossibleAction(1, 0, 0, 0.1, -5.0);
    mdp.addPossibleAction(1, 1, 0, 0.2, -1.0);
    mdp.addPossibleAction(1, 2, 0, 0.7, 5.0);

    mdp.addPossibleAction(1, 0, 1, 0.2, -1.0);
    mdp.addPossibleAction(1, 1, 1, 0.05, -1.0);
    mdp.addPossibleAction(1, 2, 1, 0.75, 5.0);

    // moving from 2:
    mdp.addPossibleAction(2, 0, 0, 0.05, -10.0);
    mdp.addPossibleAction(2, 1, 0, 0.05, -5.0);
    mdp.addPossibleAction(2, 2, 0, 0.9, 0.1);

    mdp.addPossibleAction(2, 0, 1, 0.2, -10.0);
    mdp.addPossibleAction(2, 1, 1, 0.05, -5.0);
    mdp.addPossibleAction(2, 2, 1, 0.75, 0.1);

    EXPECT_TRUE(mdp.check());

    EXPECT_TRUE(mdp.valueIteration(1e-6, 10000));

    EXPECT_TRUE(mdp.getIterationSteps() < 10000);
    EXPECT_TRUE(mdp.getValueError() < 1e-6);

    const std::vector<double> &values = mdp.getValues();
    EXPECT_NEAR(values[0], 1.97741, 1e-3);
    EXPECT_NEAR(values[1], 1.52155, 1e-3);
    EXPECT_NEAR(values[2], -3.40551, 1e-3);

    const std::vector<int> &policy = mdp.getPolicy();
    EXPECT_EQ(policy[0], 1);
    EXPECT_EQ(policy[1], 1);
    EXPECT_EQ(policy[2], 0);

    std::vector<std::size_t> st;
    std::vector<std::size_t> ac;
    mdp.extractRewardingPath(0, 2, st, ac);
    EXPECT_EQ(st.size(), 3u);
    EXPECT_EQ(ac.size(), 2u);
}

TEST(MDP, Simple2)
{
    MDP mdp;
    mdp.setStateCount(3);
    mdp.setActionCount(2);
    mdp.setDiscountFactor(1.0);

    // moving from 0:
    mdp.addPossibleAction(0, 1, 0, 1.0, -1.0);
    mdp.addPossibleAction(0, 0, 1, 0.3, -10.0);
    mdp.addPossibleAction(0, 2, 1, 0.7, -10.0);

    // moving from 1:
    mdp.addPossibleAction(1, 0, 0, 0.9, -1.0);
    mdp.addPossibleAction(1, 2, 0, 0.1, -1.0);

    // moving from 2:
    mdp.addPossibleAction(2, 2, 0, 1.0, 0.0);

    EXPECT_TRUE(mdp.check());

    EXPECT_FALSE(mdp.valueIteration(1e-12, 5));
    const std::vector<int> &policyShort = mdp.getPolicy();
    EXPECT_EQ(policyShort.size(), mdp.getStateCount());
    EXPECT_EQ(policyShort[0], 0);
    EXPECT_EQ(policyShort[1], 0);
    EXPECT_EQ(policyShort[2], 0);

    EXPECT_TRUE(mdp.valueIteration(1e-12, 10000));
    const std::vector<int> &policyLong = mdp.getPolicy();
    EXPECT_EQ(policyLong[0], 1);
    EXPECT_EQ(policyLong[1], 0);
    EXPECT_EQ(policyLong[2], 0);

    const std::vector<double> &values = mdp.getValues();
    EXPECT_NEAR(values[0], -14.28571, 1e-3);
    EXPECT_NEAR(values[1], -13.85714, 1e-3);
    EXPECT_NEAR(values[2], 0.0,       1e-3);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
