/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
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

#include <gtest/gtest.h>
#include "ompl/util/RandomNumbers.h"
#include <cmath>
#include <vector>

using namespace ompl;

/* Just test we get some random values */
TEST(Random, DifferentSeeds)
{
    RNG r1, r2, r3, r4;
    int same = 0;
    int eq   = 0;
    const int N = 100;
    for (int i = 0 ; i < N ; ++i)
    {
        int v1 = r1.uniformInt(0, 100);
        int v2 = r2.uniformInt(0, 100);
        int v3 = r3.uniformInt(0, 100);
        int v4 = r4.uniformInt(0, 100);
        printf("%d %d %d %d\n", v1, v2, v3, v4);

        if (v1 == v2 && v2 == v3 && v3 == v4)
            eq++;
        if (v1 == r1.uniformInt(0, 100))
            same++;
        if (v2 == r2.uniformInt(0, 100))
            same++;
        if (v3 == r3.uniformInt(0, 100))
            same++;
        if (v4 == r4.uniformInt(0, 100))
            same++;
    }
    EXPECT_FALSE(eq > N / 2);
    EXPECT_TRUE(same < 2 * N);
}

TEST(Random, ValidRangeInts)
{
    RNG r;
    const int N = 100;
    const int V = 10000 * N;
    std::vector<int> c(N + 1, 0);
    for (int i = 0 ; i < V ; ++i)
    {
        int v = r.uniformInt(0, N);
        EXPECT_TRUE(v >= 0);
        EXPECT_TRUE(v <= N);
        c[v]++;
    }

    for (unsigned int i = 0 ; i < c.size() ; ++i)
        EXPECT_TRUE(c[i] > V/N/3);
}

static const double NUM_INT_SAMPLES = 1000000;
static const double NUM_REAL_SAMPLES = 1000000;
/* The following widening factor is multiplied by the standard error of the mean
 * in errUniformInt() and errUniformReal() to obtain a reasonable range to pass to EXPECT_NEAR().
 * For one million samples, a widening factor of 2.0 seems to be sufficient.
 * If the number of samples is increased, then this widening factor can be decreased. */
static const double STDERR_WIDENING_FACTOR = 2.0;

static double avgIntsN(int s, int l, const int N)
{
    RNG r;
    double sum = 0.0;
    for (int i = 0 ; i < N ; ++i)
        sum += r.uniformInt(s, l);
    return sum / (double)N;
}

static double avgInts(int s, int l)
{
    return avgIntsN(s, l, NUM_INT_SAMPLES);
}

static double errUniformInt(int s, int l)
{
    const int length = l-s+1;
    //standard error of mean for discrete uniform distribution over {s,s+1,...,l}
    const double stdErr = sqrt((length*length-1)/(12.0*NUM_INT_SAMPLES));
    return stdErr * STDERR_WIDENING_FACTOR;
}

TEST(Random, AvgInts)
{
    EXPECT_NEAR(avgInts(0, 1), 0.5, errUniformInt(0,1));
    EXPECT_NEAR(avgInts(0, 10), 5.0, errUniformInt(0,10));
    EXPECT_NEAR(avgInts(-1, 1), 0.0, errUniformInt(-1,1));
    EXPECT_NEAR(avgInts(-1, 0), -0.5, errUniformInt(-1,0));
    EXPECT_NEAR(avgInts(-2, 4), 1.0, errUniformInt(-2,4));
    EXPECT_NEAR(avgInts(2, 4), 3.0, errUniformInt(2,4));
    EXPECT_NEAR(avgInts(-6, -2), -4.0, errUniformInt(-6,-2));
    EXPECT_NEAR(avgIntsN(0, 0, 1000), 0.0, errUniformInt(0,0));
}

static double avgRealsN(double s, double l, const int N)
{
    RNG r;
    double sum = 0.0;
    for (int i = 0 ; i < N ; ++i)
        sum += r.uniformReal(s, l);
    return sum / (double)N;
}

static double avgReals(double s, double l)
{
    return avgRealsN(s, l, NUM_REAL_SAMPLES);
}

static double errUniformReal(double s, double l)
{
    //standard error of mean for continuous uniform distribution over real interval [s,l].
    const double stdErr = (l-s)*sqrt(1.0/(12.0*NUM_REAL_SAMPLES));
    return stdErr * STDERR_WIDENING_FACTOR;
}

TEST(Random, AvgReals)
{
    EXPECT_NEAR(avgReals(-0.1, 0.3), 0.1, errUniformReal(-0.1,0.3));
    EXPECT_NEAR(avgReals(0, 1), 0.5, errUniformReal(0,1));
    EXPECT_NEAR(avgReals(0, 10), 5.0, errUniformReal(0,10));
    EXPECT_NEAR(avgReals(-1, 1), 0.0, errUniformReal(-1,1));
    EXPECT_NEAR(avgReals(-1, 0), -0.5, errUniformReal(-1,0));
    EXPECT_NEAR(avgReals(-2, 4), 1.0, errUniformReal(-2,4));
    EXPECT_NEAR(avgReals(2, 4), 3.0, errUniformReal(2,4));
    EXPECT_NEAR(avgReals(-6, -2), -4.0, errUniformReal(-6,-2));
    EXPECT_NEAR(avgRealsN(0, 0, 1000), 0.0, errUniformReal(0,0));
}

static double avgNormalRealsN(double mean, double stddev, const int N)
{
    RNG r;
    double sum = 0.0;
    for (int i = 0 ; i < N ; ++i)
        sum += r.gaussian(mean, stddev);
    return sum / (double)N;
}

static double avgNormalReals(double m, double s)
{
    return avgNormalRealsN(m, s, NUM_REAL_SAMPLES);
}

static double errNormal(double stddev)
{
    //standard error of mean for gaussian with given stddev
    return stddev / sqrt(NUM_REAL_SAMPLES);
}

TEST(Random, NormalReals)
{
    EXPECT_NEAR(avgNormalReals(10.0, 1.0), 10.0, errNormal(1.0));
}

int main(int argc, char **argv)
{
    ompl::RNG::setSeed(1);
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
