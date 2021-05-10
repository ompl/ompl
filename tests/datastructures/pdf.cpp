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

/* Author: Matt Maly */

#define BOOST_TEST_MODULE "PDF"
#include <boost/test/unit_test.hpp>
#include "ompl/datastructures/PDF.h"
#include "ompl/util/RandomNumbers.h"

#include <cmath>
#include <vector>

// define a convenience macro
#define BOOST_OMPL_EXPECT_NEAR(a, b, diff) BOOST_CHECK_SMALL((a) - (b), diff)

BOOST_AUTO_TEST_CASE(Simple)
{
    using Element = ompl::PDF<int>::Element;
    ompl::PDF<int> p;
    BOOST_CHECK(p.empty());
    p.add(0, 1.0);
    BOOST_CHECK_EQUAL(0, p.sample(0.5));
    BOOST_CHECK_EQUAL(1u, p.size());
    BOOST_CHECK_EQUAL(p.empty(), false);
    p.add(1, 0.0);
    p.add(2, 0.0);
    p.add(3, 0.0);
    BOOST_CHECK_EQUAL(0, p.sample(0.5));
    BOOST_CHECK_EQUAL(4u, p.size());

    p.clear();
    Element* e25 = p.add(0, 25);
    // 25
    BOOST_CHECK_EQUAL(0, p.sample(1.0));

    Element* e50 = p.add(1, 50);
    // 25 50
    BOOST_CHECK_EQUAL(0, p.sample(0.3));
    BOOST_CHECK_EQUAL(1, p.sample(0.5));

    Element* e15 = p.add(2, 15);
    // 25 50 15
    BOOST_CHECK_EQUAL(0, p.sample(0.25));
    BOOST_CHECK_EQUAL(1, p.sample(0.5));
    BOOST_CHECK_EQUAL(2, p.sample(0.85));

    Element* e10 = p.add(3, 10);
    // 25 50 15 10
    BOOST_CHECK_EQUAL(0, p.sample(0.1));
    BOOST_CHECK_EQUAL(1, p.sample(0.7));
    BOOST_CHECK_EQUAL(2, p.sample(0.8));
    BOOST_CHECK_EQUAL(3, p.sample(0.95));

    Element* e6 = p.add(4, 6);
    Element* e30 = p.add(5, 30);
    Element* e1 = p.add(6, 1);
    // 25 50 15 10 6 30 1

    p.remove(e1);
    // 25 50 15 10 6 30
    BOOST_CHECK_EQUAL(5, p.sample(0.80));
    BOOST_CHECK_EQUAL(0, p.sample(0.05));

    p.update(e30, 4);
    // 25 50 15 10 6 4
    BOOST_CHECK_EQUAL(5, p.sample(0.98));
    BOOST_CHECK_EQUAL(4, p.sample(0.93));

    p.remove(e6);
    // 25 50 15 10 4
    BOOST_CHECK_EQUAL(5u, p.size());
    BOOST_CHECK_EQUAL(3, p.sample(0.96));
    BOOST_CHECK_EQUAL(1, p.sample(0.3));

    p.remove(e25);
    // 4 50 15 10
    BOOST_CHECK_EQUAL(5, p.sample(0.03));
    BOOST_CHECK_EQUAL(1, p.sample(0.4));
    BOOST_CHECK_EQUAL(2, p.sample(0.85));
    BOOST_CHECK_EQUAL(3, p.sample(0.95));

    p.remove(e50);
    // 4 10 15
    BOOST_CHECK_EQUAL(2, p.sample(0.75));

    p.update(e10, 51);
    // 4 51 15
    BOOST_CHECK_EQUAL(3, p.sample(0.5));

    p.remove(e10);
    p.remove(e30);
    // 15
    BOOST_CHECK_EQUAL(2, p.sample(1.0));

    p.remove(e15);
    BOOST_CHECK_EQUAL(0u, p.size());
    BOOST_CHECK(p.empty());

    p.clear();
}

BOOST_AUTO_TEST_CASE(Statistical)
{
    const std::size_t NUM_SAMPLES = 5000000;
    /* The following widening factor is multiplied by the standard error of the mean
     * to obtain a reasonable range to pass to BOOST_OMPL_EXPECT_NEAR(). For five million samples,
     * a widening factor of 2.5 seems sufficient. If the number of samples is increased,
     * then this widening factor can be decreased. */
    const double STDERR_WIDENING_FACTOR = 2.5;

    std::vector<std::pair<int,double> > values;
    values.emplace_back(0, 30.0);
    values.emplace_back(1, 10.0);
    values.emplace_back(2, 25.0);
    values.emplace_back(3, 15.0);
    values.emplace_back(4, 20.0);

    ompl::PDF<int> p;
    double mean = 0.0;
    double sumWeights = 0.0;
    /* Calculate weighted mean of discrete uniform distribution as we add elements to PDF. */
    for (const auto & elem : values)
    {
        p.add(elem.first, elem.second);
        mean += elem.first*elem.second;
        sumWeights += elem.second;
    }
    mean /= sumWeights;

    /* Calculate weighted variance of discrete uniform distribution, which is defined as
     * sum(w[i]*(x[i]-mean)^2)/sumWeights. */
    double variance = 0.0;
    for (const auto & elem : values)
    {
        variance += elem.second * (elem.first - mean) * (elem.first - mean);
    }
    variance /= sumWeights;
    double standerr = sqrt(variance/NUM_SAMPLES);

    double sampleMean = 0.0;
    ompl::RNG rand;
    for (std::size_t i = 0; i < NUM_SAMPLES; ++i)
        sampleMean += p.sample(rand.uniform01());
    sampleMean /= NUM_SAMPLES;

    BOOST_OMPL_EXPECT_NEAR(sampleMean, mean, STDERR_WIDENING_FACTOR*standerr);
}
