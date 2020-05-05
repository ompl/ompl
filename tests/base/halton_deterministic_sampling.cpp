/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2019, Robert Bosch GmbH
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
*   * Neither the name of the Robert Bosch GmbH nor the names of its
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

/* Author: Luigi Palmieri */
#define BOOST_TEST_MODULE "HaltonDeterministicSampling"
#include <boost/test/unit_test.hpp>
#include <boost/filesystem.hpp>

#include <ompl/config.h>
#include <ompl/base/samplers/deterministic/HaltonSequence.h>
#include "../resources/haltonXD.h"

#include <iostream>

namespace ob = ompl::base;

BOOST_AUTO_TEST_CASE(Halton_1D)
{
    boost::filesystem::path path(TEST_RESOURCES_DIR);
    // Reading sequence from file
    HaltonXD hd1 = HaltonXD(1);
    hd1.loadSequence((path / "halton/halton_1d.txt").string().c_str());
    std::vector<std::vector<double>> seq = hd1.getSequence();
    // Defining Halton sequence from ompl::base
    ob::HaltonSequence hs1d(1);
    // checking if we have read all rows
    BOOST_CHECK_EQUAL(seq.size(), 5);
    // checking the samples of the rows
    for (unsigned int i = 0; i < 5; ++i)
    {
        std::vector<double> sample = hs1d.sample();
        std::vector<double> read_sample = seq[i];
        BOOST_CHECK_EQUAL(sample.size(), read_sample.size());

        for (unsigned int j = 0; j < sample.size(); j++)
        {
            BOOST_CHECK_CLOSE(sample[j], read_sample[j], 0.001);
        }
    }
}

BOOST_AUTO_TEST_CASE(Halton_2D)
{
    boost::filesystem::path path(TEST_RESOURCES_DIR);
    // Reading sequence from file
    HaltonXD hd2 = HaltonXD(2);
    hd2.loadSequence((path / "halton/halton_2d.txt").string().c_str());
    std::vector<std::vector<double>> seq = hd2.getSequence();
    // Defining Halton sequence from ompl::base
    ob::HaltonSequence hs2d(2);
    // checking if we have read all rows
    BOOST_CHECK_EQUAL(seq.size(), 5);
    // checking the samples of the rows
    for (unsigned int i = 0; i < 5; ++i)
    {
        std::vector<double> sample = hs2d.sample();
        std::vector<double> read_sample = seq[i];
        BOOST_CHECK_EQUAL(sample.size(), read_sample.size());

        for (unsigned int j = 0; j < sample.size(); j++)
        {
            BOOST_CHECK_CLOSE(sample[j], read_sample[j], 0.001);
        }
    }
}

BOOST_AUTO_TEST_CASE(Halton_5D)
{
    boost::filesystem::path path(TEST_RESOURCES_DIR);
    // Reading sequence from file
    HaltonXD hd5 = HaltonXD(5);
    hd5.loadSequence((path / "halton/halton_5d.txt").string().c_str());
    std::vector<std::vector<double>> seq = hd5.getSequence();
    // Defining Halton sequence from ompl::base
    ob::HaltonSequence hs5d(5);
    // checking if we have read all rows
    BOOST_CHECK_EQUAL(seq.size(), 5);
    // checking the samples of the rows
    for (unsigned int i = 0; i < 5; ++i)
    {
        std::vector<double> sample = hs5d.sample();
        std::vector<double> read_sample = seq[i];
        BOOST_CHECK_EQUAL(sample.size(), read_sample.size());

        for (unsigned int j = 0; j < sample.size(); j++)
        {
            BOOST_CHECK_CLOSE(sample[j], read_sample[j], 0.001);
        }
    }
}

BOOST_AUTO_TEST_CASE(Halton_10D)
{
    boost::filesystem::path path(TEST_RESOURCES_DIR);
    // Reading sequence from file
    HaltonXD hd10 = HaltonXD(10);
    hd10.loadSequence((path / "halton/halton_10d.txt").string().c_str());
    std::vector<std::vector<double>> seq = hd10.getSequence();
    // Defining Halton sequence from ompl::base
    ob::HaltonSequence hs10d(10);
    // checking if we have read all rows
    BOOST_CHECK_EQUAL(seq.size(), 5);
    // checking the samples of the rows
    for (unsigned int i = 0; i < 5; ++i)
    {
        std::vector<double> sample = hs10d.sample();
        std::vector<double> read_sample = seq[i];
        BOOST_CHECK_EQUAL(sample.size(), read_sample.size());

        for (unsigned int j = 0; j < sample.size(); j++)
        {
            BOOST_CHECK_CLOSE(sample[j], read_sample[j], 0.001);
        }
    }
}
