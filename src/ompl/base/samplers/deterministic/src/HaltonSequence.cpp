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

/* Author: Leonard Bruns */

#include "ompl/base/samplers/deterministic/HaltonSequence.h"
#include "ompl/util/Console.h"
#include <iostream>
#include <cmath>
#include <map>
#include <boost/math/special_functions/prime.hpp>

namespace ompl
{
    namespace base
    {
        HaltonSequence1D::HaltonSequence1D() : i_(1), base_(2)
        {
        }

        HaltonSequence1D::HaltonSequence1D(unsigned int base) : i_(1), base_(base)
        {
        }

        void HaltonSequence1D::setBase(unsigned int base)
        {
            base_ = base;
        }

        double HaltonSequence1D::sample()
        {
            double f = 1, r = 0;
            unsigned int i = i_;

            while (i > 0)
            {
                f /= base_;
                r += f * (i % base_);
                i = std::floor(i / base_);
            }

            ++i_;
            return r;
        }

        HaltonSequence::HaltonSequence(unsigned int dimensions)
          : DeterministicSequence(dimensions), halton_sequences_1d_(dimensions)
        {
            setBasesToPrimes();
        }

        HaltonSequence::HaltonSequence(unsigned int dimensions, std::vector<unsigned int> bases)
          : DeterministicSequence(dimensions), halton_sequences_1d_(dimensions)
        {
            if (bases.size() != dimensions)
            {
                OMPL_WARN("Number of bases does not match dimensions. Using first n primes instead.");
            }
            else
            {
                int i = 0;
                for (auto base : bases)
                {
                    halton_sequences_1d_[i].setBase(base);
                    i++;
                }
            }
        }

        std::vector<double> HaltonSequence::sample()
        {
            std::vector<double> samples;
            for (auto &seq : halton_sequences_1d_)
            {
                samples.push_back(seq.sample());
            }
            return samples;
        }

        void HaltonSequence::setBasesToPrimes()
        {
            // set the base of the halton sequences to the first n prime numbers, where n is dimensions
            unsigned int current = 2;
            for (unsigned int i = 0; i < dimensions_; i++)
            {
                current = boost::math::prime(i);
                halton_sequences_1d_[i].setBase(current);
            }
        }
    }  // namespace base
}  // namespace ompl
