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

#include "ompl/base/samplers/deterministic/PrecomputedSequence.h"

#include "ompl/util/Console.h"

#include <iostream>
#include <cmath>
#include <map>
#include <algorithm>
#include <fstream>
#include <sstream>

namespace ompl
{
    namespace base
    {
        PrecomputedSequence::PrecomputedSequence(std::string path, unsigned int dimensions, bool shuffle,
                                                 size_t max_samples)
          : DeterministicSequence(dimensions)
        {
            readSamplesFromFile(path, dimensions, max_samples);
            if (shuffle)
            {
                rand_eng_ = std::default_random_engine{};
                shuffleSamples();
            }
        }

        std::vector<double> PrecomputedSequence::sample()
        {
            if (current_index_ >= sample_set_.size())
            {
                OMPL_WARN("Precomputed sequence sampler requested more samples than possible, will provide same "
                          "samples again.");
                current_index_ = 0;
            }
            const auto &next_sample = sample_set_[current_index_];
            ++current_index_;  // out of bounds check is done on next sample, so that if there are n samples, sampling
                               // exactly n samples gives no error
            return next_sample;
        }

        void PrecomputedSequence::readSamplesFromFile(std::string path, unsigned int dimensions, size_t max_samples)
        {
            std::ifstream ifs(path);
            std::string line;
            while (std::getline(ifs, line))
            {
                size_t count = 0;
                std::istringstream iss(line);
                std::vector<double> new_sample;
                while (count < dimensions && iss)
                {
                    double new_number;
                    iss >> new_number;
                    new_sample.emplace_back(new_number);
                    ++count;
                }
                if (count != dimensions)
                {
                    OMPL_ERROR("Precomputed sequence contains less dimensions than requested.");
                    return;
                }
                sample_set_.emplace_back(new_sample);
                if (max_samples != 0 && sample_set_.size() >= max_samples)
                    break;
            }
        }

        void PrecomputedSequence::shuffleSamples()
        {
            std::shuffle(sample_set_.begin(), sample_set_.end(), rand_eng_);
        }

    }  // namespace base
}  // namespace ompl
