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

#ifndef OMPL_BASE_PRECOMPUTED_SEQUENCE
#define OMPL_BASE_PRECOMPUTED_SEQUENCE

#include "ompl/base/samplers/deterministic/DeterministicSequence.h"
#include <random>
#include <string>

namespace ompl
{
    namespace base
    {
        /** \brief General realization for a sampler of precomputed sequences or sets. */
        class PrecomputedSequence : public DeterministicSequence
        {
        public:
            /** \brief Constructor, requires the path of file containing the precomputed samples
            formated with one sample per line, with the dimensions separated by whitespace. Optionally the order of
            the samples can be randomized (which removes deterministim, if not all samples are used), max_samples
            optionally limits the number of samples to be read from the file (default 0 means all samples are read),
            scale_factor optinonally scales the the samples (if the precomputed samplesets are e.g. in range 0,1. **/
            PrecomputedSequence(std::string path, unsigned int dimensions, bool shuffle = false,
                                size_t max_samples = 0);

            /** \brief Returns the next sample, loops if there are no more precomputed samples. The range
            of the samples depends on the precomputed set of samples. */
            std::vector<double> sample() override;

        private:
            std::vector<std::vector<double>> sample_set_;
            size_t current_index_{0};
            std::default_random_engine rand_eng_;

            /** \brief Reads a specified number of samples of specified dimension from a specified file. */
            void readSamplesFromFile(std::string path, unsigned int dimensions, size_t max_samples = 0);

            /** \brief Randomly shuffles the set of samples **/
            void shuffleSamples();
        };
    }  // namespace base

}  // namespace ompl

#endif
