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

#ifndef OMPL_BASE_HALTON_SEQUENCE
#define OMPL_BASE_HALTON_SEQUENCE

#include "ompl/base/samplers/deterministic/DeterministicSequence.h"

namespace ompl
{
    namespace base
    {
        /**
        @anchor HaltonSequence1D
        @par Short description
        \ref HaltonSequence1D Realization of the Halton sequence for the generation of
        arbitrary dimensional, low-dispersion sequences.
        @par External documentation
        Implementation follows https://en.wikipedia.org/wiki/Halton_sequence.
        A more efficient implementation might be found in
        Struckmeier, Jens. "Fast generation of low-discrepancy sequences." Journal of Computational and
        Applied Mathematics 61.1 (1995): 29-41.
        \brief Realization of the Halton sequence for the generation of
        arbitrary dimensional, low-dispersion sequences.
        */

        class HaltonSequence1D
        {
        public:
            /** \brief Constructor */
            HaltonSequence1D();

            /** \brief Constructor */
            HaltonSequence1D(unsigned int base);

            /** \brief Sets the base of the halton sequence */
            void setBase(unsigned int base);

            /** \brief Returns the next sample in the interval [0,1] */
            double sample();

        private:
            unsigned int i_, base_;
        };

        /** \brief Realization of the Halton sequence for the generation of
        arbitrary dimensional, low-dispersion sequences. */
        class HaltonSequence : public DeterministicSequence
        {
        public:
            /** \brief Constructor, only specifiying the dimensions, first n primes will be used
            as bases. */
            HaltonSequence(unsigned int dimensions);
            /** \brief Constructor, for which the bases vector will be used to initialize the
            bases of the 1D halton sequences. bases.size() has to be equal to dimensions. */
            HaltonSequence(unsigned int dimensions, std::vector<unsigned int> bases);

            /** \brief Returns the next sample in the interval [0,1] */
            std::vector<double> sample() override;

        private:
            std::vector<HaltonSequence1D> halton_sequences_1d_;

            /** \brief Sets the bases of the 1D Halton generators to the first n primes */
            void setBasesToPrimes();
        };
    }  // namespace base

}  // namespace ompl

#endif
