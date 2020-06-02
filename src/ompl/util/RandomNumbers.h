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

/* Author: Ioan Sucan, Jonathan Gammell */

#ifndef OMPL_UTIL_RANDOM_NUMBERS_
#define OMPL_UTIL_RANDOM_NUMBERS_

#include <memory>
#include <random>
#include <cassert>
#include <cstdint>
#include <algorithm>

#include "ompl/config.h"
#include "ompl/util/ProlateHyperspheroid.h"

namespace ompl
{
    /** \brief Random number generation. An instance of this class
        cannot be used by multiple threads at once (member functions
        are not const). However, the constructor is thread safe and
        different instances can be used safely in any number of
        threads. It is also guaranteed that all created instances will
        have a different random seed. */
    class RNG
    {
    public:
        /** \brief Constructor. Always sets a different random seed */
        RNG();

        /** \brief Constructor. Set to the specified instance seed. */
        RNG(std::uint_fast32_t localSeed);

        /** \brief Generate a random real between 0 and 1 */
        double uniform01()
        {
            return uniDist_(generator_);
        }

        /** \brief Generate a random real within given bounds: [\e lower_bound, \e upper_bound) */
        double uniformReal(double lower_bound, double upper_bound)
        {
            assert(lower_bound <= upper_bound);
            return (upper_bound - lower_bound) * uniDist_(generator_) + lower_bound;
        }

        /** \brief Generate a random integer within given bounds: [\e lower_bound, \e upper_bound] */
        int uniformInt(int lower_bound, int upper_bound)
        {
            auto r = (int)floor(uniformReal((double)lower_bound, (double)(upper_bound) + 1.0));
            return (r > upper_bound) ? upper_bound : r;
        }

        /** \brief Generate a random boolean */
        bool uniformBool()
        {
            return uniDist_(generator_) <= 0.5;
        }

        /** \brief Generate a random real using a normal distribution with mean 0 and variance 1 */
        double gaussian01()
        {
            return normalDist_(generator_);
        }

        /** \brief Generate a random real using a normal distribution with given mean and variance */
        double gaussian(double mean, double stddev)
        {
            return normalDist_(generator_) * stddev + mean;
        }

        /** \brief Generate a random real using a half-normal distribution. The value is within specified bounds [\e
            r_min, \e r_max], but with a bias towards \e r_max. The function is implemended using a Gaussian
           distribution with
            mean at \e r_max - \e r_min. The distribution is 'folded' around \e r_max axis towards \e r_min.
            The variance of the distribution is (\e r_max - \e r_min) / \e focus. The higher the focus,
            the more probable it is that generated numbers are close to \e r_max. */
        double halfNormalReal(double r_min, double r_max, double focus = 3.0);

        /** \brief Generate a random integer using a half-normal
            distribution. The value is within specified bounds ([\e r_min, \e r_max]), but
            with a bias towards \e r_max. The function is implemented on top of halfNormalReal() */
        int halfNormalInt(int r_min, int r_max, double focus = 3.0);

        /** \brief Uniform random unit quaternion sampling. The computed value has the order (x,y,z,w). The return
         * variable \e value is expected to already exist. */
        void quaternion(double value[4]);

        /** \brief Uniform random sampling of Euler roll-pitch-yaw angles, each in the range (-pi, pi]. The computed
         * value has the order (roll, pitch, yaw).  The return variable \e value is expected to already exist. */
        void eulerRPY(double value[3]);

        /** \brief Set the seed used to generate the seeds of each RNG instance. Use this
            function to ensure the same sequence of random numbers is generated across multiple instances of RNG. */
        static void setSeed(std::uint_fast32_t seed);

        /** \brief Get the seed used to generate the seeds of each RNG instance.
            Passing the returned value to setSeed() at a subsequent execution of the code will ensure deterministic
            (repeatable) behaviour across multiple instances of RNG. Useful for debugging. */
        static std::uint_fast32_t getSeed();

        /** \brief Set the seed used for the instance of a RNG. Use this function to ensure that an instance of
            an RNG generates the same deterministic sequence of numbers. This function resets the member generators*/
        void setLocalSeed(std::uint_fast32_t localSeed);

        /** \brief Get the seed used for the instance of a RNG. Passing the returned value to the setInstanceSeed()
            of another RNG will assure that the two objects generate the same sequence of numbers.
            Useful for comparing different settings of a planner while maintaining the same stochastic behaviour,
            assuming that every "random" decision made by the planner is made from the same RNG. */
        std::uint_fast32_t getLocalSeed() const
        {
            return localSeed_;
        }

        /** \brief Uniform random sampling of a unit-length vector. I.e., the surface of an n-ball. The return variable
         * \e value is expected to already exist. */
        void uniformNormalVector(std::vector<double> &v);

        /** \brief Uniform random sampling of the content of an n-ball, with a radius appropriately distributed between
         * [0,r) so that the distribution is uniform in a Cartesian coordinate system. The return variable \e value is
         * expected to already exist. */
        void uniformInBall(double r, std::vector<double> &v);

        /** \brief Uniform random sampling of the surface of a prolate hyperspheroid, a special symmetric type of
        n-dimensional ellipse. The return variable \e value is expected to already exist.
        @par J. D. Gammell, T. D. Barfoot, S. S. Srinivasa, "Informed sampling for asymptotically optimal path planning."
        IEEE Transactions on Robotics (T-RO), 34(4): 966-984, Aug. 2018.
        DOI: <a href="https://doi.org/10.1109/TRO.2018.2830331">TRO.2018.2830331</a>.
        arXiv: <a href="https://arxiv.org/pdf/1706.06454">1706.06454 [cs.RO]</a>
        <a href="https://www.youtube.com/watch?v=d7dX5MvDYTc">Illustration video</a>.
        <a href="https://www.youtube.com/watch?v=nsl-5MZfwu4">Short description video</a>. */
        void uniformProlateHyperspheroidSurface(const std::shared_ptr<const ProlateHyperspheroid> &phsPtr,
                                                double value[]);

        /** \brief Uniform random sampling of a prolate hyperspheroid, a special symmetric type of
        n-dimensional ellipse. The return variable \e value is expected to already exist.
        @par J. D. Gammell, T. D. Barfoot, S. S. Srinivasa, "Informed sampling for asymptotically optimal path planning."
        IEEE Transactions on Robotics (T-RO), 34(4): 966-984, Aug. 2018.
        DOI: <a href="https://doi.org/10.1109/TRO.2018.2830331">TRO.2018.2830331</a>.
        arXiv: <a href="https://arxiv.org/pdf/1706.06454">1706.06454 [cs.RO]</a>.
        <a href="https://www.youtube.com/watch?v=d7dX5MvDYTc">Illustration video</a>.
        <a href="https://www.youtube.com/watch?v=nsl-5MZfwu4">Short description video</a>. */
        void uniformProlateHyperspheroid(const std::shared_ptr<const ProlateHyperspheroid> &phsPtr, double value[]);

        /** \brief randomly rearrange elements in the range [first, last) */
        template <class RandomAccessIterator>
        void shuffle(RandomAccessIterator first, RandomAccessIterator last)
        {
            std::shuffle(first, last, generator_);
        }

    private:
        /** \brief A forward declaration to a data structure class holding data for spherical distributions of various
         * dimension. */
        class SphericalData;

        /** \brief The seed used for the instance of a RNG */
        std::uint_fast32_t localSeed_;
        std::mt19937 generator_;
        std::uniform_real_distribution<> uniDist_{0, 1};
        std::normal_distribution<> normalDist_{0, 1};
        // A structure holding boost::uniform_on_sphere distributions and the associated boost::variate_generators for
        // various dimension
        std::shared_ptr<SphericalData> sphericalDataPtr_;
    };
}  // namespace ompl

#endif
