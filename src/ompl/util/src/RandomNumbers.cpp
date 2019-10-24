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

/* Author: Ioan Sucan, Jonathan Gammell*/

#include "ompl/util/RandomNumbers.h"
#include "ompl/util/Exception.h"
#include "ompl/util/Console.h"
#include <mutex>
#include <memory>
#include <boost/math/constants/constants.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/random/uniform_on_sphere.hpp>
#include <boost/random/variate_generator.hpp>

/// @cond IGNORE
namespace
{
    /// We use a different random number generator for the seeds of the
    /// other random generators. The root seed is from the number of
    /// nano-seconds in the current time, or given by the user.
    class RNGSeedGenerator
    {
    public:
        RNGSeedGenerator()
          : firstSeed_(std::chrono::duration_cast<std::chrono::microseconds>(
                           std::chrono::system_clock::now() - std::chrono::system_clock::time_point::min())
                           .count())
          , sGen_(firstSeed_)
          , sDist_(1, 1000000000)
        {
        }

        std::uint_fast32_t firstSeed()
        {
            std::lock_guard<std::mutex> slock(rngMutex_);
            return firstSeed_;
        }

        void setSeed(std::uint_fast32_t seed)
        {
            std::lock_guard<std::mutex> slock(rngMutex_);
            if (seed > 0)
            {
                if (someSeedsGenerated_)
                {
                    OMPL_ERROR("Random number generation already started. Changing seed now will not lead to "
                               "deterministic sampling.");
                }
                else
                {
                    // In this case, since no seeds have been generated yet, so we remember this seed as the first one.
                    firstSeed_ = seed;
                }
            }
            else
            {
                if (someSeedsGenerated_)
                {
                    OMPL_WARN("Random generator seed cannot be 0. Ignoring seed.");
                    return;
                }
                OMPL_WARN("Random generator seed cannot be 0. Using 1 instead.");
                seed = 1;
            }
            sGen_.seed(seed);
        }

        std::uint_fast32_t nextSeed()
        {
            std::lock_guard<std::mutex> slock(rngMutex_);
            someSeedsGenerated_ = true;
            return sDist_(sGen_);
        }

    private:
        bool someSeedsGenerated_{false};
        std::uint_fast32_t firstSeed_;
        std::mutex rngMutex_;
        std::ranlux24_base sGen_;
        std::uniform_int_distribution<> sDist_;
    };

    std::once_flag g_once;
    boost::scoped_ptr<RNGSeedGenerator> g_RNGSeedGenerator;

    void initRNGSeedGenerator()
    {
        g_RNGSeedGenerator.reset(new RNGSeedGenerator());
    }

    RNGSeedGenerator &getRNGSeedGenerator()
    {
        std::call_once(g_once, &initRNGSeedGenerator);
        return *g_RNGSeedGenerator;
    }
}  // namespace
/// @endcond

/// @cond IGNORE
class ompl::RNG::SphericalData
{
public:
    /** \brief The container type for the variate generators. */
    using container_type_t = std::vector<double>;

    /** \brief The uniform_on_sphere distribution type. */
    using spherical_dist_t = boost::uniform_on_sphere<double, container_type_t>;

    /** \brief The resulting variate generator type. */
    using variate_generator_t = boost::variate_generator<std::mt19937 *, spherical_dist_t>;

    /** \brief Constructor */
    SphericalData(std::mt19937 *generatorPtr) : generatorPtr_(generatorPtr){};

    /** \brief The generator for a specified dimension. Will create if not existent */
    container_type_t generate(unsigned int dim)
    {
        // Assure that the dimension is in the range of the vector.
        growVector(dim);

        // Assure that the dimension is allocated:
        allocateDimension(dim);

        // Return the generator
        return (*dimVector_.at(dim).second)();
    };

    /** \brief Iterate over all the dimensions and reset the generators that exist. */
    void reset()
    {
        // Iterate over each dimension
        for (auto &i : dimVector_)
            // Check if the variate_generator is allocated
            if (bool(i.first))
                // It is, reset THE DATA (not the pointer)
                i.first->reset();
        // No else, this is an uninitialized dimension.
    };

private:
    /** \brief The pair of distribution and variate generator. */
    using dist_gen_pair_t = std::pair<std::shared_ptr<spherical_dist_t>, std::shared_ptr<variate_generator_t>>;

    /** \brief A vector distribution and variate generators (as pointers) indexed on dimension. */
    std::vector<dist_gen_pair_t> dimVector_;

    /** \brief A pointer to the generator owned by the outer class. Needed for creating new variate_generators */
    std::mt19937 *generatorPtr_;

    /** \brief Grow the vector until it contains an (empty) entry for the specified dimension. */
    void growVector(unsigned int dim)
    {
        // Iterate until the index associated with this dimension is in the vector
        while (dim >= dimVector_.size())
            // Create a pair of empty pointers:
            dimVector_.emplace_back();
    };

    /** \brief Assure that a distribution/generator is allocated for the specified index. */
    void allocateDimension(unsigned int dim)
    {
        // Only do this if unallocated, so check that:
        if (dimVector_.at(dim).first == nullptr)
        {
            // It is not allocated, so....
            // First construct the distribution
            dimVector_.at(dim).first = std::make_shared<spherical_dist_t>(dim);
            // Then the variate generator
            dimVector_.at(dim).second = std::make_shared<variate_generator_t>(generatorPtr_, *dimVector_.at(dim).first);
        }
        // No else, the pointer is already allocated.
    };
};
/// @endcond

std::uint_fast32_t ompl::RNG::getSeed()
{
    return getRNGSeedGenerator().firstSeed();
}

void ompl::RNG::setSeed(std::uint_fast32_t seed)
{
    getRNGSeedGenerator().setSeed(seed);
}

ompl::RNG::RNG()
  : localSeed_(getRNGSeedGenerator().nextSeed())
  , generator_(localSeed_)
  , sphericalDataPtr_(std::make_shared<SphericalData>(&generator_))
{
}

ompl::RNG::RNG(std::uint_fast32_t localSeed)
  : localSeed_(localSeed), generator_(localSeed_), sphericalDataPtr_(std::make_shared<SphericalData>(&generator_))
{
}

void ompl::RNG::setLocalSeed(std::uint_fast32_t localSeed)
{
    // Store the seed
    localSeed_ = localSeed;

    // Change the generator's seed
    generator_.seed(localSeed_);

    // Reset the distributions used by the variate generators, as they can cache values
    uniDist_.reset();
    normalDist_.reset();
    sphericalDataPtr_->reset();
}

double ompl::RNG::halfNormalReal(double r_min, double r_max, double focus)
{
    assert(r_min <= r_max);

    const double mean = r_max - r_min;
    double v = gaussian(mean, mean / focus);

    if (v > mean)
        v = 2.0 * mean - v;
    double r = v >= 0.0 ? v + r_min : r_min;
    return r > r_max ? r_max : r;
}

int ompl::RNG::halfNormalInt(int r_min, int r_max, double focus)
{
    auto r = (int)floor(halfNormalReal((double)r_min, (double)(r_max) + 1.0, focus));
    return (r > r_max) ? r_max : r;
}

// From: "Uniform Random Rotations", Ken Shoemake, Graphics Gems III,
//       pg. 124-132
void ompl::RNG::quaternion(double value[4])
{
    double x0 = uniDist_(generator_);
    double r1 = sqrt(1.0 - x0), r2 = sqrt(x0);
    double t1 = 2.0 * boost::math::constants::pi<double>() * uniDist_(generator_),
           t2 = 2.0 * boost::math::constants::pi<double>() * uniDist_(generator_);
    double c1 = cos(t1), s1 = sin(t1);
    double c2 = cos(t2), s2 = sin(t2);
    value[0] = s1 * r1;
    value[1] = c1 * r1;
    value[2] = s2 * r2;
    value[3] = c2 * r2;
}

// From Effective Sampling and Distance Metrics for 3D Rigid Body Path Planning, by James Kuffner, ICRA 2004
void ompl::RNG::eulerRPY(double value[3])
{
    value[0] = boost::math::constants::pi<double>() * (-2.0 * uniDist_(generator_) + 1.0);
    value[1] = acos(1.0 - 2.0 * uniDist_(generator_)) - boost::math::constants::pi<double>() / 2.0;
    value[2] = boost::math::constants::pi<double>() * (-2.0 * uniDist_(generator_) + 1.0);
}

void ompl::RNG::uniformNormalVector(std::vector<double> &v)
{
    // Generate a random value, the variate_generator is returning a shallow_array_adaptor, which will modify the value
    // array:
    v = sphericalDataPtr_->generate(v.size());
}

// See: http://math.stackexchange.com/a/87238
void ompl::RNG::uniformInBall(double r, std::vector<double> &v)
{
    // Draw a random point on the unit sphere
    uniformNormalVector(v);

    // Draw a random radius scale
    double radiusScale = r * std::pow(uniformReal(0.0, 1.0), 1.0 / static_cast<double>(v.size()));

    // Scale the point on the unit sphere
    std::transform(v.begin(), v.end(), v.begin(), [radiusScale](double x) { return radiusScale * x; });
}

void ompl::RNG::uniformProlateHyperspheroidSurface(const std::shared_ptr<const ProlateHyperspheroid> &phsPtr,
                                                   double value[])
{
    // Variables
    // The spherical point as a std::vector
    std::vector<double> sphere(phsPtr->getDimension());

    // Get a random point on the sphere
    uniformNormalVector(sphere);

    // Transform to the PHS
    phsPtr->transform(&sphere[0], value);
}

void ompl::RNG::uniformProlateHyperspheroid(const std::shared_ptr<const ProlateHyperspheroid> &phsPtr, double value[])
{
    // Variables
    // The spherical point as a std::vector
    std::vector<double> sphere(phsPtr->getDimension());

    // Get a random point in the sphere
    uniformInBall(1.0, sphere);

    // Transform to the PHS
    phsPtr->transform(&sphere[0], value);
}
