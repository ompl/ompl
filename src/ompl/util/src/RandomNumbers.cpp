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

#include "ompl/util/RandomNumbers.h"
#include "ompl/util/Exception.h"
#include "ompl/util/Console.h"
#include <boost/random/lagged_fibonacci.hpp>
#include <boost/random/uniform_int.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/once.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/math/constants/constants.hpp>
#include <boost/random/uniform_on_sphere.hpp>
//For pre C++ 11 gamma function
#include <boost/math/special_functions/gamma.hpp>

//The Eigen Includes:
//Inversion and determinants
#include <Eigen/LU>
//SVD decomposition
#include <Eigen/SVD>

//We want to throw some exceptions
#include "ompl/util/Exception.h"

/// @cond IGNORE
namespace
{
    /// We use a different random number generator for the seeds of the
    /// other random generators. The root seed is from the number of
    /// nano-seconds in the current time, or given by the user.
    class RNGSeedGenerator
    {
    public:
        RNGSeedGenerator() :
            someSeedsGenerated_(false),
            firstSeed_((boost::uint32_t)(boost::posix_time::microsec_clock::universal_time() -
                boost::posix_time::ptime(boost::date_time::min_date_time)).total_microseconds()),
            sGen_(firstSeed_),
            sDist_(1, 1000000000),
            s_(sGen_, sDist_)
        {
        }

        boost::uint32_t firstSeed()
        {
            boost::mutex::scoped_lock slock(rngMutex_);
            return firstSeed_;
        }

        void setSeed(boost::uint32_t seed)
        {
            boost::mutex::scoped_lock slock(rngMutex_);
            if (seed > 0)
            {
                if (someSeedsGenerated_)
                {
                    OMPL_ERROR("Random number generation already started. Changing seed now will not lead to deterministic sampling.");
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
                else
                {
                    OMPL_WARN("Random generator seed cannot be 0. Using 1 instead.");
                    seed = 1;
                }
            }
            sGen_.seed(seed);
        }

        boost::uint32_t nextSeed()
        {
            boost::mutex::scoped_lock slock(rngMutex_);
            someSeedsGenerated_ = true;
            return s_();
        }

    private:
        bool                       someSeedsGenerated_;
        boost::uint32_t            firstSeed_;
        boost::mutex               rngMutex_;
        boost::lagged_fibonacci607 sGen_;
        boost::uniform_int<>       sDist_;
        boost::variate_generator<boost::lagged_fibonacci607&, boost::uniform_int<> > s_;
    };

    static boost::once_flag g_once = BOOST_ONCE_INIT;
    static boost::scoped_ptr<RNGSeedGenerator> g_RNGSeedGenerator;

    void initRNGSeedGenerator()
    {
        g_RNGSeedGenerator.reset(new RNGSeedGenerator());
    }

    RNGSeedGenerator& getRNGSeedGenerator()
    {
        boost::call_once(&initRNGSeedGenerator, g_once);
        return *g_RNGSeedGenerator;
    }
}  // namespace
/// @endcond

boost::uint32_t ompl::RNG::getSeed()
{
    return getRNGSeedGenerator().firstSeed();
}

void ompl::RNG::setSeed(boost::uint32_t seed)
{
    getRNGSeedGenerator().setSeed(seed);
}

ompl::RNG::RNG() :
    localSeed_(getRNGSeedGenerator().nextSeed()),
    generator_(localSeed_),
    uniDist_(0, 1),
    normalDist_(0, 1),
    uni_(generator_, uniDist_),
    normal_(generator_, normalDist_)
{
}

ompl::RNG::RNG(boost::uint32_t localSeed) :
    localSeed_(localSeed),
    generator_(localSeed_),
    uniDist_(0, 1),
    normalDist_(0, 1),
    uni_(generator_, uniDist_),
    normal_(generator_, normalDist_)
{
}

void ompl::RNG::setLocalSeed(boost::uint32_t localSeed)
{
    // Store the seed
    localSeed_ = localSeed;

    // Change the generator's seed
    generator_.seed(localSeed_);

    // Reset the variate generators, as they can cache values
    uni_.distribution().reset();
    normal_.distribution().reset();

}

double ompl::RNG::halfNormalReal(double r_min, double r_max, double focus)
{
    assert(r_min <= r_max);

    const double mean = r_max - r_min;
    double       v    = gaussian(mean, mean/focus);

    if (v > mean) v = 2.0 * mean - v;
    double r = v >= 0.0 ? v + r_min : r_min;
    return r > r_max ? r_max : r;
}

int ompl::RNG::halfNormalInt(int r_min, int r_max, double focus)
{
    int r = (int)floor(halfNormalReal((double)r_min, (double)(r_max) + 1.0, focus));
    return (r > r_max) ? r_max : r;
}

// From: "Uniform Random Rotations", Ken Shoemake, Graphics Gems III,
//       pg. 124-132
void ompl::RNG::quaternion(double value[4])
{
    double x0 = uni_();
    double r1 = sqrt(1.0 - x0), r2 = sqrt(x0);
    double t1 = 2.0 * boost::math::constants::pi<double>() * uni_(), t2 = 2.0 * boost::math::constants::pi<double>() * uni_();
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
    value[0] = boost::math::constants::pi<double>() * (-2.0 * uni_() + 1.0);
    value[1] = acos(1.0 - 2.0 * uni_()) - boost::math::constants::pi<double>() / 2.0;
    value[2] = boost::math::constants::pi<double>() * (-2.0 * uni_() + 1.0);
}


void ompl::RNG::uniformNormalVector(unsigned int n, double value[])
{
    //Construct the Boost distribution and generator. These would ideally be at the class level, but the spherical distribution requires dimension at construction
    boost::uniform_on_sphere<> uniformSphereDist(n);
    boost::variate_generator<boost::mt19937&, boost::uniform_on_sphere<> > normalizedVector(generator_, uniformSphereDist);

    //A temporary vector for the BOOST return value
    std::vector<double> rVector;

    //Generate a random value, the variate_generator returns a std::vector
    rVector = normalizedVector();

    //Copy into the array, wish I knew of a better way than having used the temporary vector...
    std::copy(rVector.begin(), rVector.end(), value);
}

//See: http://math.stackexchange.com/a/87238
void ompl::RNG::uniformInBall(double r, unsigned int n, double value[])
{
    //Draw a random point on the unit sphere
    this->uniformNormalVector(n, value);

    //Draw a random radius scale
    double radiusScale = r*std::pow(this->uniformReal(0.0, 1.0), 1.0/static_cast<double>(n));

    //Scale the point on the unit sphere
    for (unsigned int i = 0u; i < n; ++i)
    {
        value[i] = radiusScale*value[i];
    }
}

void ompl::RNG::uniformProlateHyperspheroidSurface(ProlateHyperspheroidPtr phsPtr, unsigned int n, double value[])
{
    //Variables
    //The spherical point as a std::vector
    std::vector<double> sphere(n);

    //Get a random point on the sphere
    this->uniformNormalVector(n, &sphere[0]);

    //Transform to the PHS
    phsPtr->transform(n, &sphere[0], value);
}

void ompl::RNG::uniformProlateHyperspheroid(ProlateHyperspheroidPtr phsPtr, unsigned int n, double value[])
{
    //Variables
    //The spherical point as a std::vector
    std::vector<double> sphere(n);

    //Get a random point in the sphere
    this->uniformInBall(1.0, n, &sphere[0]);

    //Transform to the PHS
    phsPtr->transform(n, &sphere[0], value);
}







ompl::ProlateHyperspheroid::ProlateHyperspheroid(unsigned int n, const double focus1[], const double focus2[])
  : dim_(n),
  transverseDiameter_(0.0), //Initialize to something.
  isTransformUpToDate_(false)
{
    //Copy the arrays into their Eigen containers via the Eigen::Map "view"
    xFocus1_ = Eigen::Map<const Eigen::VectorXd>(focus1, dim_);
    xFocus2_ = Eigen::Map<const Eigen::VectorXd>(focus2, dim_);

    //Calculate the minimum transverse diameter
    minTransverseDiameter_ = (xFocus1_ - xFocus2_).norm();

    //Calculate the centre:
    xCentre_ = 0.5*(xFocus1_ + xFocus2_);

    //Calculate the rotation
    this->updateRotation();
}

void ompl::ProlateHyperspheroid::setTransverseDiameter(double transverseDiameter)
{
    if (transverseDiameter < minTransverseDiameter_)
    {
        std::cout << transverseDiameter << " < " << minTransverseDiameter_ << std::endl;
        throw Exception("Transverse diameter cannot be less than the distance between the foci.");
    }

    //Store and update if changed
    if (transverseDiameter_ != transverseDiameter)
    {
        //Mark as out of date
        isTransformUpToDate_ = false;

        //Store
        transverseDiameter_ = transverseDiameter;

        //Update the transform
        this->updateTransformation();
    }
    //No else, the diameter didn't change
}

void ompl::ProlateHyperspheroid::transform(unsigned int n, const double sphere[], double phs[])
{
    if (isTransformUpToDate_ == false)
    {
      throw Exception("The transformation is not up to date in the PHS class. Has the transverse diameter been set?");
    }

    //Calculate the tranformation and offset, using Eigen::Map views of the data
    Eigen::Map<Eigen::VectorXd>(phs, n) = transformationWorldFromEllipse_*Eigen::Map<const Eigen::VectorXd>(sphere, n) + xCentre_;
}

bool ompl::ProlateHyperspheroid::isInPhs(unsigned int n, const double point[])
{
    if (isTransformUpToDate_ == false)
    {
        //The transform is not up to date until the transverse diameter has been set
        throw Exception ("The transverse diameter has not been set");
    }

    return (this->getPathLength(n, point) <= transverseDiameter_);
}

double ompl::ProlateHyperspheroid::getPhsMeasure(void)
{
    if (isTransformUpToDate_ == false)
    {
        //The transform is not up to date until the transverse diameter has been set, therefore we have no transverse diameter and we have infinite measure
        return std::numeric_limits<double>::infinity();
    }
    else
    {
        //Calculate and return:
        return phsMeasure_;
    }
}

double ompl::ProlateHyperspheroid::getPhsMeasure(double tranDiam)
{
    return calcPhsMeasure(dim_, minTransverseDiameter_, tranDiam);
}

double ompl::ProlateHyperspheroid::unitNBallMeasure(unsigned int N)
{
    return std::pow(std::sqrt(boost::math::constants::pi<double>()), static_cast<double>(N)) / boost::math::tgamma(static_cast<double>(N)/2.0 + 1.0);
}

double ompl::ProlateHyperspheroid::calcPhsMeasure(unsigned int N, double minTransverseDiameter, double transverseDiameter)
{
    if (transverseDiameter < minTransverseDiameter)
    {
        throw Exception("Transverse diameter cannot be less than the minimum transverse diameter.");
    }
    //Variable
    //The conjugate diameter:
    double conjugateDiameter;
    //The Lebesgue measure return value
    double lmeas;

    //Calculate the conjugate diameter:
    conjugateDiameter = std::sqrt(transverseDiameter*transverseDiameter - minTransverseDiameter*minTransverseDiameter);

    //Calculate as a product series of the radii, noting that one is the transverse diameter/2.0, and the other N-1 are the conjugate diameter/2.0
    lmeas = transverseDiameter/2.0;
    for (unsigned int i = 1u; i < N; ++i)
    {
        lmeas = lmeas * conjugateDiameter/2.0;
    }

    //Then multiplied by the volume of the unit n-ball.
    lmeas = lmeas*unitNBallMeasure(N);

    //Return:
    return lmeas;
}

double ompl::ProlateHyperspheroid::getPathLength(unsigned int n, const double point[])
{
    return (xFocus1_ - Eigen::Map<const Eigen::VectorXd>(point, n)).norm() + (Eigen::Map<const Eigen::VectorXd>(point, n) - xFocus2_).norm();
}

void ompl::ProlateHyperspheroid::updateRotation(void)
{
    //Mark the transform as out of date
    isTransformUpToDate_ = false;

    //If the minTransverseDiameter_ is too close to 0, we treat this as a circle.
    double circleTol = 1E-9;
    if (minTransverseDiameter_ < circleTol)
    {
        rotationWorldFromEllipse_.setIdentity(dim_, dim_);
    }
    else
    {
        //Variables
        //The transverse axis of the PHS expressed in the world frame.
        Eigen::VectorXd transverseAxis(dim_);
        //The matrix representation of the Wahba problem
        Eigen::MatrixXd wahbaProb(dim_, dim_);
        //The middle diagonal matrix in the SVD solution to the Wahba problem
        Eigen::VectorXd middleM(dim_);

        //Calculate the major axis, storing as the first eigenvector
        transverseAxis = (xFocus2_ - xFocus1_ )/minTransverseDiameter_;

        //Calculate the rotation that will allow us to generate the remaining eigenvectors
        //Formulate as a Wahba problem, first forming the matrix a_j*a_i' where a_j is the transverse axis if the ellipse in the world frame, and a_i is the first basis vector of the world frame (i.e., [1 0 .... 0])
        wahbaProb = transverseAxis*Eigen::MatrixXd::Identity(dim_, dim_).col(0).transpose();

        //Then run it through the  SVD solver
        Eigen::JacobiSVD<Eigen::MatrixXd, Eigen::NoQRPreconditioner> svd(wahbaProb, Eigen::ComputeFullV | Eigen::ComputeFullU);

        //Then calculate the rotation matrix from the U and V components of SVD
        //Calculate the middle diagonal matrix
        middleM = Eigen::VectorXd::Ones(dim_);
        //Make the last value equal to det(U)*det(V) (zero-based indexing remember)
        middleM(dim_ - 1) = svd.matrixU().determinant()*svd.matrixV().determinant();

        //Calculate the rotation
        rotationWorldFromEllipse_ = svd.matrixU()*middleM.asDiagonal()*svd.matrixV().transpose();
    }
}

void ompl::ProlateHyperspheroid::updateTransformation(void)
{
    //Variables
    //The radii of the ellipse
    Eigen::VectorXd diagAsVector(dim_);
    //The conjugate diameters:
    double conjugateDiamater;

    //Calculate the conjugate radius
    conjugateDiamater = std::sqrt(transverseDiameter_*transverseDiameter_ - minTransverseDiameter_*minTransverseDiameter_);

    //Store into the diagonal matrix
    //All the elements but one are the conjugate radius
    diagAsVector.fill(conjugateDiamater/2.0);

    //The first element in diagonal is the transverse radius
    diagAsVector(0) = 0.5*transverseDiameter_;

    //Calculate the transformation matrix
    transformationWorldFromEllipse_ = rotationWorldFromEllipse_*diagAsVector.asDiagonal();

    //Calculate the measure:
    phsMeasure_ = calcPhsMeasure(dim_, minTransverseDiameter_, transverseDiameter_);

    //Mark as up to date
    isTransformUpToDate_ = true;
}
