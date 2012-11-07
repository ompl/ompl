/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2011, Rice University.
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

/* Author: Ioan Sucan */

#include "ompl/tools/config/SelfConfig.h"
#include "ompl/tools/config/MagicConstants.h"
#include "ompl/util/Console.h"
#include <boost/thread/mutex.hpp>
#include <boost/shared_ptr.hpp>
#include <algorithm>
#include <limits>
#include <cmath>
#include <map>

/// @cond IGNORE
namespace ompl
{
    namespace tools
    {

        class SelfConfig::SelfConfigImpl
        {
            friend class SelfConfig;

        public:

            SelfConfigImpl(const base::SpaceInformationPtr &si) :
                si_(si), probabilityOfValidState_(-1.0), averageValidMotionLength_(-1.0)
            {
            }

            double getProbabilityOfValidState(void)
            {
                checkSetup();
                if (probabilityOfValidState_ < 0.0)
                    probabilityOfValidState_ = si_->probabilityOfValidState(magic::TEST_STATE_COUNT);
                return probabilityOfValidState_;
            }

            double getAverageValidMotionLength(void)
            {
                checkSetup();
                if (averageValidMotionLength_ < 0.0)
                    averageValidMotionLength_ = si_->averageValidMotionLength(magic::TEST_STATE_COUNT);
                return averageValidMotionLength_;
            }

            void configureValidStateSamplingAttempts(unsigned int &attempts)
            {
                if (attempts == 0)
                    attempts = magic::MAX_VALID_SAMPLE_ATTEMPTS;

                /*
                static const double log_of_0_9 = -0.105360516;
                if (attempts == 0)
                {
                    double p = 1.0 - getProbabilityOfValidState();
                    if (p > 0.0)
                        attempts = std::min((unsigned int)std::max((int)ceil(log_of_0_9 / log(p)), 1),
                                            magic::MAX_VALID_SAMPLE_ATTEMPTS);
                    else
                        attempts = 1;
                    msg_.debug("Number of attempts made at sampling a valid state in space %s is computed to be %u",
                               si_->getStateSpace()->getName().c_str(), attempts);
                }
                */
            }

            void configurePlannerRange(double &range)
            {
                if (range < std::numeric_limits<double>::epsilon())
                {
                    range = si_->getMaximumExtent() * magic::MAX_MOTION_LENGTH_AS_SPACE_EXTENT_FRACTION;
                    OMPL_DEBUG("Planner range detected to be %lf", range);
                }

                /*
                if (range < std::numeric_limits<double>::epsilon())
                {
                    range = getAverageValidMotionLength() / 2.0;
                    double b = si_->getMaximumExtent() * magic::MAX_MOTION_LENGTH_AS_SPACE_EXTENT_FRACTION;
                    if (range < std::numeric_limits<double>::epsilon())
                        range = b;
                    else
                        range = std::min(range, b);
                    msg_.debug("Planner range detected to be %lf", range);
                }
                */
            }

            void configureProjectionEvaluator(base::ProjectionEvaluatorPtr &proj)
            {
                checkSetup();
                if (!proj)
                {
                    OMPL_INFORM("Attempting to use default projection.");
                    proj = si_->getStateSpace()->getDefaultProjection();
                }
                if (!proj)
                    throw Exception("No projection evaluator specified");
                proj->setup();
            }

            void print(std::ostream &out) const
            {
                out << "Configuration parameters for space '" << si_->getStateSpace()->getName() << "'" << std::endl;
                out << "   - probability of a valid state is " << probabilityOfValidState_ << std::endl;
                out << "   - average length of a valid motion is " << averageValidMotionLength_ << std::endl;
            }

        private:

            void checkSetup(void)
            {
                if (!si_->isSetup())
                {
                    si_->setup();
                    probabilityOfValidState_ = -1.0;
                    averageValidMotionLength_ = -1.0;
                }
            }

            base::SpaceInformationPtr si_;
            double                    probabilityOfValidState_;
            double                    averageValidMotionLength_;

            boost::mutex              lock_;
        };

    }
}

/// @endcond

ompl::tools::SelfConfig::SelfConfig(const base::SpaceInformationPtr &si, const std::string &context) : context_(context)
{
    typedef std::map<base::SpaceInformation*, boost::shared_ptr<SelfConfigImpl> > ConfigMap;

    static ConfigMap    SMAP;
    static boost::mutex LOCK;

    boost::mutex::scoped_lock smLock(LOCK);
    ConfigMap::const_iterator it = SMAP.find(si.get());

    if (it != SMAP.end())
        impl_ = it->second.get();
    else
    {
        impl_ = new SelfConfigImpl(si);
        SMAP[si.get()].reset(impl_);
    }
}

/* ------------------------------------------------------------------------ */

double ompl::tools::SelfConfig::getProbabilityOfValidState(void)
{
    boost::mutex::scoped_lock iLock(impl_->lock_);
    return impl_->getProbabilityOfValidState();
}

double ompl::tools::SelfConfig::getAverageValidMotionLength(void)
{
    boost::mutex::scoped_lock iLock(impl_->lock_);
    return impl_->getAverageValidMotionLength();
}

void ompl::tools::SelfConfig::configureValidStateSamplingAttempts(unsigned int &attempts)
{
    boost::mutex::scoped_lock iLock(impl_->lock_);
    impl_->configureValidStateSamplingAttempts(attempts);
}

void ompl::tools::SelfConfig::configurePlannerRange(double &range)
{
    boost::mutex::scoped_lock iLock(impl_->lock_);
    impl_->configurePlannerRange(range);
}

void ompl::tools::SelfConfig::configureProjectionEvaluator(base::ProjectionEvaluatorPtr &proj)
{
    boost::mutex::scoped_lock iLock(impl_->lock_);
    return impl_->configureProjectionEvaluator(proj);
}

void ompl::tools::SelfConfig::print(std::ostream &out) const
{
    boost::mutex::scoped_lock iLock(impl_->lock_);
    impl_->print(out);
}
