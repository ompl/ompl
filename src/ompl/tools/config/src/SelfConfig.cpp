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
#include <boost/weak_ptr.hpp>
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
                wsi_(si), probabilityOfValidState_(-1.0), averageValidMotionLength_(-1.0)
            {
            }

            double getProbabilityOfValidState(void)
            {
                base::SpaceInformationPtr si = wsi_.lock();
                checkSetup(si);
                if (si && probabilityOfValidState_ < 0.0)
                    probabilityOfValidState_ = si->probabilityOfValidState(magic::TEST_STATE_COUNT);
                return probabilityOfValidState_;
            }

            double getAverageValidMotionLength(void)
            {
                base::SpaceInformationPtr si = wsi_.lock();
                checkSetup(si);
                if (si && averageValidMotionLength_ < 0.0)
                    averageValidMotionLength_ = si->averageValidMotionLength(magic::TEST_STATE_COUNT);
                return averageValidMotionLength_;
            }

            void configureValidStateSamplingAttempts(unsigned int &attempts)
            {
                if (attempts == 0)
                    attempts = magic::MAX_VALID_SAMPLE_ATTEMPTS;
            }

            void configurePlannerRange(double &range)
            {
                if (range < std::numeric_limits<double>::epsilon())
                {
                    base::SpaceInformationPtr si = wsi_.lock();
                    if (si)
                    {
                        range = si->getMaximumExtent() * magic::MAX_MOTION_LENGTH_AS_SPACE_EXTENT_FRACTION;
                        OMPL_DEBUG("Planner range detected to be %lf", range);
                    }
                    else
                        OMPL_ERROR("Unable to detect planner range. SpaceInformation instance has expired.");
                }
            }

            void configureProjectionEvaluator(base::ProjectionEvaluatorPtr &proj)
            {
                base::SpaceInformationPtr si = wsi_.lock();
                checkSetup(si);
                if (!proj && si)
                {
                    OMPL_INFORM("Attempting to use default projection.");
                    proj = si->getStateSpace()->getDefaultProjection();
                }
                if (!proj)
                    throw Exception("No projection evaluator specified");
                proj->setup();
            }

            void print(std::ostream &out) const
            {
                base::SpaceInformationPtr si = wsi_.lock();
                if (si)
                {
                    out << "Configuration parameters for space '" << si->getStateSpace()->getName() << "'" << std::endl;
                    out << "   - probability of a valid state is " << probabilityOfValidState_ << std::endl;
                    out << "   - average length of a valid motion is " << averageValidMotionLength_ << std::endl;
                }
                else
                    out << "EXPIRED" << std::endl;
            }

            bool expired(void) const
            {
                return wsi_.expired();
            }

        private:

            void checkSetup(const base::SpaceInformationPtr &si)
            {
                if (si)
                {
                    if (!si->isSetup())
                    {
                        si->setup();
                        probabilityOfValidState_ = -1.0;
                        averageValidMotionLength_ = -1.0;
                    }
                }
                else
                {
                    probabilityOfValidState_ = -1.0;
                    averageValidMotionLength_ = -1.0;
                }
            }

            // we store weak pointers so that the SpaceInformation instances are not kept in
            // memory until termination of the program due to the use of a static ConfigMap below
            boost::weak_ptr<base::SpaceInformation> wsi_;

            double                                  probabilityOfValidState_;
            double                                  averageValidMotionLength_;

            boost::mutex                            lock_;
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

    // clean expired entries from the map
    ConfigMap::iterator dit = SMAP.begin();
    while (dit != SMAP.end())
    {
        if (dit->second->expired())
            SMAP.erase(dit++);
        else
          ++dit;
    }

    ConfigMap::const_iterator it = SMAP.find(si.get());

    if (it != SMAP.end())
        impl_ = it->second.get();
    else
    {
        impl_ = new SelfConfigImpl(si);
        SMAP[si.get()].reset(impl_);
    }
}

ompl::tools::SelfConfig::~SelfConfig(void)
{
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
