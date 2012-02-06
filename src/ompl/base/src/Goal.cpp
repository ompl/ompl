/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, Rice University
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

#include "ompl/base/Goal.h"

#include <algorithm>
#include <limits>

#include <boost/thread/mutex.hpp>

/// @cond IGNORE
namespace ompl
{
    namespace base
    {

        class Goal::PlannerSolutionSet
        {
        public:
          
            PlannerSolutionSet(void) : msg_("Goal")
            {
            }
          
            void add(const PlannerSolution &s)
            {
                if (s.approximate_)
                    msg_.warn("Adding approximate solution");
                boost::mutex::scoped_lock slock(lock_);
                int index = solutions_.size();
                solutions_.push_back(s);
                solutions_.back().index_ = index;
                std::sort(solutions_.begin(), solutions_.end());
            }

            void clear(void)
            {
                boost::mutex::scoped_lock slock(lock_);
                solutions_.clear();
            }

            std::vector<PlannerSolution> getSolutions(void)
            {
                boost::mutex::scoped_lock slock(lock_);
                std::vector<PlannerSolution> copy = solutions_;
                return copy;
            }

            bool isApproximate(void)
            {
                boost::mutex::scoped_lock slock(lock_);
                bool result = false;
                if (!solutions_.empty())
                    result = solutions_[0].approximate_;
                return result;
            }

            double getDifference(void)
            {
                boost::mutex::scoped_lock slock(lock_);
                double diff = -1.0;
                if (!solutions_.empty())
                    diff = solutions_[0].difference_;
                return diff;
            }

            PathPtr getTopSolution(void)
            {
                boost::mutex::scoped_lock slock(lock_);
                PathPtr copy;
                if (!solutions_.empty())
                    copy = solutions_[0].path_;
                return copy;
            }

            std::size_t getSolutionCount(void)
            {
                boost::mutex::scoped_lock slock(lock_);
                std::size_t result = solutions_.size();
                return result;
            }

        private:

            std::vector<PlannerSolution> solutions_;
            boost::mutex                 lock_;
            msg::Interface               msg_;
        };
    }
}

/// @endcond

ompl::base::Goal::Goal(const SpaceInformationPtr &si) :
    type_(GOAL_ANY), si_(si), maximumPathLength_(std::numeric_limits<double>::infinity()), solutions_(new PlannerSolutionSet())
{
}

bool ompl::base::Goal::isAchieved(void) const
{
    return solutions_->getSolutionCount() > 0;
}

std::size_t ompl::base::Goal::getSolutionCount(void) const
{
    return solutions_->getSolutionCount();
}

ompl::base::PathPtr ompl::base::Goal::getSolutionPath(void) const
{
    return solutions_->getTopSolution();
}

void ompl::base::Goal::addSolutionPath(const PathPtr &path, bool approximate, double difference) const
{
    solutions_->add(PlannerSolution(path, approximate, difference));
}

bool ompl::base::Goal::isApproximate(void) const
{
    return solutions_->isApproximate();
}

double ompl::base::Goal::getDifference(void) const
{
    return solutions_->getDifference();
}

std::vector<ompl::base::PlannerSolution> ompl::base::Goal::getSolutions(void) const
{
    return solutions_->getSolutions();
}

void ompl::base::Goal::clearSolutionPaths(void) const
{
    solutions_->clear();
}

bool ompl::base::Goal::isSatisfied(const State *st, double *distance) const
{
    if (distance != NULL)
        *distance = std::numeric_limits<double>::max();
    return isSatisfied(st);
}

bool ompl::base::Goal::isSatisfied(const State *st, double pathLength, double *distance) const
{
    if (pathLength > maximumPathLength_)
    {
        if (distance != NULL)
            isSatisfied(st, distance);
        return false;
    }
    else
        return isSatisfied(st, distance);
}

void ompl::base::Goal::print(std::ostream &out) const
{
    out << "Goal memory address " << this << std::endl;
    out << "There are " << solutions_->getSolutionCount() << " solutions" << std::endl;
}
