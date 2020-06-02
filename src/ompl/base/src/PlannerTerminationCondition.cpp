/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2011, Rice University
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

#include "ompl/base/PlannerTerminationCondition.h"
#include "ompl/util/Time.h"
#include <thread>
#include <utility>

namespace ompl
{
    namespace base
    {
        /// @cond IGNORE
        class PlannerTerminationCondition::PlannerTerminationConditionImpl
        {
        public:
            PlannerTerminationConditionImpl(PlannerTerminationConditionFn fn, double period)
              : fn_(std::move(fn))
              , period_(period)
              , terminate_(false)
              , thread_(nullptr)
              , evalValue_(false)
              , signalThreadStop_(false)
            {
                if (period_ > 0.0)
                    startEvalThread();
            }

            ~PlannerTerminationConditionImpl()
            {
                stopEvalThread();
            }

            bool eval() const
            {
                if (terminate_)
                    return true;
                if (period_ > 0.0)
                    return evalValue_;
                return fn_();
            }

            void terminate() const
            {
                // it is ok to have unprotected write here
                terminate_ = true;
            }

        private:
            /** \brief Start the thread evaluating termination conditions if not already started */
            void startEvalThread()
            {
                if (thread_ == nullptr)
                {
                    signalThreadStop_ = false;
                    evalValue_ = false;
                    thread_ = new std::thread([this]
                                              {
                                                  periodicEval();
                                              });
                }
            }

            /** \brief Stop the thread evaluating termination conditions if not already stopped */
            void stopEvalThread()
            {
                signalThreadStop_ = true;
                if (thread_ != nullptr)
                {
                    thread_->join();
                    delete thread_;
                    thread_ = nullptr;
                }
            }

            /** \brief Worker function that runs in a separate thread (calls computeEval())*/
            void periodicEval()
            {
                // we want to check for termination at least once every ms;
                // even though we may evaluate the condition itself more rarely

                unsigned int count = 1;
                time::duration s = time::seconds(period_);
                if (period_ > 0.001)
                {
                    count = 0.5 + period_ / 0.001;
                    s = time::seconds(period_ / (double)count);
                }

                while (!terminate_ && !signalThreadStop_)
                {
                    evalValue_ = fn_();
                    for (unsigned int i = 0; i < count; ++i)
                    {
                        if (terminate_ || signalThreadStop_)
                            break;
                        std::this_thread::sleep_for(s);
                    }
                }
            }

            /** \brief Function pointer to the piece of code that decides whether a termination condition has been met
             */
            PlannerTerminationConditionFn fn_;

            /** \brief Interval of time (seconds) to wait between calls to computeEval() */
            double period_;

            /** \brief Flag indicating whether the user has externally requested that the condition for termination
             * should become true */
            mutable bool terminate_;

            /** \brief Thread for periodicEval() */
            std::thread *thread_;

            /** \brief Cached value returned by fn_() */
            bool evalValue_;

            /** \brief Flag used to signal the condition evaluation thread to stop. */
            bool signalThreadStop_;
        };

        /// @endcond
    }
}

ompl::base::PlannerTerminationCondition::PlannerTerminationCondition(const PlannerTerminationConditionFn &fn)
  : impl_(std::make_shared<PlannerTerminationConditionImpl>(fn, -1.0))
{
}

ompl::base::PlannerTerminationCondition::PlannerTerminationCondition(const PlannerTerminationConditionFn &fn,
                                                                     double period)
  : impl_(std::make_shared<PlannerTerminationConditionImpl>(fn, period))
{
}

void ompl::base::PlannerTerminationCondition::terminate() const
{
    impl_->terminate();
}

bool ompl::base::PlannerTerminationCondition::eval() const
{
    return impl_->eval();
}

ompl::base::PlannerTerminationCondition ompl::base::plannerNonTerminatingCondition()
{
    return PlannerTerminationCondition([]
                                       {
                                           return false;
                                       });
}

ompl::base::PlannerTerminationCondition ompl::base::plannerAlwaysTerminatingCondition()
{
    return PlannerTerminationCondition([]
                                       {
                                           return true;
                                       });
}

ompl::base::PlannerTerminationCondition ompl::base::plannerOrTerminationCondition(const PlannerTerminationCondition &c1,
                                                                                  const PlannerTerminationCondition &c2)
{
    return PlannerTerminationCondition([c1, c2]
                                       {
                                           return c1() || c2();
                                       });
}

ompl::base::PlannerTerminationCondition
ompl::base::plannerAndTerminationCondition(const PlannerTerminationCondition &c1, const PlannerTerminationCondition &c2)
{
    return PlannerTerminationCondition([c1, c2]
                                       {
                                           return c1() && c2();
                                       });
}

ompl::base::PlannerTerminationCondition ompl::base::timedPlannerTerminationCondition(double duration)
{
    return timedPlannerTerminationCondition(time::seconds(duration));
}

ompl::base::PlannerTerminationCondition ompl::base::timedPlannerTerminationCondition(time::duration duration)
{
    const time::point endTime(time::now() + duration);
    return PlannerTerminationCondition([endTime]
                                       {
                                           return time::now() > endTime;
                                       });
}

ompl::base::PlannerTerminationCondition ompl::base::timedPlannerTerminationCondition(double duration, double interval)
{
    if (interval > duration)
        interval = duration;
    const time::point endTime(time::now() + time::seconds(duration));
    return PlannerTerminationCondition([endTime]
                                       {
                                           return time::now() > endTime;
                                       },
                                       interval);
}

ompl::base::PlannerTerminationCondition
ompl::base::exactSolnPlannerTerminationCondition(const ompl::base::ProblemDefinitionPtr& pdef)
{
    return PlannerTerminationCondition([pdef]
                                       {
                                           return pdef->hasExactSolution();
                                       });
}
