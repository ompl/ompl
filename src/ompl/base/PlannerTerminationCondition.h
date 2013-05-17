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

#ifndef OMPL_BASE_PLANNER_TERMINATION_CONDITION_
#define OMPL_BASE_PLANNER_TERMINATION_CONDITION_

#include <boost/function.hpp>
#include <boost/thread.hpp>

namespace ompl
{

    namespace base
    {

        /** \brief Signature for functions that decide whether termination
            conditions have been met for a planner, even if no
            solution is found. This is usually reaching a time or
            memory limit. If the function returns true, the planner is
            signaled to terminate its computation. Otherwise,
            computation continues while this function returns false,
            until a solution is found. */
        typedef boost::function<bool()> PlannerTerminationConditionFn;

        /** \brief Encapsulate a termination condition for a motion
            planner. Planners will call operator() to decide whether
            they should terminate before a solution is found or
            not. operator() will return true if either the implemented
            condition is met (the call to eval() returns true) or if
            the user called terminate(true). */
        class PlannerTerminationCondition
        {
        public:

            /** \brief Construct a termination condition. By default, eval() will call the externally specified function \e fn to decide whether
                the planner should terminate. The function \e fn does not always need to be specified, if a different implementation of eval() is
                provided by a derived class. */
            PlannerTerminationCondition(const PlannerTerminationConditionFn &fn) : fn_(fn), terminate_(false)
            {
            }

            virtual ~PlannerTerminationCondition(void)
            {
            }

            /** \brief Return true if the planner should stop its computation */
            bool operator()(void) const
            {
                return terminate_ || eval();
            }

            /** \brief Cast as true if the planner should stop its computation */
            operator bool() const
            {
                return terminate_ || eval();
            }

            /** \brief Notify that the condition for termination should become true, regardless of what eval() returns.
                This function may be called while the condition is being evaluated by other threads. */
            void terminate(void) const;

            /** \brief The implementation of some termination condition. By default, this just calls \e fn_() */
            virtual bool eval(void) const;

        protected:

            /** \brief Function pointer to the piece of code that decides whether a termination condition has been met */
            PlannerTerminationConditionFn fn_;

            /** \brief Flag indicating whether the user has externally requested that the condition for termination should become true */
            mutable bool                  terminate_;
        };

        /** \brief Termination condition with lazy evaluation. This is
            just as a regular termination condition, except the
            condition is actually evaluated by computeEval() and the
            return value is stored in evalValue_. Every time eval() is
            called, evalValue_ is returned instead of actually
            evaluating the termination condition. Furthermore, the
            termination condition is evaluated every period_ seconds
            in a separate thread. The thread automatically starts when
            the condition is constructed and it terminates when the
            condition becomes true. */
        class PlannerThreadedTerminationCondition : public PlannerTerminationCondition
        {
        public:

            /** \brief Construct a termination condition that is
                evaluated every \e period seconds. The evaluation of
                the condition (the call to computeEval()) consists of
                calling \e fn().*/
            PlannerThreadedTerminationCondition(const PlannerTerminationConditionFn &fn, double period);

            virtual ~PlannerThreadedTerminationCondition(void);

            /** \brief Simply return the cached value for the termination condition (evalValue_) */
            virtual bool eval(void) const;

        protected:

            /** \brief Evaluate the termination condition. By default this is a call to fn_() from the base class */
            bool computeEval(void);

            /** \brief Start the thread evaluating termination conditions if not already started */
            void startEvalThread(void);

            /** \brief Stop the thread evaluating termination conditions if not already stopped */
            void stopEvalThread(void);

            /** \brief Worker function that runs in a separate thread (calls computeEval())*/
            void periodicEval(void);

            /** \brief Thread for periodicEval() */
            boost::thread *thread_;

            /** \brief Cached value returned by computeEval() */
            bool           evalValue_;

            /** \brief Interval of time (seconds) to wait between calls to computeEval() */
            double         period_;
        };

        /** \brief Simple termination condition that always returns false. The termination condition will never be met */
        class PlannerNonTerminatingCondition : public PlannerTerminationCondition
        {
        public:
            PlannerNonTerminatingCondition(void);
        };

        /** \brief Simple termination condition that always returns true. The termination condition will always be met */
        class PlannerAlwaysTerminatingCondition : public PlannerTerminationCondition
        {
        public:
            PlannerAlwaysTerminatingCondition(void);
        };

        /** \brief Combine two termination conditions into one. If either termination condition returns true, this one will return true as well. */
        class PlannerOrTerminationCondition : public PlannerTerminationCondition
        {
        public:
            PlannerOrTerminationCondition(const PlannerTerminationCondition &c1, const PlannerTerminationCondition &c2);
        };

        /** \brief Combine two termination conditions into one. Both termination conditions need to return true for this one to return true. */
        class PlannerAndTerminationCondition : public PlannerTerminationCondition
        {
        public:
            PlannerAndTerminationCondition(const PlannerTerminationCondition &c1, const PlannerTerminationCondition &c2);
        };

        /** \brief Return a termination condition that will become true \e duration seconds in the future (wall-time) */
        PlannerTerminationCondition         timedPlannerTerminationCondition(double duration);

        /** \brief Return a termination condition that will become true \e duration seconds in the future (wall-time), but is checked in a separate thread, every \e interval seconds; \e interval must be less than \e duration */
        PlannerThreadedTerminationCondition timedPlannerTerminationCondition(double duration, double interval);
    }
}

#endif
