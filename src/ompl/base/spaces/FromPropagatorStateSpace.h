/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, Rice University
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

/* Author: Javier V. Gomez */

#ifndef OMPL_BASE_SPACES_STATE_FROM_PROPAGATOR_SPACE_
#define OMPL_BASE_SPACES_STATE_FROM_PROPAGATOR_SPACE_

#include "ompl/base/StateSpace.h"
#include "ompl/control/StatePropagator.h"
#include "ompl/control/SpaceInformation.h"

#include "ompl/util/Exception.h"

#include <queue>

#define eps std::numeric_limits<double>::epsilon()

namespace ompl
{
    namespace base
    {
        typedef std::vector<control::TimedControl> TimedControls;

        template <typename T>
        class FromPropagatorMotionValidator;

        /** \brief  */
        template <typename T>
        class FromPropagatorStateSpace : public T
        {
            // \TODO: I cannot find a proper way of freeing the controls allocated when calling steer().
        public:
        
            FromPropagatorStateSpace(const control::StatePropagatorPtr &sp) : T(), sp_(sp)
            {
                siC_ = sp_->getSpaceInformation();
                if (siC_->getStateSpace()->getType() != T::getType())
                    throw Exception("State propagator's state space does not match the template parameter for FromPropagatorStateSpace.");

                T::setName("FromPropagator" + T::getName());
            }

            virtual ~FromPropagatorStateSpace()
            {
            }

            // \TODO This is the best way I found to solve this problem. Currently the MotionValidator requires both
            //        si and siC and we cannot set siC from the outside of this class.
            MotionValidator* allocMotionValidator(SpaceInformation *si)
            {
                si_ = si;
                MotionValidator *mv = new FromPropagatorMotionValidator< FromPropagatorStateSpace<T> >(siC_, si_);
                return mv;
            }

            // \TODO: now the duration is used as distance. The demo has always velocity = 1,
            // so duration = distance. How to distinguish for a general system? It is probably system-dependent.
            virtual double distance(const State *state1, const State *state2) const
            {
                TimedControls tcontrols;
                // \TODO: How to avoid to use tcontrols here? Not used. steer function overload? with only 3 arguments?
                double duration = 0;
                bool steered = sp_->steer(state1,state2,tcontrols,duration);
                for (size_t i = 0; i < tcontrols.size(); ++i)
                    sp_->getSpaceInformation()->freeControl(tcontrols[i].first);

                if (steered)
                    return duration;
                return -1.0;
            }

            virtual void interpolate(const State *from, const State *to, const double t, State *state) const
            {
                bool firstTime = true;
                TimedControls tcontrols;
                interpolate(from, to, t, firstTime, tcontrols, state);

                for (size_t i = 0; i < tcontrols.size(); ++i)
                        sp_->getSpaceInformation()->freeControl(tcontrols[i].first);
            }

            virtual void interpolate(const State *from, const State *to, const double t,
                                     bool &firstTime, TimedControls &tcontrols, State *state) const
            {
                if (firstTime)
                {
                    double duration = 0;
                    if (t >= 1.)
                    {
                        if (to != state)
                            T::copyState(state, to);
                        return;
                    }
                    else if (t <= 0.)
                    {
                        if (from != state)
                            T::copyState(state, from);
                        return;
                    }
                    sp_->steer(from,to,tcontrols,duration);
                    firstTime = false;
                }
                interpolate(from, tcontrols, t, state);
            }

        protected:

            virtual void interpolate(const State *from, const TimedControls &tcontrols, double t, State *state) const
            {
                // \TODO: receive this duration as argument or set as a class attribute. Is that OK?
                double duration = 0;
                for (size_t i = 0; i < tcontrols.size(); ++i)
                    duration += tcontrols[i].second;

                double interpT = duration*t, currentT = 0;
                double control_time = 0;
                unsigned int i = 0;

                T::copyState(state, from);

                // applying timed controls until the requested interpolation time is reached.
                while(currentT + eps*1e3 < interpT)
                {
                    control_time = std::min(interpT - currentT, tcontrols[i].second);
                    sp_->propagate(state, tcontrols[i].first, control_time,  state);
                    currentT += control_time;
                    ++i;
                }
            }

            control::StatePropagatorPtr sp_;
            SpaceInformation *si_;
            control::SpaceInformation *siC_;
        };

        template <typename T>
        class FromPropagatorMotionValidator : public MotionValidator
        {
        public:
            FromPropagatorMotionValidator(control::SpaceInformation *siC, SpaceInformation *si) : MotionValidator(si), siC_(siC)
            {
                defaultSettings();
            }
            FromPropagatorMotionValidator(const control::SpaceInformationPtr &siC, const SpaceInformationPtr &si) : MotionValidator(si), siC_(siC.get())
            {
                defaultSettings();
            }
            virtual ~FromPropagatorMotionValidator()
            {
            }

            virtual bool checkMotion(const State *s1, const State *s2) const
            {
                /* assume motion starts in a valid configuration so s1 is valid */
                if (!si_->isValid(s2))
                    return false;

                bool result = true, firstTime = true;
                TimedControls tcontrols;
                int nd = stateSpace_->as<T>()->validSegmentCount(s1, s2);

                /* initialize the queue of test positions */
                std::queue< std::pair<int, int> > pos;
                if (nd >= 2)
                {
                    pos.push(std::make_pair(1, nd - 1));

                    /* temporary storage for the checked state */
                    State *test = si_->allocState();

                    /* repeatedly subdivide the path segment in the middle (and check the middle) */
                    while (!pos.empty())
                    {
                        std::pair<int, int> x = pos.front();

                        int mid = (x.first + x.second) / 2;
                        stateSpace_->as<T>()->interpolate(s1, s2, (double)mid / (double)nd, firstTime, tcontrols, test);

                        if (!si_->isValid(test))
                        {
                            result = false;
                            break;
                        }

                        pos.pop();

                        if (x.first < mid)
                            pos.push(std::make_pair(x.first, mid - 1));
                        if (x.second > mid)
                            pos.push(std::make_pair(mid + 1, x.second));
                    }

                    si_->freeState(test);
                    for (size_t i = 0; i < tcontrols.size(); ++i)
                        siC_->freeControl(tcontrols[i].first);
                }

                if (result)
                    valid_++;
                else
                    invalid_++;

                return result;
            }

            virtual bool checkMotion(const State *s1, const State *s2, std::pair<State*, double> &lastValid) const
            {
                /* assume motion starts in a valid configuration so s1 is valid */
                bool result = true, firstTime = true;
                TimedControls tcontrols;
                int nd = stateSpace_->as<T>()->validSegmentCount(s1, s2);

                if (nd > 1)
                {
                    /* temporary storage for the checked state */
                    State *test = si_->allocState();

                    for (int j = 1 ; j < nd ; ++j)
                    {
                        stateSpace_->as<T>()->interpolate(s1, s2, (double)j / (double)nd, firstTime, tcontrols, test);
                        if (!si_->isValid(test))
                        {
                            lastValid.second = (double)(j - 1) / (double)nd;
                            if (lastValid.first)
                                stateSpace_->as<T>()->interpolate(s1, s2, lastValid.second, firstTime, tcontrols, lastValid.first);
                            result = false;
                            break;
                        }
                    }
                    si_->freeState(test);
                    for (size_t i = 0; i < tcontrols.size(); ++i)
                        siC_->freeControl(tcontrols[i].first);
                }

                if (result)
                    if (!si_->isValid(s2))
                    {
                        lastValid.second = (double)(nd - 1) / (double)nd;
                        if (lastValid.first)
                            stateSpace_->as<T>()->interpolate(s1, s2, lastValid.second, firstTime, tcontrols, lastValid.first);
                        result = false;
                    }

                if (result)
                    valid_++;
                else
                    invalid_++;

                return result;
            }

        private:
            StateSpacePtr stateSpace_;

            // Required to free controls allocated when interpolating.
            control::SpaceInformation *siC_;

            void defaultSettings()
            {
                //TODO: does this check even make sense? this motion validator is created in SpaceInformation::setDefaultMotionValidator()
                // so that implies a stateSpace exists. No cast check can be done unless this class is also templated as FromPropagatorStateSpace
                stateSpace_ = si_->getStateSpace();
                if (!stateSpace_)
                    throw Exception("No state space for motion validator");
            }
        };
    }
}

#endif
