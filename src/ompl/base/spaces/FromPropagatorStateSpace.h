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

#define eps std::numeric_limits<double>::epsilon()

namespace ompl
{
    namespace base
    {
        /** \brief  */
        template <typename T>
        class FromPropagatorStateSpace : public T
        {
            // \TODO: I cannot find a proper way of freeing the controls allocated when calling steer().
        public:
        
            FromPropagatorStateSpace (const control::StatePropagatorPtr &sp) : T(), sp_(sp)
            {
                if (sp_->getSpaceInformation()->getStateSpace()->getType() != T::getType())
                    throw Exception("State propagator's state space does not match the template parameter for FromPropagatorStateSpace.");

                T::setName("FromPropagator" + T::getName());
            }

            virtual ~FromPropagatorStateSpace()
            {
            }

            // \TODO: now the duration is used as distance. The demo has always velocity = 1,
            // so duration = distance. How to distinguish for a general system? It is probably system-dependent.
            virtual double distance (const State *state1, const State *state2) const
            {
                std::vector<control::TimedControl> tcontrols;
                // \TODO: How to avoid to use tcontrols here? Not used. steer function overload? with only 3 arguments?
                double duration = 0;
                bool steered = sp_->steer(state1,state2,tcontrols,duration);
                for (size_t i = 0; i < tcontrols.size(); ++i)
                    sp_->getSpaceInformation()->freeControl(tcontrols[i].first);

                if (steered)
                    return duration;
                return -1.0;
            }

            virtual void interpolate (const State *from, const State *to, const double t, State *state) const
            {
                if (t>=1.)
                {
                    if (to != state)
                        T::copyState(state, to);
                    return;
                }

                if (from != state)
                    T::copyState(state, from);

                if (t<=0.)
                    return;

                double duration = 0;

                std::vector<control::TimedControl> tcontrols;
                if(sp_->steer(from,to,tcontrols,duration))
                {
                    double interpT = t*duration, currentT = 0;
                    double control_time = 0;
                    unsigned int i = 0;

                    // applying timed controls until the requested interpolation time is reached.
                    while(currentT + eps*1e3 < interpT)
                    {
                        control_time = std::min(interpT - currentT, tcontrols[i].second);
                        sp_->propagate(state, tcontrols[i].first, control_time,  state);
                        currentT += control_time;
                        ++i;
                    }
                }

                for (size_t i = 0; i < tcontrols.size(); ++i)
                        sp_->getSpaceInformation()->freeControl(tcontrols[i].first);
            }

        protected:
            control::StatePropagatorPtr sp_;
        };

        class FromPropagatorMotionValidator : public MotionValidator
        {
        public:
            FromPropagatorMotionValidator(SpaceInformation* si) : MotionValidator(si)
            {
                defaultSettings();
            }
            
            FromPropagatorMotionValidator(const SpaceInformationPtr &si) : MotionValidator(si)
            {
                defaultSettings();
            }

            virtual ~FromPropagatorMotionValidator(void)
            {
            }
            
            virtual bool checkMotion(const State *s1, const State *s2) const;

            virtual bool checkMotion(const State *s1, const State *s2, std::pair<State*, double> &lastValid) const;

        private:
            //ReedsSheppStateSpace *stateSpace_;
            StateSpacePtr stateSpace_;
            void defaultSettings(void);
           };
    }
}

#endif
