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

#ifndef OMPL_BASE_SPACES_STATE_SPACE_FROM_PROPAGATOR_
#define OMPL_BASE_SPACES_STATE_SPACE_FROM_PROPAGATOR_

#include "ompl/base/StateSpace.h"
#include "ompl/control/StatePropagator.h"
#include "ompl/control/SpaceInformation.h"

#include "ompl/util/Exception.h"

namespace ompl
{
    namespace base
    {
/*T is a base State Space (namely SE2)
template <typename T>
class Bridge : public T {
public:
    Bridge(const StatePropagatorPtr &s) : T(), s_(s) {
                // check if the stateprop state space is the same as T (signature)}
    virtual void interpolate()...
}
*/
        /** \brief  */
        template <typename T>
        class StateSpaceFromPropagator : public T
        {
        public:
        
            StateSpaceFromPropagator (const control::StatePropagatorPtr &sp) : T(), sp_(sp)
            {
                if (sp_->getSpaceInformation()->getStateSpace()->getType() != T::getType())
                    throw Exception("State propagator's state space does not match the template parameter for StateSpaceFromPropagator.");

                T::setName("FromPropagator" + T::getName());
            }

            virtual ~StateSpaceFromPropagator()
            {
            }

            virtual double distance (const State *state1, const State *state2) const
            {
                return 1.0;
            }

            virtual void interpolate (const State *from, const State *to, const double t, State *state) const
            {
                if (t<0 || t>1)
                    throw Exception("Interpolation failed, time t out of bounds [0,1].");

                std::vector<control::TimedControl> tcontrols;
                double duration = 0;
                sp_->steer(from,to,tcontrols,duration);

                // Rescaling the relative t to the maximum control duration given by steer().
                double interpT = t*duration;
                double currentT = 0;
                double totalControlT = tcontrols[0].second;
                int i = 0;
                T::copyState(state,from);
                
                // \TODO: The number or steps and stepTime can be improved to reduce error but
                // this propagation error cannot be avoided.
                const int steps = 100;
                const double stepTime = (double)(duration/steps);
                
                while (currentT <= interpT)
                {
                     sp_->propagate(state, tcontrols[i].first, stepTime, state);
                    currentT += stepTime;
                    //std::cout << currentT << "\t" << stepTime << std::endl;
                    // Look for which control action to apply
                    if (currentT > totalControlT)
                    {
                        ++i;
                        totalControlT += tcontrols[i].second;
                    }
                }
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
