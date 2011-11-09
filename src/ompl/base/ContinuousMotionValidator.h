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

/* Author: Ryan Luna */

#ifndef OMPL_BASE_CONTINUOUS_MOTION_VALIDATOR_
#define OMPL_BASE_CONTINUOUS_MOTION_VALIDATOR_

#include "ompl/base/MotionValidator.h"
#include "ompl/base/SpaceInformation.h"

namespace ompl
{
    namespace base
    {
        /// \brief A motion validator that utilizes continuous collision checking
        class ContinuousMotionValidator : public MotionValidator
        {
        public:

            /// \brief Constructor
            ContinuousMotionValidator(SpaceInformation* si) : MotionValidator(si)
            {
                defaultSettings();
            }

            /// \brief Constructor
            ContinuousMotionValidator(const SpaceInformationPtr &si) : MotionValidator(si)
            {
                defaultSettings();
            }

            /// \brief Destructor
            virtual ~ContinuousMotionValidator(void)
            {
            }

            /// \brief Returns true if motion between s1 and s2 is collision free.
            virtual bool checkMotion(const State *s1, const State *s2) const;

            /// \brief Checks the motion between s1 and s2. If the motion is
            /// invalid, lastValid contains the last valid state and the
            /// normalized time [0,1] when this state occurs.
            virtual bool checkMotion(const State *s1, const State *s2, std::pair<State*, double> &lastValid) const;

        private:

            /// \brief Handle to the statespace that this motion validator operates in.
            StateSpace *stateSpace_;

            /// \brief Restore settings to default values.
            void defaultSettings(void);
        };
    }
}

#endif
