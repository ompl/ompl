/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2012, Willow Garage, Inc.
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

#ifndef OMPL_GEOMETRIC_TIME_PARAMETERIZATION_
#define OMPL_GEOMETRIC_TIME_PARAMETERIZATION_

#include "ompl/base/SpaceInformation.h"
#include "ompl/geometric/PathGeometric.h"
#include "ompl/util/ClassForward.h"

namespace ompl
{

    namespace geometric
    {

        /// @cond IGNORE
        /** \brief Forward declaration of ompl::geometric::TimeParameterization */
        ClassForward(TimeParameterization);
        /// @endcond

        /** \class ompl::geometric::TimeParameterizationPtr
            \brief A boost shared pointer wrapper for ompl::geometric::TimeParameterization */

        /** \brief This class contains routines that can parameterize geometric paths by time */
        class TimeParameterization
        {
        public:

            TimeParameterization(void)
            {
            }

            virtual ~TimeParameterization(void)
            {
            }

            /** \brief Compute the time parametrization for the states along this path. Return true on success.
                \param maxVel The maximum velocity to be attained
                \param maxAcc The maximum acceleration of the system
                \param times The time stamp (in seconds) for each of the states along the path. Starts at 0.0
                \param velocities The time stamp (in seconds) for each of the states along the path.
                \param startVel The velocity at the start of the path
                \param endVel The velocity at the end of the path
                \param maxSteps The maximum number of steps to run this algorithm for (the algorithm is iterative)

                \note This method attempts to get to the maximum velocity as quickly as possible, while staying within
                acceleration limits. */
            bool computeFastTimeParametrization(const PathGeometric &path,
                                                double maxVel, double maxAcc,
                                                std::vector<double> &times,  
                                                std::vector<double> &velocities,   
                                                double startVel = 0.0,
                                                double endVel = 0.0,
                                                unsigned int maxSteps = 10) const;

            /** \brief Compute the time parametrization for the states along this path. Return true on success.
                \param maxVel The maximum velocity to be attained, specified for each substate of the state space used for \e path. 
                \param maxVelDefault If \e maxVel is not specified for some of the substates, this value is used instead.
                \param maxAcc The maximum acceleration of the system, specified for each substate of the state space used for \e path. 
                \param maxAccDefault If \e maxAcc is not specified for some of the substates, this value is used instead.
                \param times The time stamp (in seconds) for each of the states along the path. Starts at 0.0
                \param velocities The time stamp (in seconds) for each of the states along the path. Velocities are specified for each of the known substates.
                \param startVel The velocity at the start of the path, for every substate
                \param endVel The velocity at the end of the path, for every substate
                \param maxSteps The maximum number of steps to run this algorithm for (the algorithm is iterative)

                \note This method attempts to get to the maximum velocity as quickly as possible, while staying within
                acceleration limits. */
            bool computeFastTimeParametrization(const PathGeometric &path,
                                                const std::map<std::string, double> &maxVel,
                                                const std::map<std::string, double> &maxAcc,
                                                std::vector<double> &times,
                                                std::map<std::string, std::vector<double> > &velocities, 
                                                double startVel = 0.0,
                                                double endVel = 0.0,
                                                unsigned int maxSteps = 10) const;

            /** \brief Compute the time parametrization for the states along this path. Return true on success.
                \param maxVel The maximum velocity to be attained, specified for each substate of the state space used for \e path. 
                \param maxVelDefault If \e maxVel is not specified for some of the substates, this value is used instead.
                \param maxAcc The maximum acceleration of the system, specified for each substate of the state space used for \e path. 
                \param maxAccDefault If \e maxAcc is not specified for some of the substates, this value is used instead.
                \param times The time stamp (in seconds) for each of the states along the path. Starts at 0.0
                \param velocities The time stamp (in seconds) for each of the states along the path. Velocities are specified for each of the known substates.
                \param startVel The velocity at the start of the path
                \param endVel The velocity at the end of the path
                \param maxSteps The maximum number of steps to run this algorithm for (the algorithm is iterative)

                \note This method attempts to get to the maximum velocity as quickly as possible, while staying within
                acceleration limits. */
            bool computeFastTimeParametrization(const PathGeometric &path,
                                                const std::map<std::string, double> &maxVel,
                                                const std::map<std::string, double> &maxAcc,
                                                std::vector<double> &times,
                                                std::map<std::string, std::vector<double> > &velocities, 
                                                const std::map<std::string, double> &startVel,
                                                const std::map<std::string, double> &endVel,
                                                unsigned int maxSteps = 10) const;
        private:
          
            bool computeFastTimeParametrizationPart(const PathGeometric &path,
                                                    const base::StateSpace::SubstateLocation &sloc, 
                                                    double maxVel, double maxAcc, 
                                                    std::vector<double> &times, 
                                                    std::vector<double> &velocities,
                                                    double startVel, double endVel,
                                                    unsigned int maxSteps) const;
        };
    }
}

#endif
