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

#include <boost/filesystem.hpp>

#include "ompl/base/SpaceInformation.h"
#include "ompl/base/ProblemDefinition.h"
#include "ompl/base/ScopedState.h"
#include "ompl/base/goals/GoalState.h"
#include "ompl/geometric/SimpleSetup.h"

#include "ompl/base/spaces/RealVectorStateSpace.h"

#include "../../resources/environment2D.h"

namespace ompl
{
    namespace geometric
    {

        /** \brief Define a one-dimensional state space with an updated distance definition (Manhattan distance) */
        class StateSpace2DMap1 : public base::RealVectorStateSpace
        {
        public:

            StateSpace2DMap1() : base::RealVectorStateSpace(1)
            {
            }

            virtual double distance(const base::State *state1, const base::State *state2) const
            {
                return fabs(state1->as<base::RealVectorStateSpace::StateType>()->values[0] -
                            state2->as<base::RealVectorStateSpace::StateType>()->values[0]);
            }
        };

        /** \brief Define a state validity checking function that assumes a compound state space for the 2DMap environment */
        static bool isValidFn2DMap1(const std::vector< std::vector<int> > *grid, const base::State *state)
        {
            const base::CompoundState *cstate = state->as<base::CompoundState>();

            /* planning is done in a continuous space, but our collision space representation is discrete */
            int x = (int)(cstate->as<base::RealVectorStateSpace::StateType>(0)->values[0]);
            int y = (int)(cstate->as<base::RealVectorStateSpace::StateType>(1)->values[0]);
            return (*grid)[x][y] == 0; // 0 means valid state
        }

        /** \brief Given a description of the environment, construct a
            complete planning context. The context is equivalent to
            the one defined by SimpleSetup2DMap but the representation
            of states and the collision checker are different (so that
            more code from OMPL is covered) */
        class SimpleSetup2DMap1 : public SimpleSetup
        {
        public:

            SimpleSetup2DMap1(const std::string &fileName) : SimpleSetup(constructSpace())
            {
                loadTestFile(fileName);
            }

            SimpleSetup2DMap1(const Environment2D &env) : SimpleSetup(constructSpace()), env_(env)
            {
                configure2DMap1();
            }

            /** \brief Load a test file */
            void loadTestFile(const std::string &testFile)
            {
                /* load environment */
                boost::filesystem::path path(TEST_RESOURCES_DIR);
                path = path / testFile;
                loadEnvironment(path.string().c_str(), env_);
                configure2DMap1();
            }

        protected:

            base::StateSpacePtr constructSpace()
            {
                return std::make_shared<StateSpace2DMap1>() + std::make_shared<StateSpace2DMap1>();
            }

            /** \brief Set the bounds and the state validity checker */
            void configure2DMap1()
            {
                base::RealVectorBounds bounds(1);
                bounds.low[0] = 0.0;
                bounds.high[0] = (double)env_.width - 0.000000001;
                getStateSpace()->as<base::CompoundStateSpace>()->as<StateSpace2DMap1>(0)->setBounds(bounds);

                bounds.high[0] = (double)env_.height - 0.000000001;
                getStateSpace()->as<base::CompoundStateSpace>()->as<StateSpace2DMap1>(1)->setBounds(bounds);

                setStateValidityChecker([this](const base::State *state)
                    {
                        return isValidFn2DMap1(&env_.grid, state);
                    });

                base::ScopedState<base::CompoundStateSpace> state(getSpaceInformation());
                state->as<base::RealVectorStateSpace::StateType>(0)->values[0] = env_.start.first;
                state->as<base::RealVectorStateSpace::StateType>(1)->values[0] = env_.start.second;

                base::ScopedState<base::CompoundStateSpace> gstate(getSpaceInformation());
                gstate->as<base::RealVectorStateSpace::StateType>(0)->values[0] = env_.goal.first;
                gstate->as<base::RealVectorStateSpace::StateType>(1)->values[0] = env_.goal.second;

                setStartAndGoalStates(state, gstate);
            }

            /** \brief Representation of environment */
            Environment2D env_;
        };
    }

}
