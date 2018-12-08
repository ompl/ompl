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

#ifndef OMPL_TEST_2DMAP_SETUP_
#define OMPL_TEST_2DMAP_SETUP_

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

        /** \brief Define a two-dimensional state space with an updated distance definition (Manhattan distance) */
        class StateSpace2DMap : public base::RealVectorStateSpace
        {
        public:

            StateSpace2DMap() : base::RealVectorStateSpace(2)
            {
            }

            virtual double distance(const base::State *state1, const base::State *state2) const
            {
                double dx = state1->as<base::RealVectorStateSpace::StateType>()->values[0] -
                    state2->as<base::RealVectorStateSpace::StateType>()->values[0];
                double dy = state1->as<base::RealVectorStateSpace::StateType>()->values[1] -
                    state2->as<base::RealVectorStateSpace::StateType>()->values[1];

                return sqrt(dx * dx + dy * dy);
            }
        };

        /** \brief Declare a class used in validating states. Such a class
            definition is needed for any use of a kinematic planner */
        class StateValidityChecker2DMap : public base::StateValidityChecker
        {
        public:

            StateValidityChecker2DMap(const base::SpaceInformationPtr &si, const std::vector< std::vector<int> > &grid) :
                base::StateValidityChecker(si), grid_(grid)
            {
            }

            virtual bool isValid(const base::State *state) const
            {
                /* planning is done in a continuous space, but our collision space representation is discrete */
                int x = (int)(state->as<base::RealVectorStateSpace::StateType>()->values[0]);
                int y = (int)(state->as<base::RealVectorStateSpace::StateType>()->values[1]);
                return grid_[x][y] == 0; // 0 means valid state
            }

        protected:

            /** \brief Map of environment */
            std::vector< std::vector<int> > grid_;
        };

        /** \brief Given a description of the environment, construct a complete planning context */
        class SimpleSetup2DMap : public SimpleSetup
        {
        public:

            SimpleSetup2DMap(const std::string &fileName)
                : SimpleSetup(std::make_shared<StateSpace2DMap>())
            {
                loadTestFile(fileName);
            }

            SimpleSetup2DMap(const Environment2D &env)
                : SimpleSetup(std::make_shared<StateSpace2DMap>()), env_(env)
            {
                configure2DMap();
            }

            /** \brief Load a test file */
            void loadTestFile(const std::string &testFile)
            {
                /* load environment */
                boost::filesystem::path path(TEST_RESOURCES_DIR);
                path = path / testFile;
                loadEnvironment(path.string().c_str(), env_);
                configure2DMap();
            }

        protected:

            /** \brief Set the bounds and the state validity checker */
            void configure2DMap()
            {
                base::RealVectorBounds sbounds(2);

                // dimension 0 (x) spans between [0, width)
                // dimension 1 (y) spans between [0, height)
                // since sampling is continuous and we round down, we allow values until just under the max limit
                // the resolution is 1.0 since we check cells only

                sbounds.low[0] = 0.0;
                sbounds.high[0] = (double)env_.width - 0.000000001;

                sbounds.low[1] = 0.0;
                sbounds.high[1] = (double)env_.height - 0.000000001;

                getStateSpace()->as<StateSpace2DMap>()->setBounds(sbounds);
                getSpaceInformation()->setStateValidityCheckingResolution(0.016);
                setStateValidityChecker(std::make_shared<StateValidityChecker2DMap>(
                    getSpaceInformation(), env_.grid));

                /* set the initial state; the memory for this is automatically cleaned by SpaceInformation */
                base::ScopedState<base::RealVectorStateSpace> state(getStateSpace());
                state->values[0] = env_.start.first;
                state->values[1] = env_.start.second;
                setStartState(state);

                /* set the goal state; the memory for this is automatically cleaned by SpaceInformation */
                auto goal(std::make_shared<base::GoalState>(getSpaceInformation()));
                base::ScopedState<base::RealVectorStateSpace> gstate(getStateSpace());
                gstate->values[0] = env_.goal.first;
                gstate->values[1] = env_.goal.second;
                goal->setState(gstate);
                goal->setThreshold(1e-3); // this basically means 0, but we want to account for numerical instabilities
                setGoal(goal);
                // we could have used setGoalState() as well
            }

            /** \brief Representation of environment */
            Environment2D env_;

        };


        /** \brief Construct an instance of space information (done automatically when using SimpleSetup) */
        static base::SpaceInformationPtr spaceInformation2DMap(const Environment2D &env)
        {
            auto sSpace(std::make_shared<StateSpace2DMap>());

            base::RealVectorBounds sbounds(2);

            // dimension 0 (x) spans between [0, width)
            // dimension 1 (y) spans between [0, height)
            // since sampling is continuous and we round down, we allow values until just under the max limit
            // the resolution is 1.0 since we check cells only

            sbounds.low[0] = 0.0;
            sbounds.high[0] = (double)env.width - 0.000000001;

            sbounds.low[1] = 0.0;
            sbounds.high[1] = (double)env.height - 0.000000001;

            sSpace->setBounds(sbounds);

            auto si(std::make_shared<base::SpaceInformation>(sSpace));
            si->setStateValidityCheckingResolution(0.016);

            si->setStateValidityChecker(std::make_shared<StateValidityChecker2DMap>(si, env.grid));

            si->setup();

            return si;
        }

        /** \brief Construct a problem definition  (done automatically when using SimpleSetup) */
        static base::ProblemDefinitionPtr problemDefinition2DMap(const base::SpaceInformationPtr &si, const Environment2D &env)
        {
            auto pdef(std::make_shared<base::ProblemDefinition>(si));

            /* set the initial state; the memory for this is automatically cleaned by SpaceInformation */
            base::ScopedState<base::RealVectorStateSpace> state(si);
            state->values[0] = env.start.first;
            state->values[1] = env.start.second;
            pdef->addStartState(state);

            /* set the goal state; the memory for this is automatically cleaned by SpaceInformation */
            auto goal(std::make_shared<base::GoalState>(si));
            base::ScopedState<base::RealVectorStateSpace> gstate(si);
            gstate->values[0] = env.goal.first;
            gstate->values[1] = env.goal.second;
            goal->setState(gstate);
            goal->setThreshold(1e-3); // this is basically 0, but we want to account for numerical instabilities
            pdef->setGoal(goal);

            return pdef;
        }

    }

}

#endif
