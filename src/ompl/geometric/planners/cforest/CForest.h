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

/* Authors: Javier V. Gómez, Ioan Sucan, Mark Moll */

#ifndef OMPL_GEOMETRIC_PLANNERS_CFOREST_CFOREST_
#define OMPL_GEOMETRIC_PLANNERS_CFOREST_CFOREST_


#include "ompl/geometric/planners/cforest/CForestStateSpace.h"
#include "ompl/geometric/planners/PlannerIncludes.h"
#include "ompl/tools/config/SelfConfig.h"

#include <boost/thread.hpp>

#include <vector>

namespace ompl
{

    namespace geometric
    {
        class CForest : public base::Planner
        {
        public:

            CForest(const base::SpaceInformationPtr &si);

            virtual ~CForest();

            virtual void getPlannerData(base::PlannerData &data) const;

            virtual void clear();

            /** \brief Add an specific planner instance. */
            template <class T>
            void addPlannerInstances(std::size_t num = 2)
            {
                planners_.reserve(planners_.size() + num);
                for (std::size_t i = 0 ; i < num; ++i)
                {
                    base::CForestStateSpace* cfspace = new base::CForestStateSpace(this, si_->getStateSpace().get());
                    base::StateSpacePtr space(cfspace);
                    base::SpaceInformationPtr si(new base::SpaceInformation(space));
                    base::PlannerPtr planner (new T(si));
                    planner->as<T>()->setPrune(true);

                    std::cerr << "planner " << i << ' ' << planner.get() << std::endl;
                    cfspace->setPlanner(planner.get());
                    si->setStateValidityChecker(si_->getStateValidityChecker());
                    si->setMotionValidator(si_->getMotionValidator());
                    planner->setProblemDefinition(pdef_);
                    planners_.push_back(planner);
                }
            }
            /** \brief remove all planner instances */
            void clearPlannerInstances()
            {
                planners_.clear();
            }
            /** \brief Return an specific planner instance. */
            base::PlannerPtr& getPlannerInstance(const std::size_t idx)
            {
                return planners_[idx];
            }

            virtual void setup();

            virtual base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc);

            void addSampler(base::StateSamplerPtr sampler)
            {
                samplers_.push_back(sampler);
            }

            /** \brief Get best cost among all the planners. */
            std::string getBestCost() const;

            /** \brief Get number of paths shared by the algorithm. */
            std::string getPathsShared() const;

        private:

            /** \brief Callback to be called everytime a new, better solution is found by a planner. */
            void newSolutionFound(const base::Planner *planner, const std::vector<const base::State *> &states, const base::Cost cost);

        protected:

            /** \brief Manages the call to solve() for each individual planner. */
            void solve(base::Planner *planner, const base::PlannerTerminationCondition &ptc);

            /** \brief Optimization objective taken into account when planning. */
            base::OptimizationObjectivePtr               opt_;

            /** \brief The set of planners to be used. */
            std::vector<base::PlannerPtr>                planners_;

            /** \brief The set of sampler allocated by the planners */
            std::vector<base::StateSamplerPtr>           samplers_;

             /** \brief Cost of the best path found so far among planners. */
            base::Cost                                   bestCost_;
            /** \brief Number of paths shared among threads. */
            unsigned int                                 pathsShared_;

            boost::mutex                                 newSolutionFoundMutex_;
        };
    }
}

#endif
