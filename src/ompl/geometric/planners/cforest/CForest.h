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

#ifndef OMPL_CONTRIB_C_FOREST_CFOREST_
#define OMPL_CONTRIB_C_FOREST_CFOREST_

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
            
            /** \brief Function signature to report intermediate solutions found by the underlying planner. */
            typedef boost::function<void(const Planner*, const std::vector<const base::State*> &, const base::Cost)> ReportIntermediateSolutionFn;

            CForest(const base::SpaceInformationPtr &si);

            virtual ~CForest();

            virtual void getPlannerData(base::PlannerData &data) const;

            virtual void clear();

            /** \brief Set the number of planner instances and the type T that will be running in different threads.*/
            template <class T>
            void setPlannerInstances(const std::size_t n)
            {
                planners_.clear();
                planners_.reserve(n);
                for (std::size_t i = 0 ; i < n; ++i)
                {
                    base::PlannerPtr planner (new T(si_));
                    planner->as<T>()->activateCForest();
                    planner->setProblemDefinition(pdef_);
                    planners_.push_back(planner);
                }
            }
            
            /** \brief Add an specific planner instance. */
            void addPlannerInstance(const base::PlannerPtr &planner);

            virtual void setup();

            virtual base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc);

            /** \brief Get an specific planner instance. */
            const base::PlannerPtr& getPlanner(const std::size_t idx) const
            {
                return planners_[idx];
            }

            ///////////////////////////////////////
            // Planner progress property functions
            /** \brief Get best cost among all the planners. */
            std::string getBestCost() const;

            /** \brief Get number of paths shared by the algorithm. */
            std::string getPathsShared() const;
            ///////////////////////////////////////

        private:

            /** \brief Callback to be called everytime a new, better solution is found by a planner. */
            void newSolutionFound(const base::Planner *planner, const std::vector<const base::State *> &states, const base::Cost cost);

            base::ValidStateSamplerPtr allocCForestValidStateSampler (const base::SpaceInformation *si);

        protected:

            /** \brief Manages the call to solve() for each individual planner. */
            void solveOne(base::Planner *planner, const base::PlannerTerminationCondition *ptc);

            /** \brief Optimization objective taken into account when planning. */
            base::OptimizationObjectivePtr               opt_;

            /** \brief The set of planners to be used. */
            std::vector<base::PlannerPtr>                planners_;

            /** \brief The maximum length of a motion to be added to a tree. */
            double                                       maxDistance_;

            //////////////////////////////
            // Planner progress properties
             /** \brief Cost of the best path found so far among planners. */
            base::Cost                                   totalBestCost_;       

            /** \brief Number of paths shared among threads. */
            int                                          pathsShared_;  
            
            boost::mutex                                 newSolutionFoundMutex_;
        };
    }
}

#endif
