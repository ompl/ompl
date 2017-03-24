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

#include "ompl/geometric/planners/cforest/CForestStateSpaceWrapper.h"
#include "ompl/geometric/planners/PlannerIncludes.h"
#include "ompl/tools/config/SelfConfig.h"

#include <mutex>

#include <vector>

namespace ompl
{
    namespace geometric
    {
        /**
           @anchor gCForest
           @par Short description
           CForest (Coupled Forest of Random Engrafting Search Trees) is a
           parallelization framework that is designed for single-query shortest
           path planning algorithms. It is not a planning algorithm <em>per se</em>.

           CForest is designed to be used with any random tree algorithm that operates
           in any configuration space such that: 1) the search tree has almost sure
           convergence to the optimal solution and 2) the configuration space obeys
           the triangle inequality. It relies in the OptimizationObjective set for
           the underlying planners.

           See also the extensive documentation [here](CForest.html).

           @par External documentation
           M. Otte, N. Correll, C-FOREST: Parallel Shortest Path Planning With
           Superlinear Speedup, IEEE Transactions on Robotics, Vol 20, No 3, 2013.
           DOI: [10.1109/TRO.2013.2240176](http://dx.doi.org/10.1109/TRO.2013.2240176)<br>
           [[PDF]](http://ieeexplore.ieee.org/ielx5/8860/6522877/06425493.pdf?tp=&amp;arnumber=6425493&amp;isnumber=6522877)
        */

        /** \brief Coupled Forest of Random Engrafting Search Trees */
        class CForest : public base::Planner
        {
        public:
            CForest(const base::SpaceInformationPtr &si);

            ~CForest() override;

            void getPlannerData(base::PlannerData &data) const override;

            void clear() override;

            /** \brief Add an specific planner instance. */
            template <class T>
            void addPlannerInstance()
            {
                auto space(std::make_shared<base::CForestStateSpaceWrapper>(this, si_->getStateSpace().get()));
                auto si(std::make_shared<base::SpaceInformation>(space));
                si->setStateValidityChecker(si_->getStateValidityChecker());
                si->setMotionValidator(si_->getMotionValidator());
                auto planner(std::make_shared<T>(si));
                space->setPlanner(planner.get());
                addPlannerInstanceInternal(planner);
            }

            /** \brief Add specific planner instances.
            CFOREST sets the planner's parameter named \e focus_search (if present) to the
            current value of CFOREST's \e focus_search parameter. */
            template <class T>
            void addPlannerInstances(std::size_t num = 2)
            {
                planners_.reserve(planners_.size() + num);
                for (std::size_t i = 0; i < num; ++i)
                {
                    addPlannerInstance<T>();
                }
            }

            /** \brief Remove all planner instances */
            void clearPlannerInstances()
            {
                planners_.clear();
            }
            /** \brief Return an specific planner instance. */
            base::PlannerPtr &getPlannerInstance(const std::size_t idx)
            {
                return planners_[idx];
            }

            void setup() override;

            base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;

            void addSampler(const base::StateSamplerPtr &sampler)
            {
                addSamplerMutex_.lock();
                samplers_.push_back(sampler);
                addSamplerMutex_.unlock();
            }

            /** \brief Option to control whether the search is focused during the search. */
            void setFocusSearch(const bool focus)
            {
                focusSearch_ = focus;
            }

            /** \brief Get the state of the search focusing option. */
            bool getFocusSearch() const
            {
                return focusSearch_;
            }

            /** \brief Set default number of threads to use when no planner instances are specified by the user. */
            void setNumThreads(unsigned int numThreads = 0);

            /** \brief Get default number of threads used by CForest when no planner instances are specified by the
             * user. */
            unsigned int getNumThreads()
            {
                return numThreads_;
            }

            /** \brief Get best cost among all the planners. */
            std::string getBestCost() const;

            /** \brief Get number of paths shared by the algorithm. */
            std::string getNumPathsShared() const;

            /** \brief Get number of states actually shared by the algorithm. */
            std::string getNumStatesShared() const;

        private:
            /** \brief Helper function to add a planner instance. */
            void addPlannerInstanceInternal(const base::PlannerPtr &planner);

            /** \brief Callback to be called everytime a new, better solution is found by a planner. */
            void newSolutionFound(const base::Planner *planner, const std::vector<const base::State *> &states,
                                  base::Cost cost);

        protected:
            /** \brief Manages the call to solve() for each individual planner. */
            void solve(base::Planner *planner, const base::PlannerTerminationCondition &ptc);

            /** \brief Optimization objective taken into account when planning. */
            base::OptimizationObjectivePtr opt_;

            /** \brief The set of planners to be used. */
            std::vector<base::PlannerPtr> planners_;

            /** \brief The set of sampler allocated by the planners */
            std::vector<base::StateSamplerPtr> samplers_;

            /** \brief Stores the states already shared to check if a specific state has been shared. */
            std::unordered_set<const base::State *> statesShared_;

            /** \brief Cost of the best path found so far among planners. */
            base::Cost bestCost_;

            /** \brief Number of paths shared among threads. */
            unsigned int numPathsShared_{0u};

            /** \brief Number of states shared among threads. */
            unsigned int numStatesShared_{0u};

            /** \brief Mutex to control the access to the newSolutionFound() method. */
            std::mutex newSolutionFoundMutex_;

            /** \brief Mutex to control the access to samplers_ */
            std::mutex addSamplerMutex_;

            /** \brief Flag to control whether the search is focused. */
            bool focusSearch_{true};

            /** \brief Default number of threads to use when no planner instances are specified by the user */
            unsigned int numThreads_;
        };
    }
}

#endif
