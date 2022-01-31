/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, JSK, The University of Tokyo.
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
 *   * Neither the name of the JSK, The University of Tokyo nor the names of its
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

/* Author: Dave Coleman
 */

#ifndef OMPL_TOOLS_THUNDER_THUNDER_
#define OMPL_TOOLS_THUNDER_THUNDER_

#include <ompl/tools/experience/ExperienceSetup.h>  // the parent class

#include <ompl/tools/thunder/ThunderDB.h>
#include <ompl/geometric/planners/experience/ThunderRetrieveRepair.h>

#include <ompl/base/Planner.h>
#include <ompl/base/PlannerData.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/StateSpace.h>  // for storing to file

#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/PathSimplifier.h>

#include <ompl/tools/multiplan/ParallelPlan.h>

#include <ompl/util/Console.h>
#include <ompl/util/Exception.h>

namespace ompl
{

  // An enum of supported thunder planners, alphabetical order
  enum thunderPlanner
  {
      PLANNER_CFOREST,
      PLANNER_RRTCONNECT
  };
    namespace tools
    {
        /**
           @anchor Thunder
           @par Short description
           Thunder is an experience-based planning framework that learns to reduce computation time
           required to solve high-dimensional planning problems in varying environments.
           @par External documentation
           Berenson, Dmitry, Pieter Abbeel, and Ken Goldberg: A robot path planning framework that learns from
           experience, in <em>Robotics and Automation (ICRA), 2012 IEEE International Conference on. IEEE</em>, 2012.
           David Coleman, Ioan A. Sucan, Mark Moll, Kei Okada, Nikolaus Correll, "Experience-Based Planning with Sparse
           Roadmap Spanners"
           <a href="https://arxiv.org/pdf/1410.1950.pdf">[PDF]</a>
        */

        /// @cond IGNORE
        OMPL_CLASS_FORWARD(Thunder);
        /// @endcond

        /** \class ompl::geometric::ThunderPtr
            \brief A shared pointer wrapper for ompl::tools::Thunder */

        /** \brief Built off of SimpleSetup but provides support for planning from experience */
        class Thunder : public ompl::tools::ExperienceSetup
        {
        public:
            /** \brief Constructor needs the state space used for planning. */
            explicit Thunder(const base::SpaceInformationPtr &si, double stretch_factor = 1.2, double DenseD = 0.001,
                             double SparseD = 0.1);

            /** \brief Constructor needs the state space used for planning.
             *  \param space - the state space to plan in
             */
            explicit Thunder(const base::StateSpacePtr &space, double stretch_factor = 1.2, double DenseD = 0.001,
                             double SparseD = 0.1);

        private:
            /** \brief Shared constructor functions */
            void initialize();
            double stretch_factor_{};
            double DenseD_{};
            double SparseD_{};
            size_t n_parallel_plans_{0};
            size_t cforest_n_threads_{0};

        public:
            /** \brief Display debug data about potential available solutions */
            void printResultsInfo(std::ostream &out = std::cout) const override;

            /** \brief Display debug data about overall results from Thunder since being loaded */
            void printLogs(std::ostream &out = std::cout) const override;

            /** \brief Get the current planner */
            ompl::base::PlannerPtr &getPlanner()
            {
                return planner_;
            }

            /** \brief Get a pointer to the retrieve repair planner */
            ompl::geometric::ThunderRetrieveRepair &getRetrieveRepairPlanner() const
            {
                return static_cast<ompl::geometric::ThunderRetrieveRepair &>(*rrPlanner_);
            }

            /** \brief Set the planner to use for repairing experience paths
                inside the ThunderRetrieveRepair planner. If the planner is not
                set, a default planner is set. */
            void setRepairPlanner(const base::PlannerPtr &planner) override
            {
                static_cast<ompl::geometric::ThunderRetrieveRepair &>(*rrPlanner_).setRepairPlanner(planner);
            }

            /** \brief Set the planner allocator to use. This is only
                used if no planner has been set. This is optional -- a default
                planner will be used if no planner is otherwise specified. */
            void setPlannerAllocator(const base::PlannerAllocator &pa);

            /** \brief Run the planner for up to a specified amount of time (default is 1 second) */
            base::PlannerStatus solve(double time = 1.0) override;

            /** \brief Run the planner until \e ptc becomes true (at most) */
            base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;

            base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc, bool hybridize)
            {
                hybridize_ = hybridize;
                return solve(ptc);
            }

            /** \brief overload so we can toggle min / max solution counts before returning and toggle hybridize */
            base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc, const std::size_t minSolCount,
                                      const std::size_t maxSolCount, const bool hybridize)
            {
                hybridize_ = hybridize;
                minSolCount_ = minSolCount;
                maxSolCount_ = maxSolCount;
                return solve(ptc);
            }

            /** \brief Save the experience database to file */
            bool save() override;

            /** \brief Save the experience database to file if there has been a change */
            bool saveIfChanged() override;

            /** \brief Clear all planning data. This only includes
                data generated by motion plan computation. Planner
                settings, start & goal states are not affected. */
            void clear() override;

            /** \brief Print information about the current setup */
            void print(std::ostream &out = std::cout) const override;

            /** \brief set the from scratch planner to be CForest */
            void setCforest() {
              planner_type_ = thunderPlanner::PLANNER_CFOREST;
            }

            /** \brief set the from scratch planner to be RRT */
            void setRRT() {
              planner_type_ = thunderPlanner::PLANNER_RRTCONNECT;
            }

            /** \brief Set the number of threads to use for planning. */
            void setNumParallelPlans(const size_t n_parallel_plans) {
              n_parallel_plans_ = n_parallel_plans;
            }

            /** \brief Get the number of threads used for planning. */
            size_t getNumParallelPlans() const {
              return n_parallel_plans_;
            }

            /** \brief Set the number of threads to use for planning. */
            void setCforestNumThreads(const size_t cforest_n_threads) {
              cforest_n_threads_ = cforest_n_threads;
            }

            /** \brief Get the number of threads used for planning. */
            size_t getCforestNumThreads() const {
              return cforest_n_threads_;
            }

            /** \brief This method will create the necessary classes
                for planning. The solve() method will call this function automatically. */
            void setup() override;

            /** \brief Get a vector of all the planning data in the database */
            void getAllPlannerDatas(std::vector<ompl::base::PlannerDataPtr> &plannerDatas) const override;

            /** \brief Get the total number of paths stored in the database */
            std::size_t getExperiencesCount() const override;

            /** \brief Convert PlannerData to PathGeometric. Assume ordering of vertices is order of path */
            void convertPlannerData(const ompl::base::PlannerDataPtr &plannerData,
                                    ompl::geometric::PathGeometric &path);

            /**
             * \brief If path1 and path2 have a better start/goal match when reverse, then reverse path2
             * \param path to test against
             * \param path to reverse
             * \return true if reverse was necessary
             */
            bool reversePathIfNecessary(ompl::geometric::PathGeometric &path1, ompl::geometric::PathGeometric &path2);

            /** \brief Hook for getting access to debug data */
            ompl::tools::ThunderDBPtr getExperienceDB();

            /** \brief Allow accumlated experiences to be processed */
            bool doPostProcessing() override;

            void addSolutionToQueue(const std::shared_ptr<ompl::geometric::PathGeometric> &path_ptr)
            {
                queuedSolutionPaths_.push_back(*path_ptr);
            }

            void setStretchFactor(double stretch_factor)
            {
                stretch_factor_ = stretch_factor;
            };

            double getStretchFactor() const
            {
                return stretch_factor_;
            };

            void setSparseDelta(double SparseD)
            {
                SparseD_ = SparseD;
            };

            double getSparseDelta() const
            {
                return SparseD_;
            };

            void setDenseDelta(double DenseD)
            {
                DenseD_ = DenseD;
            };

            double getDenseDelta() const
            {
                return DenseD_;
            };

            void setSavePlansFromRecall (const bool savePlansFromRecall) {
                savePlansFromRecall_ = savePlansFromRecall;
            }

            bool getSavePlansFromRecall() {
                return savePlansFromRecall_;
            }

        protected:
            /**  The maintained experience planner instance */
            base::PlannerPtr rrPlanner_;

            /**  planners used for testing dual-thread scratch-only planning */
            std::vector<base::PlannerPtr> planner_vec_ {};

            /**  number of threads to use for RRTConnect planning. 0 for max */
            size_t n_threads {0};

            /**  Flag indicating whether dual thread scratch planning is enabled */
            bool dualThreadScratchEnabled_{true};

            /** \brief Instance of parallel planning to use for computing solutions in parallel */
            ompl::tools::ParallelPlanPtr pp_;

            /** \brief A shared object between all the planners for saving and loading previous experience */
            ompl::tools::ThunderDBPtr experienceDB_;

            /** \brief Accumulated experiences to be later added to experience database */
            std::vector<ompl::geometric::PathGeometric> queuedSolutionPaths_;

            bool hybridize_{true};            

            /** \brief Flag to indicate whether or not we save plans obtained from recall */
            bool savePlansFromRecall_ {true};

            std::size_t minSolCount_ {1};
            std::size_t maxSolCount_ {100};

            thunderPlanner planner_type_ {thunderPlanner::PLANNER_RRTCONNECT};

        };  // end of class Thunder

    }  // end of namespace tools

}  // end of namespace ompl
#endif
