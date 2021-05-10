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
   Desc:   Implementation of the Lightning Framework for experienced-based planning

   Paper:  Berenson, Dmitry, Pieter Abbeel, and Ken Goldberg.
           "A robot path planning framework that learns from experience."
           Robotics and Automation (ICRA), 2012 IEEE International Conference on. IEEE, 2012.

   Notes:  The user of this class should invoke the loading and saving from file, otherwise experiences
           will be lost.
*/

#ifndef OMPL_TOOLS_LIGHTNING_LIGHTNING_
#define OMPL_TOOLS_LIGHTNING_LIGHTNING_

#include "ompl/tools/experience/ExperienceSetup.h"
#include "ompl/base/Planner.h"
#include "ompl/base/PlannerData.h"
#include "ompl/base/ProblemDefinition.h"
#include "ompl/base/SpaceInformation.h"
#include "ompl/base/ProblemDefinition.h"

#include "ompl/geometric/PathGeometric.h"
#include "ompl/geometric/PathSimplifier.h"
#include "ompl/geometric/planners/experience/LightningRetrieveRepair.h"

#include "ompl/tools/multiplan/ParallelPlan.h"
#include "ompl/tools/config/SelfConfig.h"

#include "ompl/util/Console.h"
#include "ompl/util/Exception.h"

#include "ompl/tools/lightning/DynamicTimeWarp.h"

namespace ompl
{
    namespace tools
    {
        // class LightningDB; // forward declaration
        OMPL_CLASS_FORWARD(LightningDB);
        OMPL_CLASS_FORWARD(ParallelPlan);

        /**
           @anchor Lightning
           @par Short description
           The Lightning Framework is a experienced-based motion planner that recalls from a database of
           previously generated paths the most similar one to the current planning problem and attempts to repair it,
           while at the same time planning from scratch in a different thread
           @par External documentation
           Berenson, Dmitry, Pieter Abbeel, and Ken Goldberg: A robot path planning framework that learns from
           experience,
           in <em>Robotics and Automation (ICRA), 2012 IEEE International Conference on. IEEE</em>, 2012.
           DOI: <a href="http://dx.doi.org/10.1109/ICRA.2012.6224742">10.1109/ICRA.2012.6224742</a><br>
           <a href="http://users.wpi.edu/~dberenson/lightning.pdf">[PDF]</a>
        */

        /// @cond IGNORE
        OMPL_CLASS_FORWARD(Lightning);
        /// @endcond

        /** \class ompl::geometric::LightningPtr
            \brief A shared pointer wrapper for ompl::tools::Lightning */

        /** \brief Built off of SimpleSetup but provides support for planning from experience */
        class Lightning : public ompl::tools::ExperienceSetup
        {
        public:
            /** \brief Constructor needs the state space used for planning. */
            explicit Lightning(const base::SpaceInformationPtr &si);

            /** \brief Constructor needs the state space used for planning.
             *  \param space - the state space to plan in
             */
            explicit Lightning(const base::StateSpacePtr &space);

        private:
            /**
             * \brief Shared constructor functions
             */
            void initialize();

        public:
            /** \brief Display debug data about potential available solutions */
            void printResultsInfo(std::ostream &out = std::cout) const override;

            /** \brief Display debug data about overall results from Lightning since being loaded */
            void printLogs(std::ostream &out = std::cout) const override;

            /**
             * \brief Get a pointer to the retrieve repair planner
             */
            ompl::geometric::LightningRetrieveRepair &getLightningRetrieveRepairPlanner() const
            {
                return static_cast<ompl::geometric::LightningRetrieveRepair &>(*rrPlanner_);
            }

            /** \brief Set the planner to use for repairing experience paths
                inside the LightningRetrieveRepair planner. If the planner is not
                set, a default planner is set. */
            void setRepairPlanner(const base::PlannerPtr &planner) override
            {
                static_cast<og::LightningRetrieveRepair &>(*rrPlanner_).setRepairPlanner(planner);
            }

            /** \brief Set the planner allocator to use. This is only
                used if no planner has been set. This is optional -- a default
                planner will be used if no planner is otherwise specified. */
            void setPlannerAllocator(const base::PlannerAllocator &pa)
            {
                pa_ = pa;
                planner_.reset();
                // note: the rrPlanner_ never uses the allocator so does not need to be reset
                configured_ = false;
            }

            /** \brief Run the planner for up to a specified amount of time (default is 1 second) */
            base::PlannerStatus solve(double time = 1.0) override;

            /** \brief Run the planner until \e ptc becomes true (at most) */
            base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;

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

            /** \brief This method will create the necessary classes
                for planning. The solve() method will call this
                function automatically. */
            void setup() override;

            /** \brief Get a vector of all the planning data in the database */
            void getAllPlannerDatas(std::vector<ompl::base::PlannerDataPtr> &plannerDatas) const override;

            /** \brief Get the total number of paths stored in the database */
            std::size_t getExperiencesCount() const override;

            /**
             * \brief Convert PlannerData to PathGeometric. Assume ordering of vertices is order of path
             * \param PlannerData
             * \param PathGeometric
             */
            void convertPlannerData(const ompl::base::PlannerDataPtr &plannerData,
                                    ompl::geometric::PathGeometric &path);

            /** \brief Tool for comparing two paths and scoring them */
            const ompl::tools::DynamicTimeWarpPtr &getDynamicTimeWarp() const
            {
                return dtw_;
            }

        protected:
            /**
             * \brief If path1 and path2 have a better start/goal match when reverse, then reverse path2
             * \param path to test against
             * \param path to reverse
             * \return true if reverse was necessary
             */
            bool reversePathIfNecessary(ompl::geometric::PathGeometric &path1, ompl::geometric::PathGeometric &path2);

            /// The maintained experience planner instance
            base::PlannerPtr rrPlanner_;

            /** \brief Instance of parallel planning to use for computing solutions in parallel */
            ompl::tools::ParallelPlanPtr pp_;

            /** \brief A shared object between all the planners for saving and loading previous experience */
            ompl::tools::LightningDBPtr experienceDB_;

            /** \brief Tool for comparing two paths and scoring them */
            ompl::tools::DynamicTimeWarpPtr dtw_;

        };  // end of class Lightning

    }  // end of namespace

}  // end of namespace
#endif
