/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, University of Colorado, Boulder
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
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
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

#ifndef OMPL_TOOLS_EXPERIENCE__EXPERIENCE_SETUP_
#define OMPL_TOOLS_EXPERIENCE__EXPERIENCE_SETUP_

#include "ompl/geometric/SimpleSetup.h"

namespace ompl
{
    namespace tools
    {
        /// @cond IGNORE
        OMPL_CLASS_FORWARD(ExperienceSetup);
        /// @endcond

        /** \class ompl::geometric::ExperienceSetupPtr
            \brief A shared pointer wrapper for ompl::geometric::ExperienceSetup */

        /** \brief Create the set of classes typically needed to solve a
            geometric problem */
        class ExperienceSetup : public geometric::SimpleSetup
        {
        public:
            /**
             * \brief Simple logging functionality encapsled in a struct
             */
            struct ExperienceStats
            {
                ExperienceStats() = default;

                double getAveragePlanningTime() const
                {
                    if (numProblems_ == 0.0)
                        return 0.0;

                    return totalPlanningTime_ / numProblems_;
                }

                double getAverageInsertionTime() const
                {
                    if (numProblems_ == 0.0)
                        return 0.0;

                    // Clean up output
                    double time = totalInsertionTime_ / numProblems_;
                    if (time < 1e-8)
                        return 0.0;
                    return totalInsertionTime_ / numProblems_;
                }

                double numSolutionsFromRecall_{0.};
                double numSolutionsFromRecallSaved_{0.};
                double numSolutionsFromScratch_{0.};
                double numSolutionsFailed_{0.};
                double numSolutionsTimedout_{0.};
                double numSolutionsApproximate_{0.};
                double numSolutionsTooShort_{0.};  // less than 3 states
                double numProblems_{0.};           // input requests
                double totalPlanningTime_{0.};     // of all input requests, used for averaging
                double totalInsertionTime_{0.};    // of all input requests, used for averaging
            };

            /**
             * \brief Single entry for the csv data logging file
             */
            struct ExperienceLog
            {
                ExperienceLog() = default;
                // Times
                double planning_time{0.0};
                double insertion_time{0.0};
                // Solution properties
                std::string planner{"NA"};
                std::string result{"NA"};
                std::string is_saved{"NA"};
                // Failure booleans
                bool approximate{false};
                bool too_short{false};
                bool insertion_failed{false};
                // Lightning properties
                double score{0.0};
                // Thunder (SPARS) properties
                std::size_t num_vertices{0};
                std::size_t num_edges{0};
                std::size_t num_connected_components{0};
            };

            /** \brief Constructor needs the state space used for planning. */
            explicit ExperienceSetup(const base::SpaceInformationPtr &si);

            /** \brief Constructor needs the state space used for planning. */
            explicit ExperienceSetup(const base::StateSpacePtr &space);

            /** \brief Load the header (first row) of the csv file */
            void logInitialize();

            /** \brief Move data to string format and put in buffer */
            void convertLogToString(const ExperienceLog &log);

            /** \brief Display debug data about potential available solutions */
            virtual void printResultsInfo(std::ostream &out = std::cout) const = 0;

            /** \brief Display debug data about overall results  since being loaded */
            virtual void printLogs(std::ostream &out = std::cout) const = 0;

            /** \brief Save debug data about overall results since being loaded */
            virtual void saveDataLog(std::ostream &out = std::cout);

            /** \brief Set the planner to use for repairing experience paths
                inside the RetrieveRepair planner. If the planner is not
                set, a default planner is set. */
            virtual void setRepairPlanner(const base::PlannerPtr &planner) = 0;

            /** \brief Save the experience database to file */
            virtual bool save() = 0;

            /** \brief Save the experience database to file if there has been a change */
            virtual bool saveIfChanged() = 0;

            /** \brief Optionally disable the ability to use previous plans in solutions (but will still save them) */
            void enablePlanningFromRecall(bool enable);

            /** \brief Optionally disable the ability to plan from scratch
             *         Note: Lightning can still save modified experiences if they are different enough
             */
            void enablePlanningFromScratch(bool enable);

            /** \brief Get a vector of all the planning data in the database */
            virtual void getAllPlannerDatas(std::vector<ompl::base::PlannerDataPtr> &plannerDatas) const = 0;

            /** \brief Get the total number of paths stored in the database */
            virtual std::size_t getExperiencesCount() const = 0;

            /** \brief After setFile() is called, access the generated file path for loading and saving the experience
             * database */
            virtual const std::string &getFilePath() const;

            /** \brief Set the database file to load. Actual loading occurs when setup() is called
             *  \param filePath - full absolute path to a experience database to load
             */
            virtual bool setFilePath(const std::string &filePath);

            /**
             * \brief Getter for logging data
             */
            const ExperienceStats &getStats() const
            {
                return stats_;
            }

            /**
             * \brief Allow accumlated experiences to be processed
             */
            virtual bool doPostProcessing()
            {
                return true;
            }

        protected:
            /// Flag indicating whether recalled plans should be used to find solutions. Enabled by default.
            bool recallEnabled_{true};

            /// Flag indicating whether planning from scratch should be used to find solutions. Enabled by default.
            bool scratchEnabled_{true};

            /** \brief File location of database */
            std::string filePath_;

            // output data to file to analyze performance externally
            std::stringstream csvDataLogStream_;

            /** \brief States data for display to console  */
            ExperienceStats stats_;
        };
    }
}
#endif
