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

#ifndef OMPL_TOOLS_THUNDER_THUNDERDB_
#define OMPL_TOOLS_THUNDER_THUNDERDB_

#include <ompl/base/StateSpace.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/base/PlannerData.h>
#include <ompl/base/PlannerDataStorage.h>
#include <ompl/base/State.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include <ompl/tools/thunder/SPARSdb.h>

namespace ompl
{
    namespace tools
    {
        /**
           @anchor ThunderDB
           @par Short description
           Database for storing and retrieving past plans
        */

        /// @cond IGNORE
        OMPL_CLASS_FORWARD(ThunderDB);
        /// @endcond

        using SPARSdbPtr = std::shared_ptr<ompl::geometric::SPARSdb>;

        /** \class ompl::geometric::ThunderDBPtr
            \brief A shared pointer wrapper for ompl::tools::ThunderDB */

        /** \brief Save and load entire paths from file */
        class ThunderDB
        {
        public:
            /** \brief Constructor needs the state space used for planning.
             *  \param space - state space
             */
            ThunderDB(const base::StateSpacePtr &space);

            /** \brief Deconstructor */
            virtual ~ThunderDB();

            /**
             * \brief Load database from file
             * \param fileName - name of database file
             * \return true if file loaded successfully
             */
            bool load(const std::string &fileName);

            /**
             * \brief Add a new solution path to our database. Des not actually save to file so
             *        experience will be lost if save() is not called
             * \param new path - must be non-const because will be interpolated
             * \param returned insertion time to add to db
             * \return true on success
             */
            bool addPath(ompl::geometric::PathGeometric &solutionPath, double &insertionTime);

            /**
             * \brief Save loaded database to file, except skips saving if no paths have been added
             * \param fileName - name of database file
             * \return true if file saved successfully
             */
            bool saveIfChanged(const std::string &fileName);

            /**
             * \brief Save loaded database to file
             * \param fileName - name of database file
             * \return true if file saved successfully
             */
            bool save(const std::string &fileName);

            /**
             * \brief Get a vector of all the planner datas in the database
             */
            void getAllPlannerDatas(std::vector<ompl::base::PlannerDataPtr> &plannerDatas) const;

            /** \brief Create the database structure for saving experiences */
            void setSPARSdb(ompl::tools::SPARSdbPtr &prm);

            /** \brief Hook for debugging */
            ompl::tools::SPARSdbPtr &getSPARSdb();

            /** \brief Find the k nearest paths to our queries one */
            bool findNearestStartGoal(int nearestK, const base::State *start, const base::State *goal,
                                      ompl::geometric::SPARSdb::CandidateSolution &candidateSolution,
                                      const base::PlannerTerminationCondition &ptc);

            /** \brief Print info to screen */
            void debugVertex(const ompl::base::PlannerDataVertex &vertex);
            void debugState(const ompl::base::State *state);

            /** \brief Get number of unsaved paths */
            int getNumPathsInserted() const
            {
                return numPathsInserted_;
            }

            /** \brief Getter for enabling experience database saving */
            bool getSavingEnabled()
            {
                return saving_enabled_;
            }

            /** \brief Setter for enabling experience database saving */
            void setSavingEnabled(bool saving_enabled)
            {
                saving_enabled_ = saving_enabled;
            }

            /**
             * \brief Check if anything has been loaded into DB
             * \return true if has no nodes
             */
            bool isEmpty()
            {
                return spars_->getNumVertices() == 0u;
            }

        protected:
            /// The created space information
            base::SpaceInformationPtr si_;  // TODO: is this even necessary?

            /// Helper class for storing each plannerData instance
            ompl::base::PlannerDataStorage plannerDataStorage_;

            // Track unsaved paths to determine if a save is required
            int numPathsInserted_;

            // Use SPARSdb's graph datastructure to store experience
            ompl::tools::SPARSdbPtr spars_;

            // Allow the database to save to file (new experiences)
            bool saving_enabled_;

        };  // end of class ThunderDB

    }  // end of namespace

}  // end of namespace
#endif
