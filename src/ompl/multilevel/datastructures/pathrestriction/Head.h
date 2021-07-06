/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020,
 *  Max Planck Institute for Intelligent Systems (MPI-IS).
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
 *   * Neither the name of the MPI-IS nor the names
 *     of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written
 *     permission.
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

/* Author: Andreas Orthey */

#ifndef OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_PATH_RESTRICTION_BASEPATHHEAD__
#define OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_PATH_RESTRICTION_BASEPATHHEAD__
#include <ompl/multilevel/datastructures/BundleSpaceGraph.h>

namespace ompl
{
    namespace multilevel
    {
        OMPL_CLASS_FORWARD(BundleSpaceGraph);
        OMPL_CLASS_FORWARD(PathRestriction);
        OMPL_CLASS_FORWARD(FiberedProjection);

        using Configuration = ompl::multilevel::BundleSpaceGraph::Configuration;

        /** \brief A pointer to a specific location on the base path of the path
         * restriction
         *
         * This class contains information about the current location over the
         * base path, including the distance traveled from start, the last index
         * of the base path states and information about the fiber space and
         * fiber element at the current location. Can be used to interpolate
         * from current location to goal of path restriction.
        */

        class Head
        {
        public:
            Head(PathRestriction *restriction, Configuration *const xCurrent, Configuration *const xTarget);

            Head(const Head &rhs);

            ~Head();

            /** \brief Get state to which head points */
            const base::State *getState() const;
            /** \brief Get projection of state onto fiber space */
            const base::State *getStateFiber() const;
            /** \brief Get projection of state onto base space */
            const base::State *getStateBase() const;

            /** \brief Get projection of state onto fiber space (non const)*/
            base::State *getStateFiberNonConst() const;
            /** \brief Get projection of state onto base space (non const)*/
            base::State *getStateBaseNonConst() const;
            /** \brief Get state as configuration */
            Configuration *getConfiguration() const;

            /** \brief Setter for current configuration/state */
            void setCurrent(Configuration *, double);

            /** \brief Get target configuration */
            Configuration *getTargetConfiguration() const;
            /** \brief Get target configuration projected onto fiber */
            const base::State *getStateTargetFiber() const;
            /** \brief Get target configuration projected onto fiber (non const)*/
            base::State *getStateTargetFiberNonConst() const;

            /** \brief Remaining discrete states starting at head
             * (including head) and relative to the head */
            int getNumberOfRemainingStates();

            /** \brief Get target configuration projected onto fiber (non const)*/
            const base::State *getBaseStateAt(int k) const;
            /** \brief Get base state at base path index */
            int getBaseStateIndexAt(int k) const;

            /** \brief Get location in [0,1] on base path to which head points*/
            double getLocationOnBasePath() const;
            /** \brief Set location of head along base path. */
            void setLocationOnBasePath(double d);

            /** \brief Get next base path index (after head)*/
            int getNextValidBasePathIndex() const;
            /** \brief Get last base path index (before head)*/
            int getLastValidBasePathIndex() const;
            void setLastValidBasePathIndex(int k);

            /** \brief Pipe head to stream */
            friend std::ostream &operator<<(std::ostream &, const Head &);

            void print(std::ostream &) const;

            /** \brief Get underlying path restriction. */
            PathRestriction *getRestriction() const;

        private:
            double locationOnBasePath_{0.0};
            int lastValidIndexOnBasePath_{0};

            PathRestriction *restriction_{nullptr};

            Configuration *xCurrent_{nullptr};
            base::State *xBaseCurrent_{nullptr};
            base::State *xFiberCurrent_{nullptr};

            Configuration *xTarget_{nullptr};
            base::State *xFiberTarget_{nullptr};
        };
    }
}

#endif
