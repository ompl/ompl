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

        class BasePathHead
        {
        public:
            BasePathHead(PathRestriction *restriction, Configuration *const xCurrent, Configuration *const xTarget);

            BasePathHead(const BasePathHead &rhs);

            ~BasePathHead();

            const base::State *getState() const;
            const base::State *getStateFiber() const;
            const base::State *getStateBase() const;

            base::State *getStateFiberNonConst() const;
            base::State *getStateBaseNonConst() const;
            Configuration *getConfiguration() const;

            void setCurrent(Configuration *, double);

            Configuration *getTargetConfiguration() const;
            const base::State *getStateTargetFiber() const;
            base::State *getStateTargetFiberNonConst() const;

            // \brief Remaining discrete states starting at head (including
            // head)
            int getNumberOfRemainingStates();

            // relative to where the head points
            const base::State *getBaseStateAt(int k) const;
            int getBaseStateIndexAt(int k) const;

            double getLocationOnBasePath() const;
            void setLocationOnBasePath(double d);

            int getNextValidBasePathIndex() const;
            int getLastValidBasePathIndex() const;
            void setLastValidBasePathIndex(int k);

            friend std::ostream &operator<<(std::ostream &, const BasePathHead &);

            void print(std::ostream &) const;

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
