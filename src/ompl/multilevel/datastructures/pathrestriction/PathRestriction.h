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

#ifndef OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_PATH_RESTRICTION__
#define OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_PATH_RESTRICTION__
#include <ompl/multilevel/datastructures/BundleSpaceGraph.h>

namespace ompl
{
    namespace base
    {
        /// @cond IGNORE
        /** \brief Forward declaration of ompl::base::Path */
        OMPL_CLASS_FORWARD(Path);
        /// @endcond
    }
    namespace geometric
    {
        /// @cond IGNORE
        /** \brief Forward declaration of ompl::geometric::PathGeometric */
        OMPL_CLASS_FORWARD(PathGeometric);
        /// @endcond
    }
    namespace multilevel
    {
        /// @cond IGNORE
        /** \brief Forward declaration of ompl::multilevel::BundleSpaceGraph */
        OMPL_CLASS_FORWARD(BundleSpaceGraph);
        /** \brief Forward declaration of ompl::multilevel::PathSection */
        OMPL_CLASS_FORWARD(PathSection);
        /** \brief Forward declaration of ompl::multilevel::Head */
        OMPL_CLASS_FORWARD(Head);
        /** \brief Forward declaration of ompl::multilevel::FindSection */
        OMPL_CLASS_FORWARD(FindSection);
        /// @endcond

        using Configuration = ompl::multilevel::BundleSpaceGraph::Configuration;

        /** \brief Representation of path restriction
            (union of fibers over a base path).

            Class represents path restriction by keeping a set of discrete base
            path states. To access states inbetween, we use the
            interpolateBasePath method. To find a feasible section over a path
            restriction, we use the strategy pattern FindSection, which has
            different implementations.

            To use this class, you need to set a base path (setBasePath), then
            you can search for sections over this base path (using
            hasFeasibleSection). Internally, this calls the FindSection
            algorithm, which can be changed in Constructor method. Please see
            the class ompl::multilevel::FindSection for details on finding
            feasible sections.

            @par External documentation
            A. Orthey and M. Toussaint,
            Section Patterns: Efficiently Solving Narrow Passage Problems
            using Multilevel Motion Planning, <em>arXiv:2010.14524 [cs.RO]</em>, 2020
            [[PDF]](https://arxiv.org/pdf/2010.14524.pdf)

        */

        class PathRestriction
        {
        public:
            PathRestriction() = delete;
            PathRestriction(BundleSpaceGraph *);

            virtual ~PathRestriction();

            virtual void clear();

            /** \brief Set base path over which restriction is defined */
            void setBasePath(base::PathPtr);

            /** \brief Set base path over which restriction is defined */
            void setBasePath(std::vector<base::State *>);

            /** \brief Return discrete states representation of base path */
            const std::vector<base::State *> &getBasePath() const;

            /** \brief Choose algorithm to find sections over restriction */
            void setFindSectionStrategy(FindSectionType type);

            /** \brief Check if feasible section exists between xStart and xGoal.
             *
             * NOTE:
             *  "const ptr*" means that the pointer itself is const
             *  "ptr* const" means that the content of the pointer is const (but ptr
             *  can change) */
            bool hasFeasibleSection(Configuration *const, Configuration *const);

            /** \brief Return pointer to underlying bundle space graph */
            BundleSpaceGraph *getBundleSpaceGraph();

            /** \brief Length of base path */
            double getLengthBasePath() const;

            /** \brief Return number of discrete states in base path */
            unsigned int size() const;

            /** \brief Return State at index k on base path */
            const base::State *getBaseStateAt(int k) const;

            /** \brief Length between base state indices k and k+1 */
            double getLengthIntermediateBasePath(int k);

            /** \brief Cumulative length until base state index k */
            double getLengthBasePathUntil(int k);

            /** \brief Given a position d in [0, lengthbasepath_], return the
             * index of the nearest state on base path before d */
            int getBasePathLastIndexFromLocation(double d);

            /** \brief Interpolate state on base path at position t in [0,
             * lengthbasepath_] (using discrete state representation) */
            void interpolateBasePath(double t, base::State *&state) const;

            friend std::ostream &operator<<(std::ostream &, const PathRestriction &);

            virtual void print(std::ostream &) const;

        protected:
            /** \brief Pointer to associated bundle space */
            BundleSpaceGraph *bundleSpaceGraph_;

            /** \brief Base path over which we define the restriction */
            std::vector<base::State *> basePath_;

            /** \brief Length of set base path */
            double lengthBasePath_{0.0};

            /** \brief Intermediate lengths between states on base path */
            std::vector<double> lengthsIntermediateBasePath_;

            /** \brief Cumulative lengths between states on base path */
            std::vector<double> lengthsCumulativeBasePath_;

            /** \brief Strategy to find a feasible section (between specific
             * elements on fiber at first base path index and fiber at
             * last base path index)*/
            FindSectionPtr findSection_;
        };
    }
}

#endif
