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
        OMPL_CLASS_FORWARD(Path);
    }
    namespace geometric
    {
        OMPL_CLASS_FORWARD(PathGeometric);
    }
    namespace multilevel
    {
        OMPL_CLASS_FORWARD(BundleSpaceGraph);
        OMPL_CLASS_FORWARD(PathSection);
        OMPL_CLASS_FORWARD(BasePathHead);
        OMPL_CLASS_FORWARD(FindSection);

        using Configuration = ompl::multilevel::BundleSpaceGraph::Configuration;

        /** \brief Representation of path restriction
            (set of all elements of bundle space
            which project onto a given base path ---
            the union of fibers over a given base path).

            This class has additional functionalities to find path sections
            (paths lying inside path restriction) using different interpolation
            methods (L1 or L2 paths).

            To use this class, you need to set a base path (setBasePath),
            then you can search for sections over this base path 
            (for example using hasFeasibleSection)
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

            const std::vector<base::State*>& getBasePath() const;

            /** \brief Check if feasible section exists between xStart and xGoal.
             * Main method to use outside this class. Use this if you are
             * unsure.
             * NOTE:
             *  "const ptr*" means that the pointer itself is const
             *  "ptr* const" means that the content of the pointer is const (but ptr
             *  can change */
            bool hasFeasibleSection(Configuration *const, Configuration *const);

            /** 
             * Interpolate L1 section recursively and check validity. 
             *
             * Return last valid configuration and call method recursively to sidestep along
             * fiber. Terminate if PATH_SECTION_TREE_MAX_DEPTH recursion were
             * done or we found a feasible section. 
             *
             * @param xStart Bundle space element (start)
             * @param xGoal Bundle space element (goal)
             * @param basePath Current base path (clipped after recursion call)
             * @param interpolateFiberFirst whether to interpolate FF or FL
             * @param depth current recursion depth
             * @param startLength current position along base path
             *
             * @retval True or false if method succeeds
             * */
            bool findSection(
                BasePathHeadPtr& head,
                bool interpolateFiberFirst = true,
                unsigned int depth = 0);

            /** \brief Sample state on fiber while keeping base state fixed */
            bool findFeasibleStateOnFiber(
                const base::State *xBase, 
                base::State *xBundle);

            BundleSpaceGraph* getBundleSpaceGraph();

            /** \brief Length of base path */
            double getLengthBasePath() const;

            int size() const;

            /** \brief Length between base state indices k and k+1 */
            double getLengthIntermediateBasePath(int k);

            /** \brief Cumulative length until base state index k */
            double getLengthBasePathUntil(int k);

            int getBasePathLastIndexFromLocation(double d);

            bool sideStepAlongFiber(
                Configuration* &xOrigin, 
                base::State *state);

            bool tripleStep(
                BasePathHeadPtr& head,
                const base::State *sBundleGoal,
                double locationOnBasePathGoal);

            bool wriggleFree(BasePathHeadPtr& head);

            bool tunneling(BasePathHeadPtr& head);

            void interpolateBasePath(double t, base::State* &state) const;

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

            FindSectionPtr findSection_;

        };
    }
}

#endif
