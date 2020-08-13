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

#ifndef OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_METRICS_BUNDLEMETRIC_SHORTEST_PATH_
#define OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_METRICS_BUNDLEMETRIC_SHORTEST_PATH_
#include <ompl/multilevel/datastructures/metrics/Geodesic.h>

namespace ompl
{
    namespace geometric
    {
        OMPL_CLASS_FORWARD(PathGeometric);
    }
    namespace multilevel
    {
        /** Shortest path Bundle Space Metric: Distance between two elements on
         * bundle space is given by first computing the shortest path between
         * the PROJECTIONS of those elements onto the base space. This is the
         * most obvious way to exploit a base space. However, it might often be
         * too slow for realistic applications. */

        class BundleSpaceMetricShortestPath : public BundleSpaceMetricGeodesic
        {
            using BaseT = BundleSpaceMetricGeodesic;

        public:
            BundleSpaceMetricShortestPath() = delete;
            BundleSpaceMetricShortestPath(BundleSpaceGraph *);
            virtual ~BundleSpaceMetricShortestPath() override;

            virtual double distanceBundle(
                const Configuration *xStart, 
                const Configuration *xDest) override;

            virtual double distanceFiber(
                const Configuration *xStart, 
                const Configuration *xDest) override;

            virtual double distanceBase(
                const Configuration *xStart, 
                const Configuration *xDest) override;

            virtual void interpolateBundle(
                const Configuration *q_from, 
                const Configuration *q_to, 
                const double step,
                Configuration *q_interp) override;

            /** \brief Interpolate path between bundle space elements xStart and
             * xDest by using the base space. First, we project elements down
             * onto base space. Second, we do a graph search on base space graph
             * to connect them. Third, we interpolate an L2 section along the
             * base path. Fourth, we return the milestones along L2 section.*/

            std::vector<const Configuration *> getInterpolationPath(
                const Configuration *xStart,
                const Configuration *xDest);

        protected:
            std::vector<Configuration *> tmpPath_;

            Configuration *xBaseStart_{nullptr};

            Configuration *xBaseDest_{nullptr};
        };
    }
}

#endif
