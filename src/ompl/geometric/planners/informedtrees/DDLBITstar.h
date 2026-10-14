/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2026, Open Motion Planning Library contributors
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
 *   * Neither the name of the copyright holder nor the names of its
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

/* Author: Elias (OMPL contribution) */
#ifndef OMPL_GEOMETRIC_PLANNERS_INFORMEDTREES_DDLBITSTAR_
#define OMPL_GEOMETRIC_PLANNERS_INFORMEDTREES_DDLBITSTAR_

#include "ompl/geometric/planners/informedtrees/BITstar.h"

namespace ompl
{
    namespace geometric
    {
        /**
         * @anchor gDDLBITstar
         * \brief Dual-Density Lock Batch Informed Trees, using BIT*'s search engine.
         *
         * Implements boundary scouting, hybrid Gaussian/informed proposals, source-attributed
         * probabilistic density filtering, and occupancy reconstruction on cost improvement.
         * Supports finite bounded RealVectorStateSpace instances. Just-in-time sampling is unsupported.
         * Spatial parameters are in state-space units; the defaults follow the paper's 500-unit maps.
         * Call clear() before changing DDL parameters on a setup planner.
         *
         * This implementation retains a uniform proposal component and a strictly positive soft-lock
         * probability. BIT*'s connection rules are reused; no additional asymptotic-optimality guarantee
         * is asserted for this nonuniform sampling integration.
         *
         * @par Associated publication:
         * C. Song, Y. Tong, Z. Li, W. Fang, Y. Zhao, "DDL-BIT*: An Improved Batch Informed Trees Algorithm
         * Using Dual-Density Lock for Motion Planning", IEEE Access, 14, 102542-102557, 2026.
         * DOI: https://doi.org/10.1109/ACCESS.2026.3710692
         */
        class DDLBITstar : public BITstar
        {
        public:
            /** \brief Construct a planner using the paper's parameter defaults. */
            explicit DDLBITstar(const base::SpaceInformationPtr &si);
            ~DDLBITstar() override;

            void setup() override;
            void clear() override;
            base::PlannerStatus solve(const base::PlannerTerminationCondition &terminationCondition) override;

            /** \brief Set/get the positive density-grid cell width (default 15). */
            void setGridResolution(double resolution);
            double getGridResolution() const;
            /** \brief Set/get the positive boundary-deduplication cell width (default 2). */
            void setBoundaryResolution(double resolution);
            double getBoundaryResolution() const;
            /** \brief Set/get the positive isotropic Gaussian standard deviation (default 10). */
            void setBoundaryStandardDeviation(double deviation);
            double getBoundaryStandardDeviation() const;
            /** \brief Set/get the boundary proposal probability before a solution, in [0,1) (default 0.2). */
            void setExplorationBias(double bias);
            double getExplorationBias() const;
            /** \brief Set/get the boundary proposal probability after a solution, in [0,1) (default 0.5). */
            void setOptimizationBias(double bias);
            double getOptimizationBias() const;
            /** \brief Set/get the positive soft capacity for uniform proposals (default 1). */
            void setOpenCellCapacity(unsigned int capacity);
            unsigned int getOpenCellCapacity() const;
            /** \brief Set/get the soft capacity for boundary proposals, greater than the open capacity (default 3). */
            void setBoundaryCellCapacity(unsigned int capacity);
            unsigned int getBoundaryCellCapacity() const;
            /** \brief Set/get the positive number of scouting rays per batch (default 18). */
            void setScoutingRays(unsigned int rays);
            unsigned int getScoutingRays() const;

            /** \brief Number of deduplicated boundary points currently stored. */
            std::size_t numBoundaryPoints() const;
            /** \brief Accepted valid Gaussian samples since the last clear(). */
            std::size_t numBoundarySamples() const;
            /** \brief Valid candidates rejected by the soft density filter since the last clear(). */
            std::size_t numDensityRejections() const;
            /** \brief Occupancy reconstructions after cost improvements since the last clear(). */
            std::size_t numGridRebuilds() const;

        protected:
            base::InformedSamplerPtr allocInformedStateSampler(const base::ProblemDefinitionPtr &problem,
                                                               unsigned int maxCalls) override;
            void onNewBatch(const ImplicitGraph &graph) override;

        private:
            void checkConfigurationChange() const;
            struct Impl;
            std::unique_ptr<Impl> impl_;
        };
    }  // namespace geometric
}  // namespace ompl

#endif
