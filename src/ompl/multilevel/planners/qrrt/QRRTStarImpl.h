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

/* Author: Andreas Orthey, Sohaib Akbar */

#ifndef OMPL_MULTILEVEL_PLANNERS_BundleSpace_QRRTSTARIMPL_
#define OMPL_MULTILEVEL_PLANNERS_BundleSpace_QRRTSTARIMPL_
#include <ompl/multilevel/datastructures/BundleSpaceGraph.h>
#include <ompl/datastructures/PDF.h>

namespace ompl
{
    namespace base
    {
        OMPL_CLASS_FORWARD(OptimizationObjective);
    }
    namespace multilevel
    {
        /** \brief Implementation of BundleSpace Rapidly-Exploring Random Tree Star Algorithm*/
        class QRRTStarImpl : public ompl::multilevel::BundleSpaceGraph
        {
            using BaseT = BundleSpaceGraph;

            void setKNearest(bool useKNearest)
            {
                useKNearest_ = useKNearest;
            }

            bool getKNearest() const
            {
                return useKNearest_;
            }
            virtual bool getSolution(ompl::base::PathPtr &solution) override;
            virtual void getPlannerData(ompl::base::PlannerData &data) const override;

            virtual void clear() override;
            virtual void setup() override;

            void getNearestNeighbors(Configuration *x, std::vector<Configuration *> &nearest);
            void removeFromParent(Configuration *q);
            void calculateRewiringLowerBounds();

        public:
            QRRTStarImpl(const ompl::base::SpaceInformationPtr &si, BundleSpace *parent_);
            virtual ~QRRTStarImpl() override;

            /** \brief One iteration of RRT with adjusted sampling function */
            virtual void grow() override;

            void updateChildCosts(Configuration *q);

            /** \brief a constant value to calculate k */
            double k_rrt_Constant_{0};

            /** \brief a constant value to calculate radius */
            double r_rrt_Constant_{0};

            double rewireFactor_{1.1};

            /** \brief true if cost from a to b is same as b to a*/
            bool symmetric_;

            /** \brief option to use k nn or radius */
            bool useKNearest_{true};

            /** \brief store dimension of bundle space to calc radius */
            double d_{0};

            // /** \brief list of configurations that satisfy the goal condition */
            // std::vector<Configuration *> goalConfigurations_;
        };
    }  // namespace multilevel
}  // namespace ompl

#endif
