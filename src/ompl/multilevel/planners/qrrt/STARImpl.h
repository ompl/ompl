/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, University of Stuttgart
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
 *   * Neither the name of the University of Stuttgart nor the names
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

#ifndef OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_STARIMPL_
#define OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_STARIMPL_
#include <ompl/multilevel/datastructures/BundleSpaceGraph.h>
#include <ompl/multilevel/planners/qrrt/BiQRRTImpl.h>
#include <ompl/multilevel/planners/qrrt/SparseTree.h>
#include <ompl/datastructures/PDF.h>

namespace ompl
{
    namespace base
    {
        OMPL_CLASS_FORWARD(OptimizationObjective);
    }
    namespace multilevel
    {
        class STARImpl : public ompl::multilevel::BundleSpaceGraph
        {
            using BaseT = BundleSpaceGraph;
            using TreeData = std::shared_ptr<NearestNeighbors<Configuration *>>;

        public:
            STARImpl(const ompl::base::SpaceInformationPtr &si, BundleSpace *parent_);
            ~STARImpl() override;

            void grow() override;
            void init() override;
            void setup() override;
            void clear() override;
            void getPlannerData(ompl::base::PlannerData &data) const override;

            void addToTree(SparseTreePtr& tree, Configuration *x);
            // void updateUnsuccessfulExtension(SparseTreePtr& tree, Configuration *x);
            // void updateSuccessfulExtension(SparseTreePtr& tree, Configuration *x);
            // void updateExtension(SparseTreePtr& tree, Configuration *x);
        protected:
            // std::vector<int> treeElement_numberOfExtensions_;
            // std::vector<int> treeElement_numberOfSuccessfulExtensions_;
            // std::vector<int> treeElement_numberOfUnsuccessfulSubsequentExtensions_;
            // std::vector<bool> treeElement_isConverged_;

            SparseTreePtr treeStart_;
            SparseTreePtr treeGoal_;
            double distanceBetweenTrees_;
            bool activeInitialTree_{true};
        };
    }  // namespace multilevel
}  // namespace ompl

#endif
