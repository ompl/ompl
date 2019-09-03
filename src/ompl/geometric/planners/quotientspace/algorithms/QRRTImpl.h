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

#ifndef OMPL_GEOMETRIC_PLANNERS_QUOTIENTSPACE_QRRTIMPL_
#define OMPL_GEOMETRIC_PLANNERS_QUOTIENTSPACE_QRRTIMPL_
#include <ompl/geometric/planners/quotientspace/datastructures/QuotientSpaceGraph.h>
#include <ompl/datastructures/PDF.h>

namespace ompl
{
    namespace base
    {
        OMPL_CLASS_FORWARD(OptimizationObjective);
    }
    namespace geometric
    {
        /** \brief Implementation of QuotientSpace Rapidly-Exploring Random Trees Algorithm*/
        class QRRTImpl : public ompl::geometric::QuotientSpaceGraph
        {
            using BaseT = QuotientSpaceGraph;

        public:
            QRRTImpl(const ompl::base::SpaceInformationPtr &si, QuotientSpace *parent_);
            virtual ~QRRTImpl() override;
            /** \brief One iteration of RRT with adjusted sampling function */
            virtual void grow() override;
            virtual bool getSolution(ompl::base::PathPtr &solution) override;
            /** \brief Importance based on how many vertices the tree has */
            double getImportance() const override;
            /** \brief Uniform sampling */
            virtual bool sample(ompl::base::State *q_random) override;
            /** \brief \brief Quotient-Space sampling by choosing a random vertex from parent
                class tree */
            virtual bool sampleQuotient(ompl::base::State *) override;

            virtual void setup() override;
            virtual void clear() override;

            void setGoalBias(double goalBias);
            double getGoalBias() const;
            void setRange(double distance);
            double getRange() const;

        protected:
            /** \brief Random configuration placeholder */
            Configuration *qRandom_{nullptr};
            /** \brief Current shortest path on tree */
            std::vector<Vertex> shortestPathVertices_;

            /** \brief Maximum distance of expanding the tree */
            double maxDistance_{.0};
            /** \brief Goal bias similar to RRT */
            double goalBias_{.05};

            /** \brief Goal state or goal region */
            ompl::base::Goal *goal_;
        };
    }  // namespace geometric
}  // namespace ompl

#endif
