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

#ifndef OMPL_GEOMETRIC_PLANNERS_QUOTIENTSPACE_QRRT_
#define OMPL_GEOMETRIC_PLANNERS_QUOTIENTSPACE_QRRT_
#include "QuotientGraph.h"
#include <ompl/datastructures/PDF.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace ompl
{
    namespace base
    {
        OMPL_CLASS_FORWARD(OptimizationObjective);
    }
    namespace geometric
    {
        /**
             @anchor QRRT
             @par Short description
             QRRT is a planner using different abstractions levels, each described by
             a quotient-space, and grows trees both sequentially and simultaneously on
             them. The growing of each tree is similar to the RRT algorithm, but it
             differs that (1) a tree is only started if there exists a solution on a
             lower-dimensional quotient-space, and (2) a sample is not drawn
             uniformly, but constraint to the tree of the lower-dimensional
             quotient-space. The algorithm stops if a planner terminate condition (ptc) is
             reached, or if a solution has been found on the last quotient-space,
             which is equivalent to the configuration space.
             @par External documentation
             A. Orthey and M. Toussaint,
             Rapidly-Exploring Quotient-Space Trees: Motion Planning using Sequential Simplifications,
             [[PDF]]()
             A. Orthey and A. Escande and E. Yoshida,
             Quotient-Space Motion Planning,
             <em>International Conference on Robotics and Intelligent Systems</em>, 2018
             [[PDF]](https://arxiv.org/pdf/1807.09468.pdf)
        */

        /** \brief Rapidly Exploring Quotient-Space Tree Algorithm*/
        class QRRT: public og::QuotientGraph
        {

        typedef og::QuotientGraph BaseT;
        public:

            QRRT(const ob::SpaceInformationPtr &si, Quotient *parent_);
            virtual ~QRRT() override;
            /// One iteration of RRT with adjusted sampling function
            virtual void grow() override;
            virtual bool getSolution(ob::PathPtr &solution) override;
            /// Importance based on how many vertices the tree has
            double getImportance() const override;
            /// Uniform sampling
            virtual bool sample(ob::State *q_random) override;
            /// \brief Quotient-Space sampling by choosing a random vertex from parent
            /// class tree
            virtual bool sampleQuotient(ob::State*) override;

            virtual void setup() override;
            virtual void clear() override;

            void setGoalBias(double goalBias);
            double getGoalBias() const;
            void setRange(double distance);
            double getRange() const;

        protected:

            /// Random configuration placeholder
            Configuration *qRandom_{nullptr};
            /// Current shortest path on tree
            std::vector<Vertex> shortestPathVertices_;

            /// Maximum distance of expanding the tree
            double maxDistance_{.0};
            /// Goal bias similar to RRT
            double goalBias_{.05};

            /// Goal state or goal region
            ob::Goal *goal_;
        };

    }
}

#endif
