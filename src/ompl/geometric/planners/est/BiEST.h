/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2015, Rice University
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
*   * Neither the name of the Rice University nor the names of its
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

/* Author: Ryan Luna */

#ifndef OMPL_GEOMETRIC_PLANNERS_EST_BIEST_
#define OMPL_GEOMETRIC_PLANNERS_EST_BIEST_

#include "ompl/geometric/planners/PlannerIncludes.h"
#include "ompl/datastructures/NearestNeighbors.h"
#include "ompl/datastructures/PDF.h"
#include <vector>

namespace ompl
{
    namespace geometric
    {
        /**
           @anchor gBiEST
           @par Short description
           EST is a tree-based motion planner that attempts to detect
           the less explored area of the space by measuring the density
           of the explored space, biasing exploration toward parts of
           the space with lowest density.
           @par External documentation
           D. Hsu, J.-C. Latombe, and R. Motwani, Path planning in expansive configuration spaces,
           <em>Intl. J. Computational Geometry and Applications</em>,
           vol. 9, no. 4-5, pp. 495–512, 1999. DOI:
           [10.1142/S0218195999000285](http://dx.doi.org/10.1142/S0218195999000285)<br>
           [[PDF]](http://bigbird.comp.nus.edu.sg/pmwiki/farm/motion/uploads/Site/ijcga96.pdf)
        */

        /** \brief Bi-directional Expansive Space Trees */
        class BiEST : public base::Planner
        {
        public:
            /** \brief Constructor */
            BiEST(const base::SpaceInformationPtr &si);

            ~BiEST() override;

            base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;

            void clear() override;

            /** \brief Set the range the planner is supposed to use.

                This parameter greatly influences the runtime of the
                algorithm. It represents the maximum length of a
                motion to be added in the tree of motions. */
            void setRange(double distance)
            {
                maxDistance_ = distance;

                // Make the neighborhood radius smaller than sampling range to
                // keep probabilities relatively high for rejection sampling
                nbrhoodRadius_ = maxDistance_ / 3.0;
            }

            /** \brief Get the range the planner is using */
            double getRange() const
            {
                return maxDistance_;
            }

            void setup() override;

            void getPlannerData(base::PlannerData &data) const override;

        protected:
            /// \brief The definition of a motion
            class Motion
            {
            public:
                Motion() = default;

                /// \brief Constructor that allocates memory for the state
                Motion(const base::SpaceInformationPtr &si)
                  : state(si->allocState())
                {
                }

                ~Motion() = default;

                /// \brief The state contained by the motion
                base::State *state{nullptr};

                /// \brief The parent motion in the exploration tree
                Motion *parent{nullptr};

                /// \brief A pointer to the corresponding element in the probability distribution function
                PDF<Motion *>::Element *element{nullptr};

                /// \brief The root node of the tree this motion is in
                const base::State *root{nullptr};
            };

            /// \brief Compute distance between motions (actually distance between contained states)
            double distanceFunction(const Motion *a, const Motion *b) const
            {
                return si_->distance(a->state, b->state);
            }

            /// \brief A nearest-neighbors datastructure containing the tree of motions
            std::shared_ptr<NearestNeighbors<Motion *>> nnStart_;
            std::shared_ptr<NearestNeighbors<Motion *>> nnGoal_;

            /// \brief The set of all states in the start tree
            std::vector<Motion *> startMotions_;
            std::vector<Motion *> goalMotions_;

            /// \brief The probability distribution function over states in each tree
            PDF<Motion *> startPdf_;
            PDF<Motion *> goalPdf_;

            ///\brief Free the memory allocated by this planner
            void freeMemory();

            /// \brief Add a motion to the exploration tree
            void addMotion(Motion *motion, std::vector<Motion *> &motions, PDF<Motion *> &pdf,
                           const std::shared_ptr<NearestNeighbors<Motion *>> &nn,
                           const std::vector<Motion *> &neighbors);

            /// \brief Valid state sampler
            base::ValidStateSamplerPtr sampler_;

            /// \brief The maximum length of a motion to be added to a tree
            double maxDistance_{0.0};

            /// \brief The radius considered for neighborhood
            double nbrhoodRadius_;

            /// \brief The random number generator
            RNG rng_;

            /// \brief The pair of states in each tree connected during planning.  Used for PlannerData computation
            std::pair<base::State *, base::State *> connectionPoint_{nullptr,nullptr};
        };
    }
}

#endif
