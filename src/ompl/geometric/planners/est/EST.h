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

#ifndef OMPL_GEOMETRIC_PLANNERS_EST_EST_
#define OMPL_GEOMETRIC_PLANNERS_EST_EST_

#include "ompl/geometric/planners/PlannerIncludes.h"
#include "ompl/datastructures/NearestNeighbors.h"
#include "ompl/datastructures/PDF.h"
#include <vector>

namespace ompl
{
    namespace geometric
    {
        /**
           @anchor gEST
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

        /** \brief Expansive Space Trees */
        class EST : public base::Planner
        {
        public:
            /** \brief Constructor */
            EST(const base::SpaceInformationPtr &si);

            ~EST() override;

            base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;

            void clear() override;

            /** \brief In the process of randomly selecting states in
                the state space to attempt to go towards, the
                algorithm may in fact choose the actual goal state, if
                it knows it, with some probability. This probability
                is a real number between 0.0 and 1.0; its value should
                usually be around 0.05 and should not be too large. It
                is probably a good idea to use the default value. */
            void setGoalBias(double goalBias)
            {
                goalBias_ = goalBias;
            }

            /** \brief Get the goal bias the planner is using */
            double getGoalBias() const
            {
                return goalBias_;
            }

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
                Motion(const base::SpaceInformationPtr &si) : state(si->allocState())
                {
                }

                ~Motion() = default;

                /// \brief The state contained by the motion
                base::State *state{nullptr};

                /// \brief The parent motion in the exploration tree
                Motion *parent{nullptr};

                /// \brief A pointer to the corresponding element in the probability distribution function
                PDF<Motion *>::Element *element{nullptr};
            };

            /// \brief Compute distance between motions (actually distance between contained states)
            double distanceFunction(const Motion *a, const Motion *b) const
            {
                return si_->distance(a->state, b->state);
            }

            /// \brief A nearest-neighbors datastructure containing the tree of motions
            std::shared_ptr<NearestNeighbors<Motion *>> nn_;

            /// \brief The set of all states in the tree
            std::vector<Motion *> motions_;

            /// \brief The probability distribution function over states in the tree
            PDF<Motion *> pdf_;

            ///\brief Free the memory allocated by this planner
            void freeMemory();

            /// \brief Add a motion to the exploration tree
            void addMotion(Motion *motion, const std::vector<Motion *> &neighbors);

            /// \brief Valid state sampler
            base::ValidStateSamplerPtr sampler_;

            /// \brief The fraction of time the goal is picked as the state to expand towards (if such a state is
            /// available)
            double goalBias_{0.5};

            /// \brief The maximum length of a motion to be added to a tree
            double maxDistance_{0.};

            /// \brief The radius considered for neighborhood
            double nbrhoodRadius_;

            /** \brief The random number generator */
            RNG rng_;

            /// \brief The most recent goal motion.  Used for PlannerData computation
            Motion *lastGoalMotion_{nullptr};
        };
    }
}

#endif
