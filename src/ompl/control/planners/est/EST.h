/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2011, Rice University
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

#ifndef OMPL_CONTROL_PLANNERS_EST_EST_
#define OMPL_CONTROL_PLANNERS_EST_EST_

#include "ompl/datastructures/Grid.h"
#include "ompl/control/planners/PlannerIncludes.h"
#include "ompl/base/ProjectionEvaluator.h"
#include "ompl/datastructures/PDF.h"
#include <unordered_map>
#include <vector>

namespace ompl
{
    namespace control
    {
        /**
           @anchor cEST
           @par Short description
           EST is a tree-based motion planner that attempts to detect
           the less explored area of the space through the use of a
           grid imposed on a projection of the state space. Using this
           information, EST continues tree expansion primarily from
           less explored areas.  It is important to set the projection
           the algorithm uses (setProjectionEvaluator() function). If
           no projection is set, the planner will attempt to use the
           default projection associated to the state space. An
           exception is thrown if no default projection is available
           either.
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
            EST(const SpaceInformationPtr &si);

            virtual ~EST();

            virtual base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc);

            virtual void clear();

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
            }

            /** \brief Get the range the planner is using */
            double getRange() const
            {
                return maxDistance_;
            }

            /** \brief Set the projection evaluator. This class is
                able to compute the projection of a given state.  */
            void setProjectionEvaluator(const base::ProjectionEvaluatorPtr &projectionEvaluator)
            {
                projectionEvaluator_ = projectionEvaluator;
            }

            /** \brief Set the projection evaluator (select one from
                the ones registered with the state space). */
            void setProjectionEvaluator(const std::string &name)
            {
                projectionEvaluator_ = si_->getStateSpace()->getProjection(name);
            }

            /** \brief Get the projection evaluator */
            const base::ProjectionEvaluatorPtr &getProjectionEvaluator() const
            {
                return projectionEvaluator_;
            }

            virtual void setup();

            virtual void getPlannerData(base::PlannerData &data) const;

        protected:
            /** \brief Representation of a motion

                This only contains pointers to parent motions as we
                only need to go backwards in the tree. */
            class Motion
            {
            public:
                Motion() : state(nullptr), control(nullptr), steps(0), parent(nullptr)
                {
                }

                /** \brief Constructor that allocates memory for the state and the control */
                Motion(const SpaceInformation *si)
                  : state(si->allocState()), control(si->allocControl()), steps(0), parent(nullptr)
                {
                }

                ~Motion()
                {
                }

                /** \brief The state contained by the motion */
                base::State *state;

                /** \brief The control contained by the motion */
                Control *control;

                /** \brief The number of steps the control is applied for */
                unsigned int steps;

                /** \brief The parent motion in the exploration tree */
                Motion *parent;
            };

            struct MotionInfo;

            /** \brief A grid cell */
            typedef Grid<MotionInfo>::Cell GridCell;

            /** \brief A PDF of grid cells */
            typedef PDF<GridCell *> CellPDF;

            /** \brief A struct containing an array of motions and a corresponding PDF element */
            struct MotionInfo
            {
                Motion *operator[](unsigned int i)
                {
                    return motions_[i];
                }
                const Motion *operator[](unsigned int i) const
                {
                    return motions_[i];
                }
                void push_back(Motion *m)
                {
                    motions_.push_back(m);
                }
                unsigned int size() const
                {
                    return motions_.size();
                }
                bool empty() const
                {
                    return motions_.empty();
                }
                std::vector<Motion *> motions_;
                CellPDF::Element *elem_;
            };

            /** \brief The data contained by a tree of exploration */
            struct TreeData
            {
                TreeData() : grid(0), size(0)
                {
                }

                /** \brief A grid where each cell contains an array of motions */
                Grid<MotionInfo> grid;

                /** \brief The total number of motions in the grid */
                unsigned int size;
            };

            /** \brief Free the memory allocated by this planner */
            void freeMemory();

            /** \brief Add a motion to the exploration tree */
            void addMotion(Motion *motion);

            /** \brief Select a motion to continue the expansion of the tree from */
            Motion *selectMotion();

            /** \brief Valid state sampler */
            base::ValidStateSamplerPtr sampler_;

            /** \brief Directed control sampler */
            DirectedControlSamplerPtr controlSampler_;

            /** \brief The base::SpaceInformation cast as control::SpaceInformation, for convenience */
            const SpaceInformation *siC_;

            /** \brief The exploration tree constructed by this algorithm */
            TreeData tree_;

            /** \brief This algorithm uses a discretization (a grid) to guide the exploration. The exploration is
             * imposed on a projection of the state space. */
            base::ProjectionEvaluatorPtr projectionEvaluator_;

            /** \brief The fraction of time the goal is picked as the state to expand towards (if such a state is
             * available) */
            double goalBias_;

            /** \brief The maximum length of a motion to be added to a tree */
            double maxDistance_;

            /** \brief The random number generator */
            RNG rng_;

            /** \brief The PDF used for selecting a cell from which to sample a motion */
            CellPDF pdf_;

            /** \brief The most recent goal motion.  Used for PlannerData computation */
            Motion *lastGoalMotion_;
        };
    }
}

#endif
