/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, Rice University
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
 *   * Neither the name of Rice University nor the names of its
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

#ifndef OMPL_GEOMETRIC_PLANNERS_RRT_TSRRT_
#define OMPL_GEOMETRIC_PLANNERS_RRT_TSRRT_

#include "ompl/base/ProjectionEvaluator.h"
#include "ompl/geometric/planners/PlannerIncludes.h"
#include "ompl/datastructures/NearestNeighbors.h"

namespace ompl
{
    namespace geometric
    {
        OMPL_CLASS_FORWARD(TaskSpaceConfig);

        class TaskSpaceConfig
        {
        public:
            virtual ~TaskSpaceConfig()
            {
            }
            // Returns the dimension of the task space.
            virtual int getDimension() const = 0;
            // Project the c-space state into the task-space.
            virtual void project(const base::State *state, Eigen::Ref<Eigen::VectorXd> ts_proj) const = 0;

            // Sample point uniformly in task-space.
            virtual void sample(Eigen::Ref<Eigen::VectorXd> ts_proj) const = 0;

            // Given a point in task-space, generate a configuraton space state
            // that projects to this point.  seed is the nearest task-space neighbor.
            virtual bool lift(const Eigen::Ref<Eigen::VectorXd> &ts_proj, const base::State *seed,
                              base::State *state) const = 0;
        };

        /**
        \anchor gTSRRT

        \ref gTSRRT "Task-space Rapidly-exploring Random Trees (TSRRT)" is a variant of RRT where exploration is guided
        by the task space. It requires an ompl::geometric::TaskSpaceConfig instance that defines how to project
        configuration space states to the task spaces and an inverse operation to lift task space states to the
        configuration space.

        \par Associated publication:
        A. Shkolnik and R. Tedrake, “Path planning in 1000+ dimensions using a task-space voronoi bias,” in IEEE Intl.
        Conf. on Robotics and Automation, pp. 2061–2067, 2009. DOI:
        [10.1109/ROBOT.2009.5152638](http://dx.doi.org/10.1109/ROBOT.2009.5152638)<br>
        [[PDF]](https://groups.csail.mit.edu/robotics-center/public_papers/Shkolnik09.pdf)

        **/

        /** \brief Task-space Rapidly-exploring Random Trees */
        class TSRRT : public base::Planner
        {
        public:
            /** \brief Constructor */
            TSRRT(const base::SpaceInformationPtr &si, const TaskSpaceConfigPtr &task_space);

            virtual ~TSRRT();

            virtual void getPlannerData(base::PlannerData &data) const;

            virtual base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc);

            virtual void clear();

            /** \brief Set the goal bias

                In the process of randomly selecting states in
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

            /** \brief Set a different nearest neighbors datastructure */
            template <template <typename T> class NN>
            void setNearestNeighbors()
            {
                nn_.reset(new NN<Motion *>());
            }

            virtual void setup();

        protected:
            /** \brief Representation of a motion

                This only contains pointers to parent motions as we
                only need to go backwards in the tree. */
            class Motion
            {
            public:
                /** \brief Constructor that allocates memory for the state */
                Motion(const base::SpaceInformationPtr &si) : state(si->allocState()), parent(nullptr)
                {
                }

                ~Motion() = default;

                /** \brief The state contained by the motion */
                base::State *state{nullptr};

                /** \brief The parent motion in the exploration tree */
                Motion *parent{nullptr};

                // Projection of the state into the task space.
                Eigen::VectorXd proj;
            };

            /** \brief Free the memory allocated by this planner */
            void freeMemory();

            /** \brief Compute distance between motions (actually distance between contained states) */
            double distanceFunction(const Motion *a, const Motion *b) const
            {
                double sqr_dist = 0.0;
                for (int ix = 0; ix < a->proj.size(); ++ix)
                {
                    double sqr_val = (*b).proj[ix] - (*a).proj[ix];
                    sqr_val *= sqr_val;

                    sqr_dist += sqr_val;
                }
                return sqr_dist;
            }

            /** \brief A nearest-neighbors datastructure containing the tree of motions */
            std::shared_ptr<NearestNeighbors<Motion *>> nn_;

            /** \brief The fraction of time the goal is picked as the state to expand towards (if such a state is
             * available) */
            double goalBias_{0.05};

            /** \brief The maximum length of a motion to be added to a tree */
            double maxDistance_{0.};

            /** \brief The random number generator */
            RNG rng_;

            /** \brief The most recent goal motion.  Used for PlannerData computation */
            Motion *lastGoalMotion_{nullptr};

            // Mapping to/from task space.
            TaskSpaceConfigPtr task_space_;
        };

    }  // namespace geometric
}  // namespace ompl

#endif
