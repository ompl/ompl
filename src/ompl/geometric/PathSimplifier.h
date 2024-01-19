/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
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
 *   * Neither the name of the Willow Garage nor the names of its
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

/* Author: Ioan Sucan, Ryan Luna */

#ifndef OMPL_GEOMETRIC_PATH_SIMPLIFIER_
#define OMPL_GEOMETRIC_PATH_SIMPLIFIER_

#include "ompl/base/SpaceInformation.h"
#include "ompl/geometric/PathGeometric.h"
#include "ompl/base/PlannerTerminationCondition.h"
#include "ompl/base/OptimizationObjective.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/util/ClassForward.h"
#include "ompl/util/RandomNumbers.h"
#include "ompl/util/Console.h"
#include <limits>

namespace ompl
{
    namespace geometric
    {
        /// @cond IGNORE
        /** \brief Forward declaration of ompl::geometric::PathSimplifier */
        OMPL_CLASS_FORWARD(PathSimplifier);
        /// @endcond

        /** \class ompl::geometric::PathSimplifierPtr
            \brief A shared pointer wrapper for ompl::geometric::PathSimplifier */

        /** \brief This class contains routines that attempt to simplify geometric paths.

            Some of these are in fact routines that shorten the path, and do not necessarily make it smoother. */
        class PathSimplifier
        {
        public:
            /** \brief Create an instance for a specified space information. Optionally, a GoalSampleableRegion may be
            passed in to attempt improvements at the end of the path as well. */
            PathSimplifier(base::SpaceInformationPtr si, const base::GoalPtr &goal = ompl::base::GoalPtr(),
                           const base::OptimizationObjectivePtr &obj = nullptr);

            virtual ~PathSimplifier() = default;

            /** \brief Given a path, attempt to remove vertices from it while keeping the path valid. This is an
                iterative process that attempts to do "short-cutting" on the path. Connection is attempted between
                non-consecutive way-points on the path. If the connection is successful, the path is shortened by
                removing the in-between way-points. This function returns true if changes were made to the path.

                \param path the path to reduce vertices from

                \param maxSteps the maximum number of attempts to "short-cut" the path. If this value is set to 0 (the
                default), the number of attempts made is equal to the number of states in \e path.

                \param maxEmptySteps not all iterations of this function produce a simplification. If an iteration does
                not produce a simplification, it is called an empty step. \e maxEmptySteps denotes the maximum number of
                consecutive empty steps before the simplification process terminates. If this value is set to 0 (the
                default), the number of attempts made is equal to the number of states in \e path.

                \param rangeRatio the maximum distance between states a connection is attempted, as a fraction relative
                to the total number of states (between 0 and 1).

            */
            bool reduceVertices(PathGeometric &path, unsigned int maxSteps = 0, unsigned int maxEmptySteps = 0,
                                double rangeRatio = 0.33);

            /** \brief Given a path, attempt to shorten it while maintaining its validity. This is an iterative process
                that attempts to do "short-cutting" on the path. Connection is attempted in a deterministic order
               between non-consecutive states, considering the furthest states first. Unlike the reduceVertices()
               function, this function does not sample only vertices produced by the planner, but intermediate points on
               the path. If the connection is successful, the path is shortened by removing the in-between states (and
               new vertices are created on the new segment). This function returns true if changes were made to the
               path. Unlike the partialShortcutPath() function, this function uses a deterministic order of connection
               attempts, which makes it more efficient. This function uses the optimization process of RRT-Rope and
               works well with a path produced with RRTConnect.
                @par External documentation
                L. Petit and A. L. Desbiens, RRT-Rope: A deterministic shortening approach for fast near-optimal path
               planning in large-scale uncluttered 3D environments, in <em>2021 IEEE International Conference on
               Systems, Man, and Cybernetics (SMC)</em>, Melbourne, Australia, 2021, pp. 1111-1118. DOI:
                [10.1109/SMC52423.2021.9659071](http://dx.doi.org/10.1109/SMC52423.2021.9659071)<br>
                [[PDF]](https://www.researchgate.net/publication/357636884_RRT-Rope_A_deterministic_shortening_approach_for_fast_near-optimal_path_planning_in_large-scale_uncluttered_3D_environments)
                [[more]](https://www.edu.louispetit.be/rrt-rope)

                \param path the path to shorten

                \param delta the step size between two consecutive states on the path. This parameter also influences
                the runtime of the algorithm. See the RRT-Rope paper for more details. The default value is 1.0.

                \param equivalenceTolerance the tolerance used to determine if a path segment cost is equivalent to the
               optimal shortcut segment cost. This parameter is relative to delta. For example, if equivalenceTolerance
               is 0.1, then two segments are considered equivalent if they are within 10% of delta of each other. The
               default value is 0.1.

                \note This function assumes that improvements are only made within the convex hull of the path. If the
                triangle inequality does not holds for the optimization objective, this will not perform well without
                being run with conjunction with perturbPath.
            */
            bool ropeShortcutPath(PathGeometric &path, double delta = 1.0, double equivalenceTolerance = 0.1);

            /** \brief Given a path, attempt to shorten it while maintaining its validity. This is an iterative process
                that attempts to do "short-cutting" on the path. Connection is attempted between random points along the
                path segments. Unlike the reduceVertices() function, this function does not sample only vertices
                produced by the planner, but intermediate points on the path. If the connection is successful, the path
                is shortened by removing the in-between states (and new vertices are created when needed). This function
                returns true if changes were made to the path. This function uses Partial-Shortcut. This may lead to
               irrelevant shortcuts (for either a portion of path that is already straight, or for a portion of path
               that is intended to be pruned in the future). Also, the undeterministic approach does not provide a clear
               point at which the shortcutting process is finished, besides the max number of steps set by the user.
               Setting maxSteps to infinity will produce the same path as ropeShortcutPath, but with a longer runtime.
                If \e maxSteps is large, ropeShortcutPath will statistically produce an equal or shorter path, in a
               shorter computation time. If \e maxSteps is small, ropeShortcutPath will statistically produce a shorter
               path.
                @par External documentation
                R. Geraerts and M. H. Overmars, Clearance based path optimizationfor motion planning,
                in <em>IEEE International Conference on Robotics and Automation</em>, 2004, vol. 3, pp. 2386–2392.

                \param path the path to reduce vertices from

                \param maxSteps the maximum number of attempts to "short-cut" the path. If this value is set to 0 (the
                default), the number of attempts made is equal to the number of states in \e path.

                \param maxEmptySteps not all iterations of this function produce a simplification. If an iteration does
                not produce a simplification, it is called an empty step. \e maxEmptySteps denotes the maximum number of
                consecutive empty steps before the simplification process terminates. If this value is set to 0 (the
                default), the number of attempts made is equal to the number of states in \e path.

                \param rangeRatio the maximum distance between states a connection is attempted, as a fraction relative
                to the total length of the path (between 0 and 1).

                \param snapToVertex While sampling random points on the path, sometimes the points may be close to
                vertices on the original path (way-points). This function will then "snap to the near vertex", if the
                distance is less than \e snapToVertex fraction of the total path length. This should usually be a small
                value (e.g., one percent of the total path length: 0.01; the default is half a percent)

                \note This function assumes that improvements are only made within the convex hull of the path. If the
                triangle inequality does not holds for the optimization objective, this will not perform well without
                being run with conjunction with perturbPath.
            */
            bool partialShortcutPath(PathGeometric &path, unsigned int maxSteps = 0, unsigned int maxEmptySteps = 0,
                                     double rangeRatio = 0.33, double snapToVertex = 0.005);

            /** \brief Given a path, attempt to improve the cost by randomly perturbing a randomly selected point on
                the path. This is an iterative process that should ideally be run in conjunction with
               partialShortcutPath. This function is not called by any of the 'simplify*' funcions because it is only
               effective when used with a non-metric cost. The default cost used is path length, on which perturbPath is
               not as performant. This function returns true if changes were make to the path.

                \param path the path to reduce vertices from

                \param stepSize the size of the perturbations, i.e. the distance from a randomly selected configuration
                and its position after perturbation. Also used to determine how far the points on either side of the
                selected configuration are (stepSize / 2).

                \param maxSteps the maximum number of attemps to perturb the path. If this value is set to 0 (the
               default), the number of attempts made is equal to the sumber of of states in the \e path (not suggested).

                \param maxEmptySteps not all iterations of this function produce an improvement. If an iteration does
                not produce an improvement, it is called an empty step. \e maxEmptySteps denotes the maximum
                number of consecutive empty steps before the simplification process terminates.

                \param snapToVertex While sampling random points on the path, sometimes the points may be close to
                vertices on the original path (way-points). This function will then "snap to the near vertex", if the
                distance is less than \e snapToVertex fraction of the total path length. This should usually be a small
                value (e.g., one percent of the total path length: 0.01; the default is half a percent)

                @par External Documentation

                J. Mainprice, E. Sisbot, L. Jaillet, J. Cortes, R. Alami, T. Simeon
                Planning human-aware motions using a sampling-based costmap planner,
                <em>Robotics and Automation (ICRA)</em>, August 2011.
                DOI: [10.1109/ICRA.2011.5980048](http://dx.doi.org/10.1109/ICRA.2011.5980048)<br>
                [[PDF]](http://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=5980048)
            */
            bool perturbPath(PathGeometric &path, double stepSize, unsigned int maxSteps = 0,
                             unsigned int maxEmptySteps = 0, double snapToVertex = 0.005);

            /** \brief Given a path, attempt to remove vertices from it while keeping the path valid. This is an
                iterative process that attempts to do "short-cutting" on the path. Connection is attempted between
                non-consecutive states that are close along the path. If the connection is successful, the path is
                shortened by removing the in-between states. This function returns true if changes were made to the
                path.

                \param path the path to reduce vertices from

                \param maxSteps the maximum number of attempts to "short-cut" the path. If this value is set to 0 (the
                default), the number of attempts made is equal to the number of states in \e path.

                \param maxEmptySteps not all iterations of this function produce a simplification. If an iteration does
                not produce a simplification, it is called an empty step. \e maxEmptySteps denotes the maximum number of
                consecutive empty steps before the simplification process terminates.
            */
            bool collapseCloseVertices(PathGeometric &path, unsigned int maxSteps = 0, unsigned int maxEmptySteps = 0);

            /** \brief Given a path, attempt to smooth it (the validity of the path is maintained).

                This function applies \e maxSteps steps of smoothing with B-Splines. Fewer steps are applied if no
                progress is detected: states are either not updated or their update is smaller than \e minChange. At
                each step the path is subdivided and states along it are updated such that the smoothness is improved.

                \note This function may significantly increase the number of states along the solution path. \note This
                function assumes the triangle inequality holds and should not be run on non-metric spaces.
                */
            void smoothBSpline(PathGeometric &path, unsigned int maxSteps = 5,
                               double minChange = std::numeric_limits<double>::epsilon());

            /** \brief Given a path, attempt to remove vertices from it while keeping the path valid. Then, try to
               smooth the path. This function applies the same set of default operations to the path, except in
               non-metric spaces, with the intention of simplifying it. In non-metric spaces, some operations are
               skipped because they do not work correctly when the triangle inequality may not hold. Return
               \e false iff the simplified path is not valid. */
            bool simplifyMax(PathGeometric &path);

            /** \brief Run simplification algorithms on the path for at most \e maxTime seconds, and at least once if \e
               atLeastOnce. Return false iff the simplified path is not valid. */
            bool simplify(PathGeometric &path, double maxTime, bool atLeastOnce = true);

            /** \brief Run simplification algorithms on the path as long as the termination condition does not become
               true, and at least once if \e atLeastOnce. Return \e false iff the simplified path is not valid. */
            bool simplify(PathGeometric &path, const base::PlannerTerminationCondition &ptc, bool atLeastOnce = true);

            /** \brief Attempt to improve the solution path by sampling a new goal state and connecting this state to
                the solution path for at most \e maxTime seconds.

                \param sampingAttempts The maximum number of attempts to connect a candidate goal state to a part of \e
                path

                \param rangeRatio The fraction of the end of the path to consider for connection to a candidate goal
                state, in (0,1].

                \param snapToVertex The percentage of the total path length to consider as "close enough" to an existing
                state in the path for the method to "snap" the connection to that particular state. This prevents states
                in the path that are very close to each other.
            */
            bool findBetterGoal(PathGeometric &path, double maxTime, unsigned int samplingAttempts = 10,
                                double rangeRatio = 0.33, double snapToVertex = 0.005);

            /** \brief Attempt to improve the solution path by sampling a new goal state and connecting this state to
                the solution path while the termination condition is not met.

                \param sampingAttempts The maximum number of attempts to connect a candidate goal state to a part of \e
                path

                \param rangeRatio The fraction of the end of the path to consider for connection to a candidate goal
                state, in (0,1].

                \param snapToVertex The percentage of the total path length to consider as "close enough" to an existing
                state in the path for the method to "snap" the connection to that particular state. This prevents states
                in the path that are very close to each other.
            */
            bool findBetterGoal(PathGeometric &path, const base::PlannerTerminationCondition &ptc,
                                unsigned int samplingAttempts = 10, double rangeRatio = 0.33,
                                double snapToVertex = 0.005);

            /** \brief Set this flag to false to avoid freeing the memory allocated for states that are removed from a
               path during simplification. Setting this to true makes this free memory. Memory is freed by default (flag
               is true by default) */
            void freeStates(bool flag);

            /** \brief Return true if the memory of states is freed when they are removed from a path during
             * simplification */
            bool freeStates() const;

        protected:
            int selectAlongPath(std::vector<double> dists, std::vector<base::State *> states, double distTo,
                                double threshold, base::State *select_state, int &pos);

            /** \brief The space information this path simplifier uses */
            base::SpaceInformationPtr si_;

            /** \brief The goal object for the path simplifier.  Used for end-of-path improvements */
            std::shared_ptr<base::GoalSampleableRegion> gsr_;

            /** \brief The optimization objective to use when making improvements. Will be used on all methods except
                reduce vertices (whose goal is not necessary to improve the solution). */
            base::OptimizationObjectivePtr obj_;

            /** \brief Flag indicating whether the states removed from a motion should be freed */
            bool freeStates_;

            /** \brief Instance of random number generator */
            RNG rng_;
        };
    }  // namespace geometric
}  // namespace ompl

#endif
