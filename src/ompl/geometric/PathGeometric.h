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

/* Author: Ioan Sucan */

#ifndef OMPL_GEOMETRIC_PATH_GEOMETRIC_
#define OMPL_GEOMETRIC_PATH_GEOMETRIC_

#include "ompl/base/SpaceInformation.h"
#include "ompl/base/Path.h"
#include <vector>
#include <utility>

namespace ompl
{
    namespace base
    {
        /// @cond IGNORE
        OMPL_CLASS_FORWARD(OptimizationObjective);
        /// @endcond
    }

    /** \brief This namespace contains code that is specific to planning under geometric constraints */
    namespace geometric
    {
        /// @cond IGNORE
        /** \brief Forward declaration of ompl::geometric::PathGeometric */
        OMPL_CLASS_FORWARD(PathGeometric);
        /// @endcond

        /** \brief Definition of a geometric path.

            This is the type of path computed by geometric planners. */
        class PathGeometric : public base::Path
        {
        public:
            /** \brief Construct a path instance for a given space information */
            PathGeometric(const base::SpaceInformationPtr &si) : base::Path(si)
            {
            }

            /** \brief Copy constructor */
            PathGeometric(const PathGeometric &path);

            /** \brief Construct a path instance from a single state */
            PathGeometric(const base::SpaceInformationPtr &si, const base::State *state);

            /** \brief Construct a path instance from two states (thus making a segment) */
            PathGeometric(const base::SpaceInformationPtr &si, const base::State *state1, const base::State *state2);

            ~PathGeometric() override
            {
                freeMemory();
            }

            /** \brief Assignment operator */
            PathGeometric &operator=(const PathGeometric &other);

            /** \brief The sum of the costs for the sequence of segments that make up the path, computed using
                OptimizationObjective::motionCost(). OptimizationObjective::initialCost() and
               OptimizationObjective::terminalCost()
                are also used in the computation for the first and last states, respectively. Empty paths have identity
               cost. */
            base::Cost cost(const base::OptimizationObjectivePtr &obj) const override;

            /** \brief Compute the length of a geometric path (sum of lengths of segments that make up the path) */
            double length() const override;

            /** \brief Check if the path is valid */
            bool check() const override;

            /** \brief Compute a notion of smoothness for this
                path. The closer the value is to 0, the smoother the
                path. Detailed formula follows.

                The idea is to look at the triangles formed by consecutive path segments and compute the angle between
               those segments using
                Pythagora's theorem. Then, the outside angle for the computed angle is normalized by the path segments
               and contributes to the path smoothness.
                For a straight line path, the smoothness will be 0.
                \f[
                    \mbox{smoothness} = \sum\limits_{i=2}^{n-1}\left(\frac{2\left(\pi -
               \arccos\left(\frac{a_i^2+b_i^2-c_i^2}{2 a_i b_i}\right)\right)}{a_i + b_i}\right)^2
                \f]
                where \f$a_i = \mbox{dist}(s_{i-2}, s_{i-1}), b_i = \mbox{dist}(s_{i-1}, s_{i}), c_i =
               \mbox{dist}(s_{i-2}, s_i)\f$, \f$s_i\f$ is the i<sup>th</sup>
                state along the path (see getState()) and \f$\mbox{dist}(s_i, s_j)\f$ gives the distance between two
               states (see ompl::base::StateSpace::distance()).
            */
            double smoothness() const;

            /** \brief Compute the clearance of the way-points along
                the path (no interpolation is performed). Detailed formula follows.

                The formula used for computing clearance is:
                \f[
                    \mbox{clearance} = \frac{1}{n}\sum\limits_{i=0}^{n-1}cl(s_i)
                \f]
                \f$n\f$ is the number of states along the path (see getStateCount())
                \f$s_i\f$ is the i<sup>th</sup> state along the path (see getState())
                \f$cl()\f$ gives the distance to the nearest invalid state for a particular state (see
               ompl::base::StateValidityChecker::clearance())
            */
            double clearance() const;

            /** \brief Print the path to a stream */
            void print(std::ostream &out) const override;

            /** \brief Print the path as a real-valued matrix where the
                i-th row represents the i-th state along the path. Each
                row contains the state components as returned by
                ompl::base::StateSpace::copyToReals. */
            virtual void printAsMatrix(std::ostream &out) const;

            /** @name Path operations
                @{ */

            /** \brief Insert a number of states in a path so that the
                path is made up of exactly \e count states. States are
                inserted uniformly (more states on longer
                segments). Changes are performed only if a path has
                less than \e count states. */
            void interpolate(unsigned int count);

            /** \brief Insert a number of states in a path so that the
                path is made up of (approximately) the states checked
                for validity when a discrete motion validator is
                used. */
            void interpolate();

            /** \brief Add a state at the middle of each segment */
            void subdivide();

            /** \brief Reverse the path */
            void reverse();

            /** \brief Check if the path is valid. If it is not,
                attempts are made to fix the path by sampling around
                invalid states. Not more than \e attempts samples are
                drawn.
                \return A pair of boolean values is returned. The first
                value represents the validity of the path before any
                change was made. The second value represents the
                validity of the path after changes were attempted. If
                no changes are attempted, the both values are true.

                \note If repairing a path fails, the path may still be altered */
            std::pair<bool, bool> checkAndRepair(unsigned int attempts);

            /** \brief Overlay the path \e over on top of the current
                path. States are added to the current path if needed
                (by copying the last state).

                If \e over consists of states form a different
                state space than the existing path, the data from those
                states is copied over, for the corresponding
                components. If \e over is from the same state space as this path,
                and \e startIndex is 0, this function's result will be the same
                as with operator=() */
            void overlay(const PathGeometric &over, unsigned int startIndex = 0);

            /** \brief Append \e state to the end of this path. The memory for \e state is copied. */
            void append(const base::State *state);

            /** \brief Append \e path at the end of this path. States from \e path are copied.

                Let the existing path consist of states [ \e s1, \e
                s2, ..., \e sk ]. Let \e path consist of states [\e y1, ..., \e yp].

                If the existing path and \e path consist of states
                from the same state space, [\e y1, ..., \e yp] are added after \e sk.
                If they are not from the same state space, states [\e z1, ..., \e zp]
                are added, where each \e zi is a copy of \e sk that
                has components overwritten with ones in \e yi (if there are any common subspaces).
            */
            void append(const PathGeometric &path);

            /** \brief Prepend \e state to the start of this path. The memory for \e state is copied. */
            void prepend(const base::State *state);

            /** \brief Keep the part of the path that is after \e state (getClosestIndex() is used to find out which
             * way-point is closest to \e state) */
            void keepAfter(const base::State *state);

            /** \brief Keep the part of the path that is before \e state (getClosestIndex() is used to find out which
             * way-point is closest to \e state) */
            void keepBefore(const base::State *state);

            /** \brief Set this path to a random segment */
            void random();

            /** \brief Set this path to a random valid segment. Sample \e attempts times for valid segments. Returns
             * true on success.*/
            bool randomValid(unsigned int attempts);
            /** @} */

            /** @name Functionality for accessing states
                @{ */

            /** \brief Get the index of the way-point along the path that is closest to \e state. Returns -1 for an
             * empty path. */
            int getClosestIndex(const base::State *state) const;

            /** \brief Get the states that make up the path (as a reference, so it can be modified, hence the function
             * is not const) */
            std::vector<base::State *> &getStates()
            {
                return states_;
            }

            /** \brief Get the state located at \e index along the path */
            base::State *getState(unsigned int index)
            {
                return states_[index];
            }

            /** \brief Get the state located at \e index along the path */
            const base::State *getState(unsigned int index) const
            {
                return states_[index];
            }

            /** \brief Get the number of states (way-points) that make up this path */
            std::size_t getStateCount() const
            {
                return states_.size();
            }

            /** \brief Remove all states and clear memory */
            void clear();

            /** @} */

        protected:
            /** \brief Free the memory corresponding to the states on this path */
            void freeMemory();

            /** \brief Copy data to this path from another path instance */
            void copyFrom(const PathGeometric &other);

            /** \brief The list of states that make up the path */
            std::vector<base::State *> states_;
        };
    }
}

#endif
