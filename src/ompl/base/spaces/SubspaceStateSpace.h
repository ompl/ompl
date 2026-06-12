/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2026, Rice University and National University of Singapore.
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
 *   * Neither the name of the copyright holders nor the names of its
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

#ifndef OMPL_BASE_SPACES_SUBSPACE_STATE_SPACE_
#define OMPL_BASE_SPACES_SUBSPACE_STATE_SPACE_

#include "ompl/base/spaces/RealVectorBounds.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "ompl/util/Exception.h"

#include <algorithm>
#include <cstddef>
#include <string>
#include <utility>
#include <vector>

namespace ompl
{
    namespace base
    {
        /** \brief A real-vector state space that represents a subset of the
         * dimensions of a larger ambient real-vector space.
         *
         * The subspace has dimension \c active_indices.size(); each active
         * index identifies one dimension of the ambient space that the
         * planner is allowed to vary. The remaining dimensions stay pinned
         * to the values supplied in \c frozen_values.
         *
         * Because the class is just a RealVectorStateSpace with extra
         * bookkeeping, every OMPL planner that consumes a real-vector space
         * works on it directly, and it can be wrapped by ConstrainedStateSpace
         * or ProjectedStateSpace for manifold-constrained planning. State
         * validity checkers and motion validators that bridge to backends
         * operating on the ambient configuration (e.g. VAMP, FCL, MoveIt)
         * read the active indices and frozen values from this space and use
         * \c expandToFull to lift a reduced-DOF state back to the ambient
         * configuration before running their checks.
         *
         * Sample use:
         * \code
         * RealVectorBounds full(7);                       // ambient bounds
         * full.setLow(-3.14); full.setHigh(3.14);
         * std::vector<std::size_t> active{3, 4, 5, 6};    // distal 4 joints
         * std::vector<double> frozen{0, -0.785, 0, -2.356, 0, 1.571, 0.785};
         * auto subspace = std::make_shared<SubspaceStateSpace>(full, active, frozen);
         * \endcode
         */
        class SubspaceStateSpace : public RealVectorStateSpace
        {
        public:
            /** \brief Construct a subspace.
             *
             * \param ambient_bounds Bounds of the full ambient real-vector
             *        space. Determines the ambient dimension via
             *        \c ambient_bounds.low.size().
             * \param active_indices Indices into the ambient vector that
             *        the planner is allowed to vary. Must be non-empty,
             *        unique, and each entry must be < ambient dimension.
             *        Order is preserved and defines the layout of the
             *        subspace state (state[i] corresponds to
             *        ambient[active_indices[i]]).
             * \param frozen_values Values to use for ambient dimensions not
             *        in \c active_indices. Must have length equal to the
             *        ambient dimension. Entries at active indices are
             *        ignored.
             */
            SubspaceStateSpace(const RealVectorBounds &ambient_bounds, std::vector<std::size_t> active_indices,
                               std::vector<double> frozen_values)
              : RealVectorStateSpace(active_indices.size())
              , ambient_dim_(ambient_bounds.low.size())
              , active_indices_(std::move(active_indices))
              , frozen_values_(std::move(frozen_values))
            {
                if (active_indices_.empty())
                {
                    throw Exception("SubspaceStateSpace: active_indices must be non-empty");
                }
                if (frozen_values_.size() != ambient_dim_)
                {
                    throw Exception("SubspaceStateSpace: frozen_values length must equal the ambient dimension");
                }

                std::vector<std::size_t> sorted(active_indices_);
                std::sort(sorted.begin(), sorted.end());
                if (std::adjacent_find(sorted.begin(), sorted.end()) != sorted.end())
                {
                    throw Exception("SubspaceStateSpace: active_indices contains duplicates");
                }
                if (sorted.back() >= ambient_dim_)
                {
                    throw Exception("SubspaceStateSpace: active_indices contains an out-of-range entry");
                }

                RealVectorBounds projected(active_indices_.size());
                for (std::size_t i = 0; i < active_indices_.size(); ++i)
                {
                    const auto idx = active_indices_[i];
                    projected.setLow(i, ambient_bounds.low[idx]);
                    projected.setHigh(i, ambient_bounds.high[idx]);
                }
                setBounds(projected);

                setName("Subspace[" + std::to_string(active_indices_.size()) + "/" + std::to_string(ambient_dim_) +
                        "]");
            }

            ~SubspaceStateSpace() override = default;

            /** \brief Dimension of the ambient space the subspace was projected from. */
            std::size_t getAmbientDimension() const
            {
                return ambient_dim_;
            }

            /** \brief Indices into the ambient vector that this subspace varies. */
            const std::vector<std::size_t> &getActiveIndices() const
            {
                return active_indices_;
            }

            /** \brief Reference pose for the ambient dimensions held frozen
             * (length \c getAmbientDimension(); entries at active indices are
             * unused). */
            const std::vector<double> &getFrozenValues() const
            {
                return frozen_values_;
            }

            /** \brief Version counter for the frozen pose.
             *
             * Incremented every time \c setFrozenValues mutates the frozen
             * vector. Adapters that cache derived data (e.g. a single-precision
             * float buffer for SIMD collision checking) can compare a stored
             * version against the current one to know when their cache is
             * stale, so a per-call refresh stays at the cost of one integer
             * compare. */
            std::size_t getFrozenVersion() const
            {
                return frozen_version_;
            }

            /** \brief Update the frozen reference pose in place.
             *
             * Bounds and dimension are unchanged, so any
             * SpaceInformation/planner already pointing at this space
             * remains valid; only collision/motion checks computed after the
             * call see the new pose. Useful when the frozen subset
             * represents a slow-changing component the user wants to
             * advance between planning queries. */
            void setFrozenValues(std::vector<double> frozen_values)
            {
                if (frozen_values.size() != ambient_dim_)
                {
                    throw Exception("SubspaceStateSpace::setFrozenValues: length must equal the ambient dimension");
                }
                frozen_values_ = std::move(frozen_values);
                ++frozen_version_;
            }

            /** \brief Lift a subspace state into a full ambient configuration.
             * \c out is resized to \c getAmbientDimension(). */
            void expandToFull(const State *state, std::vector<double> &out) const
            {
                out.assign(frozen_values_.begin(), frozen_values_.end());
                const auto *values = state->as<StateType>()->values;
                for (std::size_t i = 0; i < active_indices_.size(); ++i)
                {
                    out[active_indices_[i]] = values[i];
                }
            }

            /** \brief Allocating overload of expandToFull. */
            std::vector<double> expandToFull(const State *state) const
            {
                std::vector<double> out;
                expandToFull(state, out);
                return out;
            }

        private:
            std::size_t ambient_dim_;
            std::vector<std::size_t> active_indices_;
            std::vector<double> frozen_values_;
            std::size_t frozen_version_{0};
        };
    }  // namespace base
}  // namespace ompl

#endif
