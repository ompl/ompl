/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Rice University
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

/* Author: Ioan Sucan */

#ifndef OMPL_BASE_SPACES_REAL_VECTOR_STATE_PROJECTIONS_
#define OMPL_BASE_SPACES_REAL_VECTOR_STATE_PROJECTIONS_

#include "ompl/base/ProjectionEvaluator.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"

namespace ompl
{
    namespace base
    {
        /** \brief Definition for a class computing linear projections
            (multiplication of a k-by-n matrix to the the
            R<sup>n</sup> vector state to produce an R<sup>k</sup>
            projection. The multiplication matrix needs to be supplied
            as input. */
        class RealVectorLinearProjectionEvaluator : public ProjectionEvaluator
        {
        public:
            /** \brief Initialize a linear projection evaluator for state space \e space. The used projection matrix is
                \e projection and the cell sizes are \e cellSizes. */
            RealVectorLinearProjectionEvaluator(const StateSpace *space, const std::vector<double> &cellSizes,
                                                const ProjectionMatrix::Matrix &projection);

            /** \brief Initialize a linear projection evaluator for state space \e space. The used projection matrix is
                \e projection and the cell sizes are \e cellSizes. */
            RealVectorLinearProjectionEvaluator(const StateSpacePtr &space, const std::vector<double> &cellSizes,
                                                const ProjectionMatrix::Matrix &projection);

            /** \brief Initialize a linear projection evaluator for state space \e space. The used projection matrix is
                \e projection and the cell sizes are automatically inferred through sampling. */
            RealVectorLinearProjectionEvaluator(const StateSpace *space, const ProjectionMatrix::Matrix &projection);

            /** \brief Initialize a linear projection evaluator for state space \e space. The used projection matrix is
                \e projection and the cell sizes are automatically inferred through sampling. */
            RealVectorLinearProjectionEvaluator(const StateSpacePtr &space, const ProjectionMatrix::Matrix &projection);

            unsigned int getDimension() const override;

            void project(const State *state, Eigen::Ref<Eigen::VectorXd> projection) const override;

        protected:
            /** \brief The projection matrix */
            ProjectionMatrix projection_;
        };

        /** \brief Definition for a class computing a random linear projections */
        class RealVectorRandomLinearProjectionEvaluator : public RealVectorLinearProjectionEvaluator
        {
        public:
            /** \brief Initialize a linear projection evaluator for state space \e space. The used projection matrix is
                sampled at random and the cell sizes are automatically inferred through sampling. */
            RealVectorRandomLinearProjectionEvaluator(const StateSpace *space, const std::vector<double> &cellSizes)
              : RealVectorLinearProjectionEvaluator(
                    space, cellSizes, ProjectionMatrix::ComputeRandom(space->getDimension(), cellSizes.size()))
            {
            }

            /** \brief Initialize a linear projection evaluator for state space \e space. The used projection matrix is
                sampled at random and the cell sizes are automatically inferred through sampling. */
            RealVectorRandomLinearProjectionEvaluator(const StateSpacePtr &space, const std::vector<double> &cellSizes)
              : RealVectorLinearProjectionEvaluator(
                    space, cellSizes, ProjectionMatrix::ComputeRandom(space->getDimension(), cellSizes.size()))
            {
            }

            /** \brief Initialize a linear projection evaluator for state space \e space. The used projection matrix is
                sampled at random to produce a space of dimension \e dim and the cell sizes are automatically inferred
               through sampling. */
            RealVectorRandomLinearProjectionEvaluator(const StateSpace *space, unsigned int dim)
              : RealVectorLinearProjectionEvaluator(
                    space,
                    ProjectionMatrix::ComputeRandom(space->getDimension(), dim,
                                                    space->as<RealVectorStateSpace>()->getBounds().getDifference()))
            {
            }

            /** \brief Initialize a linear projection evaluator for state space \e space. The used projection matrix is
                sampled at random to produce a space of dimension \e dim and the cell sizes are automatically inferred
               through sampling. */
            RealVectorRandomLinearProjectionEvaluator(const StateSpacePtr &space, unsigned int dim)
              : RealVectorLinearProjectionEvaluator(
                    space,
                    ProjectionMatrix::ComputeRandom(space->getDimension(), dim,
                                                    space->as<RealVectorStateSpace>()->getBounds().getDifference()))
            {
            }
        };

        /** \brief Definition for a class computing orthogonal projections */
        class RealVectorOrthogonalProjectionEvaluator : public ProjectionEvaluator
        {
        public:
            /** \brief Initialize an orthogonal projection evaluator for state space \e space. The indices of the
                kept components are in \e components and the cell sizes are in \e cellSizes */
            RealVectorOrthogonalProjectionEvaluator(const StateSpace *space, const std::vector<double> &cellSizes,
                                                    std::vector<unsigned int> components);

            /** \brief Initialize an orthogonal projection evaluator for state space \e space. The indices of the
                kept components are in \e components and the cell sizes are in \e cellSizes */
            RealVectorOrthogonalProjectionEvaluator(const StateSpacePtr &space, const std::vector<double> &cellSizes,
                                                    std::vector<unsigned int> components);

            /** \brief Initialize an orthogonal projection evaluator for state space \e space. The indices of the
                kept components are in \e components and the cell sizes are a tenth of the corresponding bounds from the
               state space. */
            RealVectorOrthogonalProjectionEvaluator(const StateSpace *space, std::vector<unsigned int> components);

            /** \brief Initialize an orthogonal projection evaluator for state space \e space. The indices of the
                kept components are in \e components and the cell sizes are a tenth of the corresponding bounds from the
               state space.  */
            RealVectorOrthogonalProjectionEvaluator(const StateSpacePtr &space, std::vector<unsigned int> components);

            unsigned int getDimension() const override;

            void defaultCellSizes() override;

            void project(const State *state, Eigen::Ref<Eigen::VectorXd> projection) const override;

        protected:
            /** \brief Fill bounds_ with bounds from the state space */
            void copyBounds();

            /** \brief The set of components selected by the projection */
            std::vector<unsigned int> components_;
        };

        /** \brief Define the identity projection */
        class RealVectorIdentityProjectionEvaluator : public ProjectionEvaluator
        {
        public:
            /** \brief Initialize the identity projection evaluator for state space \e space. The indices of the
                kept components are in \e components and the cell sizes are in \e cellSizes */
            RealVectorIdentityProjectionEvaluator(const StateSpace *space, const std::vector<double> &cellSizes);

            /** \brief Initialize the identity projection evaluator for state space \e space. The indices of the
                kept components are in \e components and the cell sizes are in \e cellSizes */
            RealVectorIdentityProjectionEvaluator(const StateSpacePtr &space, const std::vector<double> &cellSizes);

            /** \brief Initialize the identity projection evaluator for state space \e space. The indices of the
                kept components are in \e components and the cell sizes are a tenth of the bounds from the state space.
               */
            RealVectorIdentityProjectionEvaluator(const StateSpace *space);

            /** \brief Initialize the identity projection evaluator for state space \e space. The indices of the
                kept components are in \e components and the cell sizes are a tenth of the bounds from the state space.
               */
            RealVectorIdentityProjectionEvaluator(const StateSpacePtr &space);

            unsigned int getDimension() const override;

            void defaultCellSizes() override;

            void setup() override;

            void project(const State *state, Eigen::Ref<Eigen::VectorXd> projection) const override;

        private:
            /** \brief Fill bounds_ with bounds from the state space */
            void copyBounds();

            /** \brief The amount of data to copy from projection to state */
            std::size_t copySize_;
        };
    }  // namespace base
}  // namespace ompl
#endif
