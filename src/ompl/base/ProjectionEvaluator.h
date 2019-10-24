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

#ifndef OMPL_BASE_PROJECTION_EVALUATOR_
#define OMPL_BASE_PROJECTION_EVALUATOR_

#include "ompl/base/State.h"
#include "ompl/util/ClassForward.h"
#include "ompl/util/Console.h"
#include "ompl/base/GenericParam.h"
#include "ompl/base/spaces/RealVectorBounds.h"

#include <vector>
#include <valarray>
#include <iostream>
#include <Eigen/Core>

namespace ompl
{
    namespace base
    {
        /** \brief A projection matrix -- it allows multiplication of
            real vectors by a specified matrix. The matrix can also be
            randomly generated. */
        class ProjectionMatrix
        {
        public:
            /** \brief Datatype for projection matrices */
            using Matrix = Eigen::MatrixXd;

            /** \brief Compute a random projection matrix with \e from
                columns and \e to rows. A vector with \e from elements
                can be multiplied by this matrix in order to produce a
                vector with \e to elements.

                If the \e scale argument is specified, the columns of
                the matrix are divided by the corresponding scaling
                argument: all elements (rows) in column[i] are divided
                by scale[i]. This is useful to specify if scaling of
                the elements of the state is to be applied before
                projection. If the scale for a column is 0, the column
                itself is set to 0.

                Each element is sampled with a Gaussian distribution
                with mean 0 and variance 1 and the matrix rows are
                made orthonormal. */
            static Matrix ComputeRandom(unsigned int from, unsigned int to, const std::vector<double> &scale);

            /** \brief Compute a random projection matrix with \e from
                columns and \e to rows. A vector with \e from elements
                can be multiplied by this matrix in order to produce a
                vector with \e to elements. This uses the function above
                called with an empty \e scale vector.

                Each element is sampled with a Gaussian distribution
                with mean 0 and variance 1 and the matrix rows are
                made orthonormal. */
            static Matrix ComputeRandom(unsigned int from, unsigned int to);

            /** \brief Wrapper for ComputeRandom(from, to, scale) */
            void computeRandom(unsigned int from, unsigned int to, const std::vector<double> &scale);

            /** \brief Wrapper for ComputeRandom(from, to) */
            void computeRandom(unsigned int from, unsigned int to);

            /** \brief Multiply the vector \e from by the contained projection matrix to obtain the vector \e to. */
            void project(const double *from, Eigen::Ref<Eigen::VectorXd> to) const;

            /** \brief Print the contained projection matrix to a stram */
            void print(std::ostream &out = std::cout) const;

            /** \brief Projection matrix */
            Matrix mat;

        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        };

        /// @cond IGNORE
        OMPL_CLASS_FORWARD(StateSpace);
        /// @endcond

        /// @cond IGNORE
        /** \brief Forward declaration of ompl::base::ProjectionEvaluator */
        OMPL_CLASS_FORWARD(ProjectionEvaluator);
        /// @endcond

        /** \class ompl::base::ProjectionEvaluatorPtr
            \brief A shared pointer wrapper for ompl::base::ProjectionEvaluator */

        /** \brief Abstract definition for a class computing
            projections to R<sup>n</sup>. Implicit integer grids are
            imposed on this projection space by setting cell
            sizes. Before use, the user must supply cell sizes
            for the integer grid (setCellSizes()). The
            implementation of this class is thread safe. */
        class ProjectionEvaluator
        {
        public:
            // non-copyable
            ProjectionEvaluator(const ProjectionEvaluator &) = delete;
            ProjectionEvaluator &operator=(const ProjectionEvaluator &) = delete;

            /** \brief Construct a projection evaluator for a specific state space */
            ProjectionEvaluator(const StateSpace *space);

            /** \brief Construct a projection evaluator for a specific state space */
            ProjectionEvaluator(const StateSpacePtr &space);

            virtual ~ProjectionEvaluator();

            /** \brief Return the dimension of the projection defined by this evaluator */
            virtual unsigned int getDimension() const = 0;

            /** \brief Compute the projection as an array of double values */
            virtual void project(const State *state, Eigen::Ref<Eigen::VectorXd> projection) const = 0;

            /** \brief Define the size (in each dimension) of a grid
                cell. The number of sizes set here must be the
                same as the dimension of the projection computed by
                the projection evaluator. After a call to this
                function, setup() will not call
                defaultCellSizes() or inferCellSizes() any
                more. */
            virtual void setCellSizes(const std::vector<double> &cellSizes);

            /** \brief Set the cell sizes to \e cellSize for a particular dimension \e dim. This function simply calls
               getCellSizes(),
                modifies the desired dimension and then calls setCellSizes(). This is done intentionally to enforce a
               call to setCellSizes(). */
            void setCellSizes(unsigned int dim, double cellSize);

            /** \brief Multiply the cell sizes in each dimension by a specified factor \e factor. This function does
               nothing
                if cell sizes have not been set. If cell sizes have been set (irrespective of source; e.g., user,
               default or inferred),
                this function will then call setCellSizes(), so the source of the cell sizes will be considered to be
               the user. */
            void mulCellSizes(double factor);

            /** \brief Return true if any user configuration has been done to this projection evaluator (setCellSizes()
             * was called) */
            bool userConfigured() const;

            /** \brief Get the size (each dimension) of a grid cell  */
            const std::vector<double> &getCellSizes() const
            {
                return cellSizes_;
            }

            /** \brief Get the size of a particular dimension of a grid cell  */
            double getCellSizes(unsigned int dim) const;

            /** \brief Check if cell dimensions match projection dimension */
            void checkCellSizes() const;

            /** \brief Sample the state space and decide on default
                cell sizes. This function is called by setup() if
                no cell dsizes have been set and
                defaultCellSizes() does not fill the cell
                sizes either. */
            void inferCellSizes();

            /** \brief Set the default cell dimensions for this
                projection. The default implementation of this
                function is empty. setup() calls this function if no
                cell dimensions have been previously set. */
            virtual void defaultCellSizes();

            /** \brief Check if the projection dimension matched the dimension of the bounds */
            void checkBounds() const;

            /** \brief Check if bounds were specified for this projection */
            bool hasBounds() const
            {
                return !bounds_.low.empty();
            }

            /** \brief Set bounds on the projection. The PDST planner
                 needs to known the bounds on the projection. Default bounds
                 are automatically computed by inferCellSizes(). */
            void setBounds(const RealVectorBounds &bounds);

            /** \brief Get the bounds computed/set for this projection */
            const RealVectorBounds &getBounds() const
            {
                return bounds_;
            }

            /** \brief Compute an approximation of the bounds for this projection space. getBounds() will then report
             * the computed bounds. */
            void inferBounds();

            /** \brief Perform configuration steps, if needed */
            virtual void setup();

            /** \brief Compute integer coordinates for a projection */
            void computeCoordinates(const Eigen::Ref<Eigen::VectorXd> &projection,
                                    Eigen::Ref<Eigen::VectorXi> coord) const;

            /** \brief Compute integer coordinates for a state */
            void computeCoordinates(const State *state, Eigen::Ref<Eigen::VectorXi> coord) const
            {
                Eigen::VectorXd projection(getDimension());
                project(state, projection);
                computeCoordinates(projection, coord);
            }

            /** \brief Get the parameters for this projection */
            ParamSet &params()
            {
                return params_;
            }

            /** \brief Get the parameters for this projection */
            const ParamSet &params() const
            {
                return params_;
            }

            /** \brief Print settings about this projection */
            virtual void printSettings(std::ostream &out = std::cout) const;

            /** \brief Print a euclidean projection */
            virtual void printProjection(const Eigen::Ref<Eigen::VectorXd> &projection,
                                         std::ostream &out = std::cout) const;

        protected:
            /** \brief Fill estimatedBounds_ with an approximate bounding box for the projection space (via sampling) */
            void estimateBounds();

            /** \brief The state space this projection operates on */
            const StateSpace *space_;

            /** \brief The size of a cell, in every dimension of the
                projected space, in the implicitly defined integer
                grid. */
            std::vector<double> cellSizes_;

            /** \brief A bounding box for projected state values */
            RealVectorBounds bounds_;

            /** \brief An approximate bounding box for projected state values;
                This is the cached result of estimateBounds() which may later be copied
                to bounds_ if bounds are needed but were not specified. */
            RealVectorBounds estimatedBounds_;

            /** \brief Flag indicating whether cell sizes have
                been set by the user, or whether they were inferred
                automatically. This flag becomes false if
                setCellSizes() is called. */
            bool defaultCellSizes_;

            /** \brief Flag indicating whether projection cell sizes
                were automatically inferred. */
            bool cellSizesWereInferred_;

            /** \brief The set of parameters for this projection */
            ParamSet params_;
        };

        /** \brief If the projection for a CompoundStateSpace is
            supposed to be the same as the one for one of its included
            subspaces, this class facilitates selecting a projection
            of that subspace. */
        class SubspaceProjectionEvaluator : public ProjectionEvaluator
        {
        public:
            /** \brief The constructor states that for space \e space,
                the projection to use is the same as the component at
                position \e index of space \e space. The actual
                projection to use can be specified by \e projToUse. If
                the projection is not specified, the default one for
                the subspace at position \e index is used. */
            SubspaceProjectionEvaluator(const StateSpace *space, unsigned int index,
                                        ProjectionEvaluatorPtr projToUse = ProjectionEvaluatorPtr());

            void setup() override;

            unsigned int getDimension() const override;

            void project(const State *state, Eigen::Ref<Eigen::VectorXd> projection) const override;

        protected:
            /** \brief The index of the subspace from which to project */
            unsigned int index_;

            /** \brief The projection to use. This is either the same
                as \e specifiedProj_ or, if specifiedProj_ is not
                initialized, it is the default projection for the
                subspace at index \e index_ */
            ProjectionEvaluatorPtr proj_;

            /** \brief The projection that is optionally specified by the user in the constructor argument (\e
             * projToUse) */
            ProjectionEvaluatorPtr specifiedProj_;
        };
    }  // namespace base
}  // namespace ompl

#endif
