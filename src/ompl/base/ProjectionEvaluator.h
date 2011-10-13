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
#include <vector>
#include <valarray>
#include <iostream>
#include <boost/noncopyable.hpp>
#include <boost/numeric/ublas/matrix.hpp>

namespace ompl
{

    namespace base
    {

        /** \brief Grid cells corresponding to a projection value are described in terms of their coordinates. */
        typedef std::vector<int> ProjectionCoordinates;

        /** \brief The datatype for state projections. This class contains a real vector. */
        typedef boost::numeric::ublas::vector<double> EuclideanProjection;


        /** \brief A projection matrix -- it allows multiplication of
            real vectors by a specified matrix. The matrix can also be
            randomly generated. */
        class ProjectionMatrix
        {
        public:

            /** \brief Datatype for projection matrices */
            typedef boost::numeric::ublas::matrix<double> Matrix;

            /** \brief Compute a random projection matrix with \e from
                columns and \e to rows. A vector with \e from elements
                can be multiplied by this matrix in order to produce a
                vector with \e to elements.

                If the \e scale argument is specified, the columns of
                the matrix are divided by the corresponding scaling
                argument: all elements (rows) in column[i] are divided
                by scale[i]. This is useful to specify if scaling of
                the elements of the state is to be applied before
                projection.

                Each element is sampled with a Gaussian distribution
                with mean 0 and variance 1 and the matrix columns are
                made orthonormal. */
            static Matrix ComputeRandom(const unsigned int from, const unsigned int to, const std::vector<double> &scale);

            /** \brief Compute a random projection matrix with \e from
                columns and \e to rows. A vector with \e from elements
                can be multiplied by this matrix in order to produce a
                vector with \e to elements.

                Each element is sampled with a Gaussian distribution
                with mean 0 and variance 1 and the matrix columns are
                made orthonormal. */
            static Matrix ComputeRandom(const unsigned int from, const unsigned int to);

            /** \brief Wrapper for ComputeRandom(from, to, scale) */
            void computeRandom(const unsigned int from, const unsigned int to, const std::vector<double> &scale);

            /** \brief Wrapper for ComputeRandom(from, to) */
            void computeRandom(const unsigned int from, const unsigned int to);

            /** \brief Multiply the vector \e from by the contained projection matrix to obtain the vector \e to. */
            void project(const double *from, EuclideanProjection& to) const;

            /** \brief Print the contained projection matrix to a stram */
            void print(std::ostream &out = std::cout) const;

            /** \brief Projection matrix */
            Matrix mat;
        };

        /// @cond IGNORE
        ClassForward(StateSpace);
        /// @endcond

        /// @cond IGNORE
        /** \brief Forward declaration of ompl::base::ProjectionEvaluator */
        ClassForward(ProjectionEvaluator);
        /// @endcond

        /** \class ompl::base::ProjectionEvaluatorPtr
            \brief A boost shared pointer wrapper for ompl::base::ProjectionEvaluator */

        /** \brief Abstract definition for a class computing
            projections to R<sup>n</sup>. Implicit integer grids are
            imposed on this projection space by setting cell
            sizes. Before use, the user must supply cell sizes
            for the integer grid (setCellSizes()). The
            implementation of this class is thread safe. */
        class ProjectionEvaluator : private boost::noncopyable
        {
        public:

            /** \brief Construct a projection evaluator for a specific state space */
            ProjectionEvaluator(const StateSpace *space) : space_(space), defaultCellSizes_(true), cellSizesWereInferred_(false)
            {
            }

            /** \brief Construct a projection evaluator for a specific state space */
            ProjectionEvaluator(const StateSpacePtr &space) : space_(space.get()), defaultCellSizes_(true), cellSizesWereInferred_(false)
            {
            }

            virtual ~ProjectionEvaluator(void)
            {
            }

            /** \brief Return the dimension of the projection defined by this evaluator */
            virtual unsigned int getDimension(void) const = 0;

            /** \brief Compute the projection as an array of double values */
            virtual void project(const State *state, EuclideanProjection &projection) const = 0;

            /** \brief Define the size (in each dimension) of a grid
                cell. The number of sizes set here must be the
                same as the dimension of the projection computed by
                the projection evaluator. After a call to this
                function, setup() will not call
                defaultCellSizes() or inferCellSizes() any
                more. */
            virtual void setCellSizes(const std::vector<double> &cellSizes);

            /** \brief Return true if any user configuration has been done to this projection evaluator (setCellSizes() was called) */
            bool userConfigured(void) const;

            /** \brief Get the size (each dimension) of a grid cell  */
            const std::vector<double>& getCellSizes(void) const
            {
                return cellSizes_;
            }

            /** \brief Check if cell dimensions match projection dimension */
            void checkCellSizes(void) const;

            /** \brief Sample the state space and decide on default
                cell sizes. This function is called by setup() if
                no cell dsizes have been set and
                defaultCellSizes() does not fill the cell
                sizes either. */
            void inferCellSizes(void);

            /** \brief Set the default cell dimensions for this
                projection. The default implementation of this
                function is empty. setup() calls this function if no
                cell dimensions have been previously set. */
            virtual void defaultCellSizes(void);

            /** \brief Perform configuration steps, if needed */
            virtual void setup(void);

            /** \brief Compute integer coordinates for a projection */
            void computeCoordinates(const EuclideanProjection &projection, ProjectionCoordinates &coord) const;

            /** \brief Compute integer coordinates for a state */
            void computeCoordinates(const State *state, ProjectionCoordinates &coord) const
            {
                EuclideanProjection projection(getDimension());
                project(state, projection);
                computeCoordinates(projection, coord);
            }

            /** \brief Print settings about this projection */
            virtual void printSettings(std::ostream &out = std::cout) const;

            /** \brief Print a euclidean projection */
            virtual void printProjection(const EuclideanProjection &projection, std::ostream &out = std::cout) const;

        protected:

            /** \brief The state space this projection operates on */
            const StateSpace    *space_;

            /** \brief The size of a cell, in every dimension of the
                projected space, in the implicitly defined integer
                grid. */
            std::vector<double>  cellSizes_;

            /** \brief Flag indicating whether cell sizes have
                been set by the user, or whether they were inferred
                automatically. This flag becomes false if
                setCellSizes() is called. */
            bool                 defaultCellSizes_;

            /** \brief Flag indicating whether projection cell sizes
                were automatically inferred. */
            bool                 cellSizesWereInferred_;

            /** \brief The console interface */
            msg::Interface       msg_;
        };

        /** \brief If the projection for a CompoundStateSpace is
            supposed to be the same as the one for one of its included
            subspaces, this class facilitates selecting a projection
            of that subspace. */
        class SubSpaceProjectionEvaluator : public ProjectionEvaluator
        {
        public:

            /** \brief The constructor states that for space \e space,
                the projection to use is the same as the component at
                position \e index of space \e space. The actual
                projection to use can be specified by \e projToUse. If
                the projection is not specified, the default one for
                the subspace at position \e index is used. */
            SubSpaceProjectionEvaluator(const StateSpace *space, unsigned int index, const ProjectionEvaluatorPtr &projToUse = ProjectionEvaluatorPtr());

            virtual void setup(void);

            virtual unsigned int getDimension(void) const;

            virtual void project(const State *state, EuclideanProjection &projection) const;

        protected:

            /** \brief The index of the subspace from which to project */
            unsigned int           index_;

            /** \brief The projection to use. This is either the same
                as \e specifiedProj_ or, if specifiedProj_ is not
                initialized, it is the default projection for the
                subspace at index \e index_ */
            ProjectionEvaluatorPtr proj_;

            /** \brief The projection that is optionally specified by the user in the constructor argument (\e projToUse) */
            ProjectionEvaluatorPtr specifiedProj_;
        };

    }

}

#endif
