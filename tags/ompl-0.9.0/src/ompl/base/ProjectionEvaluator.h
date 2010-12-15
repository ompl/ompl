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
#include <vector>
#include <valarray>
#include <iostream>
#include <boost/noncopyable.hpp>

namespace ompl
{

    namespace base
    {

        /** \brief Grid cells corresponding to a projection value are described in terms of their coordinates. */
        typedef std::vector<int> ProjectionCoordinates;

        /** \brief The datatype for state projections. This class contains a real vector. */
        class EuclideanProjection
        {
        public:

            /** \brief Allocate a projection of dimension n */
            EuclideanProjection(unsigned int n) : values(new double[n])
            {
            }

            ~EuclideanProjection(void)
            {
                delete[] values;
            }

            /** \brief Access operator (constant) */
            double operator[](unsigned int i) const
            {
                return values[i];
            }

            /** \brief Access operator */
            double& operator[](unsigned int i)
            {
                return values[i];
            }

            /** \brief The values of the R<sup>n</sup> vector that makes up the projection */
            double *values;
        };

        /** \brief A projection matrix -- it allows multiplication of
            real vectors by a specified matrix. The matrix can also be
            randomly generated. */
        class ProjectionMatrix
        {
        public:

            /** \brief Datatype for projection matrices */
            typedef std::vector< std::valarray<double> > Matrix;

            /** \brief Special value for specifying that the columns in the projection matrix are as originaly sampled (unscaled) */
            static const std::vector<double> UNSCALED;

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
            static Matrix ComputeRandom(const unsigned int from, const unsigned int to, const std::vector<double> &scale = UNSCALED);

            /** \copydoc ComputeRandom() */
            void computeRandom(const unsigned int from, const unsigned int to, const std::vector<double> &scale = UNSCALED);

            /** \brief Multiply the vector \e from by the contained projection matrix to obtain the vector \e to. */
            void project(const double *from, double *to) const;

            /** \brief Print the contained projection matrix to a stram */
            void print(std::ostream &out = std::cout) const;

            /** \brief Projection matrix */
            Matrix mat;
        };

        ClassForward(StateManifold);

        /** \brief Forward declaration of ompl::base::ProjectionEvaluator */
        ClassForward(ProjectionEvaluator);

        /** \class ompl::base::ProjectionEvaluatorPtr
            \brief A boost shared pointer wrapper for ompl::base::ProjectionEvaluator */

        /** \brief Abstract definition for a class computing
            projections to R<sup>n</sup>. Implicit integer grids are
            imposed on this projection space by setting cell
            sizes. Before use, the user must supply cell dimensions
            for the integer grid (setCellDimensions()). The
            implementation of this class is thread safe. */
        class ProjectionEvaluator : private boost::noncopyable
        {
        public:

            /** \brief Construct a projection evaluator for a specific manifold */
            ProjectionEvaluator(const StateManifold *manifold) : manifold_(manifold)
            {
            }

            /** \brief Construct a projection evaluator for a specific manifold */
            ProjectionEvaluator(const StateManifoldPtr &manifold) : manifold_(manifold.get())
            {
            }

            virtual ~ProjectionEvaluator(void)
            {
            }

            /** \brief Return the dimension of the projection defined by this evaluator */
            virtual unsigned int getDimension(void) const = 0;

            /** \brief Compute the projection as an array of double values */
            virtual void project(const State *state, EuclideanProjection &projection) const = 0;

            /** \brief Define the dimension (each component) of a grid cell. The
                number of dimensions set here must be the same as the
                dimension of the projection computed by the projection
                evaluator. */
            void setCellDimensions(const std::vector<double> &cellDimensions);

            /** \brief Get the dimension (each component) of a grid cell  */
            const std::vector<double>& getCellDimensions(void) const
            {
                return cellDimensions_;
            }

            /** \brief Check if cell dimensions match projection dimension */
            void checkCellDimensions(void) const;

            /** \brief Sample the manifold and decide on default cell dimensions */
            void inferCellDimensions(void);

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

            /** \brief The manifold this projection operates on */
            const StateManifold *manifold_;

            /** \brief The size of a cell, in every dimension of the
                projected space, in the implicitly defined integer
                grid. */
            std::vector<double>  cellDimensions_;

        };

        /** \brief Construct a projection evaluator from a set of
            existing projection evaluators. A compound state is
            projected by concatenating the projection of each
            component of a state into one composite projected
            vector. If this vector has dimension larger than 2, it is
            further projected with a random matrix to a dimension of
            logarithmic size.

            \note For example, if two projection evaluators are
            composed, each with dimension 1, the result of the
            compound projection would simply be the concatenation of
            the two contained projections (so the result will be a
            projection of dimension 2). If the concatenation of the
            contained projections is K > 2, a projection of this
            concatenation to dimension ceil(log(K)) is computed. */
        class CompoundProjectionEvaluator : public ProjectionEvaluator
        {
        public:

            /** \brief Constructor */
            CompoundProjectionEvaluator(const StateManifold *manifold) : ProjectionEvaluator(manifold), dimension_(0), compoundDimension_(0)
            {
            }

            /** \brief Constructor */
            CompoundProjectionEvaluator(const StateManifoldPtr &manifold) : ProjectionEvaluator(manifold), dimension_(0), compoundDimension_(0)
            {
            }

            virtual unsigned int getDimension(void) const;

            /** \brief Add a projection evaluator to consider when computing projections of compound states */
            virtual void addProjectionEvaluator(const ProjectionEvaluatorPtr &proj);

            virtual void project(const State *state, EuclideanProjection &projection) const;

            virtual void printSettings(std::ostream &out = std::cout) const;

        protected:


            /** \brief Update the maintained projection. Called by addProjectionEvaluator() */
            void computeProjection(void);

            /** \brief Projections for each of the contained components */
            std::vector<ProjectionEvaluatorPtr> components_;

            /** \brief Projection matrix used in case the concatenanted projection vectors make up a vector of large dimension */
            ProjectionMatrix                    projection_;

            /** \brief The dimension of the projection (number of elements in the projected vector) */
            unsigned int                        dimension_;

            /** \brief The sum of dimensions of the contained components. This is the dimension of the concatenated projection vector */
            unsigned int                        compoundDimension_;

        };
    }

}

#endif
