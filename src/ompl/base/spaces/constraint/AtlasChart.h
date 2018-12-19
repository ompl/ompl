/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, Rice University
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

/* Author: Zachary Kingston, Caleb Voss */

#ifndef OMPL_BASE_SPACES_ATLAS_CHART_
#define OMPL_BASE_SPACES_ATLAS_CHART_

#include "ompl/base/spaces/constraint/AtlasStateSpace.h"
#include "ompl/datastructures/PDF.h"

#include <vector>
#include <Eigen/Core>

namespace ompl
{
    namespace base
    {
        /** \brief Tangent space and bounding polytope approximating some patch
         * of the manifold. */
        class AtlasChart
        {
        private:
            /** \brief Halfspace equation on a chart. \note Use
             * AtlasChart::generateHalfspace to create new halfspace objects.
             * Since each halfspace is associated to exactly one chart, we let
             * the chart be responsible for deleting it. */
            class Halfspace
            {
            public:
                // non-copyable
                Halfspace(const Halfspace &) = delete;
                Halfspace &operator=(const Halfspace &) = delete;

                /** \brief Create a halfspace equitably separating charts \a
                 * owner and \a neighbor. This halfspace will coincide with
                 * chart \a owner. */
                Halfspace(const AtlasChart *owner, const AtlasChart *neighbor);

                /** \brief Return whether point \a v on the owning chart
                 * lies within the halfspace. */
                bool contains(const Eigen::Ref<const Eigen::VectorXd> &v) const;

                /** \brief If point \a v on the owning chart is very close to
                 * the halfspace boundary, the "complementary" halfspace will
                 * extend its boundary so that it also contains \a v when \a v
                 * is projected onto the neighboring chart. */
                void checkNear(const Eigen::Ref<const Eigen::VectorXd> &v) const;

                /** \brief Compute up to two vertices of intersection with a
                 * circle of radius \a r.  If one vertex is found, it is stored
                 * to both \a v1 and \a v2; if two are found, they are stored to
                 * \a v1 and \a v2. If no vertex is found, returns false;
                 * otherwise returns true. */
                bool circleIntersect(double r, Eigen::Ref<Eigen::VectorXd> v1, Eigen::Ref<Eigen::VectorXd> v2) const;

                /** \brief Compute the vertex of intersection of two
                 * 1-dimensional inequalities \a l1 and \a l2.  Result stored in
                 * \a out, which should be allocated to a size of 2. */
                static void intersect(const Halfspace &l1, const Halfspace &l2, Eigen::Ref<Eigen::VectorXd> out);

                /** \brief Inform this halfspace about the "complementary"
                 * halfspace which coincides with the neighboring chart. */
                void setComplement(Halfspace *complement)
                {
                    complement_ = complement;
                }

                /** \brief Get the complementary halfspace. */
                Halfspace *getComplement() const
                {
                    return complement_;
                }

                /** \brief Get the chart to which this halfspace belongs. */
                const AtlasChart *getOwner() const
                {
                    return owner_;
                }

            private:
                /** \brief Chart to which this halfspace belongs. */
                const AtlasChart *owner_;

                /** \brief Halfspace complementary to this one, but on the
                 * neighboring chart. */
                Halfspace *complement_{nullptr};

                /** \brief Center of the neighboring chart projected onto our
                 * chart. */
                Eigen::VectorXd u_;

                /** \brief Precomputed squared norm of u_ */
                double usqnorm_;

                /** \brief Precomputed right-hand side of the inequality. */
                double rhs_;

                /** \brief Generate the linear inequality. We will divide the
                 * space in half between \a u and 0, and 0 will lie inside. */
                void setU(const Eigen::Ref<const Eigen::VectorXd> &u);

                /** \brief Compute the distance between a point \a v on our
                 * chart and the halfspace boundary as a scalar factor of
                 * \a u_. That is, \a result * \a u_ lies on the halfspace
                 * boundary, and \a v, \a u_, \a result * \a u_ are colinear. */
                double distanceToPoint(const Eigen::Ref<const Eigen::VectorXd> &v) const;

                /** \brief Expand the halfspace to include ambient point \a x
                 * when it is projected onto our chart. */
                void expandToInclude(const Eigen::Ref<const Eigen::VectorXd> &x);
            };

        public:
            // non-copyable
            AtlasChart(const AtlasChart &) = delete;
            AtlasChart &operator=(const AtlasChart &) = delete;

            /** \brief Create a tangent space chart for \a atlas with center at
             * ambient space point \a xorigin.
             * \throws ompl::Exception when manifold seems degenerate here. */
            AtlasChart(const AtlasStateSpace *atlas, const AtlasStateSpace::StateType *state);

            /** \brief Destructor. */
            ~AtlasChart();

            /** \brief Forget all acquired information such as the halfspace
             * boundary. */
            void clear();

            /** \brief Returns phi(0), the center of the chart in ambient
             * space. */
            const AtlasStateSpace::StateType *getOrigin() const
            {
                return state_;
            }

            /** \brief Returns the dimension of the ambient space. */
            unsigned int getAmbientDimension() const
            {
                return n_;
            }

            /** \brief Returns the dimension of the manifold. */
            unsigned int getManifoldDimension() const
            {
                return k_;
            }

            /** \brief Rewrite a chart point \a u in ambient space coordinates
             * and store the result in \a out, which should be allocated to size
             * n_.
             */
            void phi(const Eigen::Ref<const Eigen::VectorXd> &u, Eigen::Ref<Eigen::VectorXd> out) const;

            /** \brief Exponential mapping. Project chart point \a u onto the
             * manifold and store the result in \a out, which should be
             * allocated to size n_. */
            bool psi(const Eigen::Ref<const Eigen::VectorXd> &u, Eigen::Ref<Eigen::VectorXd> out) const;

            /** \brief Logarithmic mapping. Project ambient point \a x onto the
             * chart and store the result in \a out, which should be allocated
             * to size k_. */
            void psiInverse(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::VectorXd> out) const;

            /** \brief Check if a point \a u on the chart lies within its
             * polytope boundary. Can ignore up to 2 of the halfspaces if
             * specified in \a ignore1 and \a ignore2. */
            bool inPolytope(const Eigen::Ref<const Eigen::VectorXd> &u, const Halfspace *ignore1 = nullptr,
                            const Halfspace *ignore2 = nullptr) const;

            /** \brief Check if chart point \a v lies very close to any part of
             * the boundary. Wherever it does, expand the neighboring chart's
             * boundary to include. */
            void borderCheck(const Eigen::Ref<const Eigen::VectorXd> &v) const;

            /** \brief Try to find an owner for ambient point \x from among the
             * neighbors of this chart. Returns nullptr if none found.*/
            const AtlasChart *owningNeighbor(const Eigen::Ref<const Eigen::VectorXd> &x) const;

            /** \brief For manifolds of dimension 2, return in order in \a
             * vertices the polygon boundary of this chart, including an
             * approximation of the circular boundary where the polygon exceeds
             * radius \a rho_. Returns true if a circular portion is
             * included. */
            bool toPolygon(std::vector<Eigen::VectorXd> &vertices) const;

            /** \brief Returns the number of charts this chart shares a
             * halfspace boundary with. */
            std::size_t getNeighborCount() const
            {
                return polytope_.size();
            }

            /** \brief Use sampling to make a quick estimate as to whether this
             * chart's polytope boundary is completely defined by its
             * halfspaces. */
            bool estimateIsFrontier() const;

            /** \brief Create two complementary halfspaces dividing the space
             * between charts \a c1 and \a c2, and add them to the charts'
             * polytopes boundaries.
             * \note Charts must be different charts from the same atlas. */
            static void generateHalfspace(AtlasChart *c1, AtlasChart *c2);

        protected:
            /** \brief The constraint function that defines the manifold. */
            const Constraint *constraint_;

            /** \brief Set of halfspaces defining the polytope boundary. */
            std::vector<Halfspace *> polytope_;

            /** \brief Introduce a new \a halfspace to the chart's bounding
             * polytope. This chart assumes responsibility for deleting \a
             * halfspace. */
            void addBoundary(Halfspace *halfspace);

        private:
            /** \brief Dimension of the ambient space. */
            const unsigned int n_;

            /** \brief Dimension of the chart, which is the dimension of the
             * manifold. */
            const unsigned int k_;

            /** \brief Origin of the chart in ambient space coordinates. */
            const AtlasStateSpace::StateType *state_;

            /** \brief Basis for the chart space. */
            const Eigen::MatrixXd bigPhi_;

            /** \brief Maximum valid radius of this chart. */
            const double radius_;
        };
    }
}

#endif
