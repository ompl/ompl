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

/* Author: Caleb Voss */

#ifndef OMPL_BASE_SPACES_ATLAS_CHART_
#define OMPL_BASE_SPACES_ATLAS_CHART_

#include "ompl/base/spaces/AtlasStateSpace.h"

#include <vector>

#include <eigen3/Eigen/LU>

namespace ompl
{
    namespace base
    {
        /** \brief Tangent space and bounding polytope approximating some patch
         * of the manifold. Has dimension k. */
        class AtlasChart : private boost::noncopyable
        {
            /** \brief Halfspace equation on a chart.
             * \note Use AtlasChart::generateHalfspace to create new halfspace
             * objects. Since each halfspace is associated to exactly one chart,
             * we let the chart be responsible for deleting it. */
            class Halfspace
            {
            public:
                
                /** \brief Create a halfspace equitably separating charts \a
                 * owner and \a neighbor. This halfspace will coincide with
                 * chart \a owner. */
                Halfspace (const AtlasChart &owner, const AtlasChart &neighbor);
                
                /** \brief Inform this halfspace about the "complementary"
                 * halfspace which coincides with the neighboring chart. */
                void setComplement (Halfspace *complement);
                
                /** \brief Get the complementary halfspace. */
                Halfspace *getComplement (void) const;
                
                /** \brief Get the chart to which this halfspace belongs. */
                const AtlasChart &getOwner (void) const;
                
                /** \brief Return whether point \a v on the owning chart
                 * lies within the halfspace. */
                bool contains (Eigen::Ref<const Eigen::VectorXd> v) const;
                
                /** \brief If point \a v on the owning chart is very close to
                 * the halfspace boundary, the "complementary" halfspace will
                 * extend its boundary so that it also contains \a v when \a v
                 * is projected onto the neighboring chart. */
                void checkNear (Eigen::Ref<const Eigen::VectorXd> v) const;

                /// @cond IGNORE
                
                /** \brief Compute up to two vertices of intersection with a
                 * circle of radius \a r.  If one vertex is found, it is stored
                 * to both \a v1 and \a v2; if two are found, they are stored to
                 * \a v1 and \a v2. If no vertex is found, returns false;
                 * otherwise returns true. */
                bool circleIntersect (const double r,
                                      Eigen::Ref<Eigen::VectorXd> v1,
                                      Eigen::Ref<Eigen::VectorXd> v2) const;
                
                /** \brief Compute the vertex of intersection of two
                 * 1-dimensional inequalities \a l1 and \a l2.  Result stored in
                 * \a out, which should be allocated to a size of 2. */
                static void intersect (const Halfspace &l1, const Halfspace &l2,
                                       Eigen::Ref<Eigen::VectorXd> out);

                /// @endcond
                
            private:
                
                /** \brief Chart to which this halfspace belongs. */
                const AtlasChart &owner_;
                
                /** \brief Halfspace complementary to this one, but on the
                 * neighboring chart. */
                Halfspace *complement_;
                
                /** \brief Center of the neighboring chart projected onto our
                 * chart. */
                Eigen::VectorXd u_;
                
                /** \brief Precomputed right-hand side of the inequality. */
                double rhs_;
                
                /** \brief Generate the linear inequality. We will divide the
                 * space in half between \a u and 0, and 0 will lie inside. */
                void setU (const Eigen::VectorXd &u);
                
                /** \brief Compute the distance between a point \a v on our
                 * chart and the halfspace boundary as a scalar factor of
                 * \a u_. That is, \a result * \a u_ lies on the halfspace
                 * boundary, and \a v, \a u_, \a result * \a u_ are colinear. */
                double distanceToPoint (Eigen::Ref<const Eigen::VectorXd> v) const;
                
                /** \brief Expand the halfspace to include ambient point \a x
                 * when it is projected onto our chart. */
                void expandToInclude (Eigen::Ref<const Eigen::VectorXd> x);
            };
            
        public:
            
            /** \brief Create a tangent space chart for \a atlas with center at
             * ambient space point \a xorigin. Use \a isAnchor to specify if
             * this is a special chart (e.g. start or goal is on it) that should
             * persist through a call to AtlasStateSpace::clear().
             * \throws ompl::Exception when manifold seems degenerate here. */
            AtlasChart (const AtlasStateSpace &atlas,
                        Eigen::Ref<const Eigen::VectorXd> xorigin,
                        const bool isAnchor = false);
            
            /** \brief Destructor. */
            virtual ~AtlasChart (void);
            
            /** \brief Forget all acquired information such as the halfspace boundary. */
            void clear (void);
            
            /** \brief Returns phi(0), the center of the chart in ambient space. */
            Eigen::Ref<const Eigen::VectorXd> getXorigin (void) const;
            
            /** \brief Returns the pointer to the internal xorigin variable. */
            const Eigen::VectorXd *getXoriginPtr (void) const;
            
            /** \brief Write a chart point \a u in ambient space coordinates. Result stored in \a out,
             * which should be allocated to a size of ambient dimension. */
            void phi (Eigen::Ref<const Eigen::VectorXd> u, Eigen::Ref<Eigen::VectorXd> out) const;
            
            /** \brief Same as psi, except it starts from a point in ambient that is hopefully already close to the manifold. */
            void psiFromGuess (Eigen::Ref<const Eigen::VectorXd> x_0, Eigen::Ref<Eigen::VectorXd> out) const;
            
            /** \brief Exponential mapping; projects chart point \a u onto the manifold. Result stored in \a out,
             * which should be allocated to a size of ambient dimension. */
            void psi (Eigen::Ref<const Eigen::VectorXd> u, Eigen::Ref<Eigen::VectorXd> out) const;
            
            /** \brief Logarithmic mapping; projects ambient point \a x onto the chart. Result stored in \a out,
             * which should be allocated to a size of manifold dimension.*/
            void psiInverse (Eigen::Ref<const Eigen::VectorXd> x, Eigen::Ref<Eigen::VectorXd> out) const;
            
            /** \brief Check if a point \a u on the chart lies within its polytope P. LinearInequalities
             * \a ignore1 and \a ignore2, if specified, are ignored during the check. */
            virtual bool inP (Eigen::Ref<const Eigen::VectorXd> u, const Halfspace *const ignore1 = nullptr, const Halfspace *const ignore2 = nullptr) const;
            
            /** \brief Check if chart point \a v lies too close to any linear inequality. When it does,
             * expand the neighboring chart's polytope. */
            virtual void borderCheck (Eigen::Ref<const Eigen::VectorXd> v) const;
            
            /** \brief Check each of our neighboring charts to see if ambient point \a x lies within its
             * polytope when projected onto it. Returns nullptr if none. */
            virtual const AtlasChart *owningNeighbor (Eigen::Ref<const Eigen::VectorXd> x) const;
            
            /** \brief Set this chart's unique identifier in its atlas. Needs to be its index in the atlas'
             * vector of charts. */
            void setID (unsigned int);
            
            /** \brief Get this chart's unique identifier in its atlas. Same as its index in the atlas'
             * vector of charts. */
            unsigned int getID (void) const;
            
            /** \brief Is this chart marked as an anchor chart for the atlas? */
            bool isAnchor (void) const;

            /// @cond IGNORE
            /** \brief For manifolds of dimension 2, return in order in \a
             * vertices the polygon boundary of this chart, including an
             * approximation of the circular boundary where the polygon exceeds
             * radius \a rho_. Returns true if a circular portion is
             * included. */
            bool toPolygon (std::vector<Eigen::VectorXd> &vertices) const;

            /** \brief Use sampling to make a quick estimate as to whether this
             * chart's polytope boundary is completely defined by its
             * halfspaces. */
            bool estimateIsFrontier () const;
            /// @cond IGNORE
            
            /** \brief Create two complementary linear inequalities dividing the space between charts \a c1 and \a c2,
             * and add them to the charts' polytopes. */
            static void generateHalfspace (AtlasChart &c1, AtlasChart &c2);
            
            /** \brief Compute the distance between the centers of the two charts. */
            static double distanceBetweenCenters (AtlasChart *c1, AtlasChart *c2);
            
        protected:
            
            /** \brief Atlas to which this chart belongs. */
            const AtlasStateSpace &atlas_;
            
            /** \brief Set of linear inequalities defining the polytope P. */
            std::vector<Halfspace *> bigL_;
            
            /** \brief Introduce a new linear inequality \a halfspace to bound the polytope P. Updates
             * approximate measure. This chart assumes responsibility for deleting \a halfspace. */
            void addBoundary (Halfspace &halfspace);
            
        private:
            
            /** \brief Dimension of the ambient space. */
            const unsigned int n_;
            
            /** \brief Dimension of the chart, i.e. the dimension of the manifold. */
            const unsigned int k_;
            
            /** \brief Origin of the chart in ambient space coordinates. */
            const Eigen::VectorXd xorigin_;
            
            /** \brief Maximum valid radius of this chart. */
            const double radius_;

            /** \brief Whether this chart is an anchor chart in the atlas. */
            const bool isAnchor_;
            
            /** \brief Unique ID in the atlas. Must be manually set. */
            unsigned int id_;
            
            /** \brief Basis for the chart space. */
            Eigen::MatrixXd bigPhi_;

            /// @cond IGNORE
            /** \brief Compare the angles \a v1 and \a v2 make with the origin. */
            bool angleCompare (Eigen::Ref<const Eigen::VectorXd> v1, Eigen::Ref<const Eigen::VectorXd> v2) const;
            /// @endcond
        };
    }
}

#endif
