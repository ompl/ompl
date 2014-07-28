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

/* Author: Caleb Voss */

#ifndef OMPL_BASE_SPACE_ATLAS_CHART_
#define OMPL_BASE_SPACE_ATLAS_CHART_

#include "ompl/base/spaces/AtlasStateSpace.h"

#include <list>

#include <eigen3/Eigen/LU>

namespace ompl
{
    namespace base
    {
        /** \brief Tangent space for use as a projection of a manifold patch. */
        class AtlasChart : private boost::noncopyable
        {
            /** \brief Information defining a halfspace in which a chart may be valid. */
            class LinearInequality
            {
            public:
                
                /** \brief Constructor using the chart \a c owning the inequality and a neighboring chart,
                 * \a neighbor. Inequality inferred from centers of \a c and \a neighbor. */
                LinearInequality (const AtlasChart &c, const AtlasChart &neighbor);
                
                /** \brief Constructor using the chart owning the inequality \a c and an explicit point \a u
                 * on \a c. Inequality inferred from center of \a c and point \a u. */
                LinearInequality (const AtlasChart &c, const Eigen::VectorXd &u);
                
                /** \brief Set the linear inequality which complements this one (though not exactly
                 * because it would lie on a different chart). */
                void setComplement (LinearInequality *const complement);
                
                /** \brief Get the complementary inequality. Returns NULL if none. */
                LinearInequality *getComplement (void) const;
                
                /** \brief Get the chart to which this inequality belongs. */
                const AtlasChart &getOwner (void) const;
                
                /** \brief Return whether point \a v on the owning chart satisfies the inequality. */
                bool accepts (const Eigen::VectorXd &v) const;
                
                /** \brief If point \a v on the owning chart is too close to this inequality, ask
                 * the complementary inequality to relax in order to accept \a v when projected onto
                 * the neighboring chart. */
                void checkNear (const Eigen::VectorXd &v) const;
                
                /** \brief Compute up to two vertices of intersection with a circle of radius \a r.
                 * If one vertex is found, it is stored to both \a v1 and \a v2; if two are found, they are
                 * stored to \a v1 and \a v2. If no vertex is found, returns false; otherwise returns true. */
                bool circleIntersect (const double r, Eigen::VectorXd &v1, Eigen::VectorXd &v2) const;
                
                /** \brief Compute the vertex of intersection of two 1-dimensional inequalities \a l1 and \a l2. */
                static Eigen::VectorXd intersect (const LinearInequality &l1, const LinearInequality &l2);
                
            private:
                
                /** \brief Chart to which this inequality belongs. */
                const AtlasChart &owner_;
                
                /** \brief Inequality accepting the halfspace complementary to this one, but on
                 * the neighboring chart. */
                LinearInequality *complement_;
                
                /** \brief Center of the neighboring chart which owns the complement, projected
                 * onto our chart. */
                Eigen::VectorXd u_;
                
                /** \brief Precomputed right-hand side of the inequality. */
                double rhs_;
                
                /** \brief Set the point on our chart that generates the inequality. We will divide the
                 * space in half between \a u and the origin. */
                void setU (const Eigen::VectorXd &u);
                
                /** \brief Compute the distance between a point \a v on our chart and the nearest point
                 * on this linear inequality as a scalar factor of u_. */
                double distanceToPoint (const Eigen::VectorXd &v) const;
                
                /** \brief Adjust the inequality to include ambient point \a x when it is projected
                 * onto our chart. */
                void expandToInclude (const Eigen::VectorXd &x);
            };
            
        public:
            
            /** \brief Constructor; \a atlas is the atlas to which it belongs, and \a xorigin
             * is the ambient space point on the manifold at which the chart will be centered. */
            AtlasChart (const AtlasStateSpace &atlas, const Eigen::VectorXd &xorigin);
            
            /** \brief Destructor. */
            virtual ~AtlasChart (void);
            
            /** \brief Write a chart point \a u in ambient space coordinates. */
            Eigen::VectorXd phi (const Eigen::VectorXd &u) const;
            
            /** \brief Exponential mapping; projects chart point \a u onto the manifold. */
            Eigen::VectorXd psi (const Eigen::VectorXd &u) const;
            
            /** \brief Logarithmic mapping; projects ambient point \a x onto the chart. */
            Eigen::VectorXd psiInverse (const Eigen::VectorXd &x) const;
            
            /** \brief Check if a point \a u on the chart lies within its polytope P. If \a solitary
             * is not NULL, perform a thorough check to find solitary volations. If only one
             * of the linear inequalities is violated, its index is returned in \a solitary; otherwise
             * \a solitary is set to the total number of linear inequalities. LinearInequalities
             * \a ignore1 and \a ignore2, if specified, are ignored during the check.
             */
            virtual bool inP (const Eigen::VectorXd &u, std::size_t *const solitary = NULL,
                              const LinearInequality *const ignore1 = NULL, const LinearInequality *const ignore2 = NULL) const;
            
            /** \brief Check if chart point \a v lies too close to any linear inequality. When it does,
             * expand the neighboring chart's polytope. */
            virtual void borderCheck (const Eigen::VectorXd &v) const;
            
            /** \brief Track that this chart owns \a state. Assumes we are not already tracking it. */
            void own (ompl::base::AtlasStateSpace::StateType *const state) const;
            
            /** \brief Stop tracking \a state. Assumes it is listed at most once. */
            void disown (ompl::base::AtlasStateSpace::StateType *const state) const;
            
            /** \brief Check each of our neighboring charts to see if ambient point \a x lies within its
             * polytope when projected onto it. Returns NULL if none. */
            virtual const AtlasChart *owningNeighbor (const Eigen::VectorXd &x) const;
            
            /** \brief Perform calculations to approximate the measure of this chart. */
            virtual void approximateMeasure (void);
            
            /** \brief Get the measure (k_-dimensional volume) of this chart. */
            double getMeasure (void) const;
            
            /** \brief Get this chart's unique identifier in its atlas. Same as its index in the atlas'
             * vector of charts. */
            unsigned int getID (void) const;
            
            /** \brief If the manifold dimension is 2, compute the sequence of vertices for the polygon of this
             * chart and return them in \a vertices, in order. */
            void toPolygon (std::vector<Eigen::VectorXd> &vertices) const;
            
            /** \brief Create two complementary linear inequalities dividing the space between charts \a c1 and \a c2,
             * and add them to the charts' polytopes. */
            static void generateHalfspace (AtlasChart &c1, AtlasChart &c2);
            
        protected:
            
            /** \brief Atlas to which this chart belongs. */
            const AtlasStateSpace &atlas_;
            
            /** \brief Measure of the convex polytope P. */
            double measure_;
            
            /** \brief Set of linear inequalities defining the polytope P. */
            std::list<LinearInequality *> bigL_;
            
            /** \brief Introduce a new linear inequality \a halfspace to bound the polytope P. Updates
             * approximate measure and prune redundant inequalities. This chart assumes
             * responsibility for deleting \a halfspace. If \a halfspace is NULL, it is not added. */
            virtual void addBoundary (LinearInequality *const halfspace = NULL);
            
        private:
            
            /** \brief Dimension of the ambient space. */
            const unsigned int n_;
            
            /** \brief Dimension of the chart, i.e. the dimension of the manifold. */
            const unsigned int k_;
            
            /** \brief Origin of the chart in ambient space coordinates. */
            const Eigen::VectorXd xorigin_;
            
            /** \brief Unique ID in the atlas. */
            const unsigned int id_;
            
            /** \brief Basis for the chart space. */
            Eigen::MatrixXd bigPhi_;
            
            /** \brief Transpose of basis. */
            Eigen::MatrixXd bigPhi_t_;
            
            /** \brief List of states on this chart. */
            mutable std::list<ompl::base::AtlasStateSpace::StateType *> owned_;
            
            /** \brief Compare the angles \a v1 and \a v2 make with the origin. */
            bool angleCompare (const Eigen::VectorXd &v1, const Eigen::VectorXd &v2) const;
            
            /** \brief Enables pruning of redundant linear inequalities. */
            const bool pruning;
        };
    }
}

#endif
