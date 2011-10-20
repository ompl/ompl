/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2011, Rice University
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

/* Author: Matt Maly */

#ifndef OMPL_CONTROL_PLANNERS_SYCLOP_SYCLOP_
#define OMPL_CONTROL_PLANNERS_SYCLOP_SYCLOP_

#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/graphviz.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/unordered_map.hpp>
#include "ompl/control/planners/PlannerIncludes.h"
#include "ompl/control/planners/syclop/Decomposition.h"
#include "ompl/control/planners/syclop/GridDecomposition.h"
#include "ompl/datastructures/PDF.h"
#include <map>

namespace ompl
{
    namespace control
    {
        /**
           @anchor cSyclop

           @par Short description
           Syclop is a multi-layered planner that guides a low-level sampling-based tree planner
           through a sequence of sequence of workspace regions from start to goal.
           Syclop is defined as an abstract base class whose pure virtual methods are defined
           by the chosen low-level sampling-based tree planner.

           @par External documentation
           E. Plaku, L.E. Kavraki, and M.Y. Vardi,
           Motion Planning with Dynamics by a Synergistic Combination of Layers of Planning,
           in <em>IEEE Transactions on Robotics</em>, 2010.<br>
           <a href="http://kavrakilab.org/node/737">Abstract</a>
        */

        /** \brief Synergistic Combination of Layers of Planning. */
        class Syclop : public base::Planner
        {
        public:
            /** \brief Each edge weight between two adjacent regions in the Decomposition is defined
                as a product of edge cost factors. By default, given adjacent regions \f$r\f$ and \f$s\f$, Syclop uses the sole edge cost factor
                \f[
                    \frac{1 + \mbox{sel}^2(r,s)}{1 + \mbox{conn}^2(r,s)} \alpha(r) \alpha(s),
                \f]
                where for any region \f$t\f$,
                \f[
                    \alpha(t) = \frac{1}{\left(1 + \mbox{cov}(t)\right) \mbox{freeVol}^4(t)},
                \f]
                \f$\mbox{sel}(r,s)\f$ is the number of times \f$r\f$ and \f$s\f$ have been part of a lead or selected for exploration,
                \f$\mbox{conn}(r,s)\f$ estimates the progress made by the low-level planner in extending the tree from \f$r\f$ to \f$s\f$,
                \f$\mbox{cov}(t)\f$ estimates the tree coverage of the region \f$t\f$, and \f$\mbox{freeVol}(t)\f$ estimates the free volume
                of \f$t\f$.
                Additional edge cost factors can be added
                with the addEdgeCostFactor() function, and Syclop's list of edge cost factors can be cleared using clearEdgeCostFactors() . */
            typedef boost::function2<double, int, int> EdgeCostFactorFn;

            virtual ~Syclop()
            {
            }
            virtual void setup(void);
            virtual void clear(void);

            /** \brief Continues solving until a solution is found or a given planner termination condition is met.
                Returns true if solution was found. */
            virtual bool solve(const base::PlannerTerminationCondition& ptc);

            /** \brief Adds an edge cost factor to be used for edge weights between adjacent regions. */
            void addEdgeCostFactor(const EdgeCostFactorFn& factor);

            /** \brief Clears all edge cost factors, making all edge weights equivalent to 1. */
            void clearEdgeCostFactors(void);

            /** \brief Returns a copy of the most recently-computed high-level lead. */
            std::vector<int> getLead();

            /** \brief The number of states to sample to estimate free volume in the Decomposition. */
            int numFreeVolSamples;

            /** \brief The probability that a lead will be computed as a shortest-path instead of a random-DFS. */
            double probShortestPath;

            /** \brief The probability that the set of available regions will be augmented. */
            double probKeepAddingToAvail;

            /** \brief The number of times a new region will be chosen and promoted for expansion from a given lead. */
            int numAvailExplorations;

            /** \brief The number of calls to selectAndExtend() in the low-level tree planner for a given lead and region. */
            int numTreeSelections;

            /** \brief The probability that a lead will be abandoned early, before a new region is chosen for expansion. */
            double probAbandonLeadEarly;

        protected:
            /** \brief Contains default values for Syclop parameters. */
            struct Defaults
            {
                static const int NUM_FREEVOL_SAMPLES = 100000;
                static const double PROB_SHORTEST_PATH = 0.95;
                static const int COVGRID_LENGTH = 512;
                static const double PROB_KEEP_ADDING_TO_AVAIL = 0.95;
                static const int NUM_AVAIL_EXPLORATIONS = 100;
                static const int NUM_TREE_SELECTIONS = 50;
                static const double PROB_ABANDON_LEAD_EARLY = 0.25;
            };

            /** \brief Representation of a motion

                A motion contains pointers to its state, its parent motion, and the control
                that was applied to get from its parent to its state. */
            class Motion
            {
            public:
                Motion(void) : state(NULL), control(NULL), steps(0), parent(NULL)
                {
                }
                /** \brief Constructor that allocates memory for the state and the control */
                Motion(const SpaceInformation* si) : state(si->allocState()), control(si->allocControl()), steps(0), parent(NULL)
                {
                }
                virtual ~Motion(void)
                {
                }
                /** \brief The state contained by the motion */
                base::State* state;
                /** \brief The control contained by the motion */
                Control* control;
                /** \brief The number of steps for which the control is applied */
                std::size_t steps;
                /** \brief The parent motion in the tree */
                Motion* parent;
            };

            /** \brief Representation of a region in the Decomposition assigned to Syclop. */
            class Region
            {
            public:
                Region(void)
                {
                }
                virtual ~Region(void)
                {
                }
                /** \brief Clears motions and coverage information from this region. */
                void clear(void)
                {
                    motions.clear();
                    covGridCells.clear();
                }
                std::vector<Motion*> motions;
                int index;
                int numSelections;
                double volume;
                double freeVolume;
                double percentValidCells;
                double weight;
                double alpha;
                std::set<int> covGridCells;
            };

            /** \brief Representation of an adjacency (or edge) between two regions
                in the Decomposition assigned to Syclop. */
            class Adjacency
            {
            public:
                Adjacency(void)
                {
                }
                virtual ~Adjacency(void)
                {
                }
                /** \brief Clears coverage information from this adjacency. */
                void clear(void)
                {
                    covGridCells.clear();
                }
                std::set<int> covGridCells;
                bool empty;
                int numLeadInclusions;
                int numSelections;
                double cost;
                Region* source;
                Region* target;
            };

            /** \brief Constructor. Requires a Decomposition, which Syclop uses to create high-level leads. */
            Syclop(const SpaceInformationPtr& si, DecompositionPtr& d, const std::string& plannerName) : ompl::base::Planner(si, plannerName),
                numFreeVolSamples(Defaults::NUM_FREEVOL_SAMPLES),
                probShortestPath(Defaults::PROB_SHORTEST_PATH),
                probKeepAddingToAvail(Defaults::PROB_KEEP_ADDING_TO_AVAIL),
                numAvailExplorations(Defaults::NUM_AVAIL_EXPLORATIONS),
                numTreeSelections(Defaults::NUM_TREE_SELECTIONS),
                probAbandonLeadEarly(Defaults::PROB_ABANDON_LEAD_EARLY),
                siC_(si.get()),
                decomp_(d),
                covGrid_(Defaults::COVGRID_LENGTH, 2, decomp_),
                graphReady_(false)
            {
                specs_.approximateSolutions = true;
            }

            /** \brief Returns a reference to the Region object with the given index. Assumes the index is valid. */
            Region& getRegionFromIndex(const int rid);

            /** \brief Initialize the low-level tree rooted at State s, and return the Motion corresponding to s. */
            virtual Motion* initializeTree(const base::State* s) = 0;

            /** \brief Select a Motion from the given Region, and extend the tree from the Motion.
                Add any new motions created to newMotions. */
            virtual void selectAndExtend(Region& region, std::set<Motion*>& newMotions) = 0;

            const SpaceInformation* siC_;
            DecompositionPtr decomp_;
            RNG rng_;
            int startRegion_;
            int goalRegion_;

        private:
            /** \brief Syclop uses a CoverageGrid to estimate coverage in its assigned Decomposition.
                The CoverageGrid should have finer resolution than the Decomposition. */
            class CoverageGrid : public GridDecomposition
            {
            public:
                CoverageGrid(const int len, const int dim, DecompositionPtr& d) : GridDecomposition(len,dim,d->getBounds()), decomp(d)
                {
                }

                virtual ~CoverageGrid()
                {
                }

                /** \brief Since the CoverageGrid is defined in the same space as the Decomposition,
                    it uses the Decomposition's projection function. */
                virtual void project(const base::State* s, std::valarray<double>& coord) const
                {
                    decomp->project(s, coord);
                }

                /** \brief Syclop will not sample from the CoverageGrid. */
                virtual void sampleFromRegion(const int rid, base::StateSamplerPtr& sampler, base::State* s) const
                {
                }

            protected:
                DecompositionPtr& decomp;
            };

            typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::directedS, Region, Adjacency> RegionGraph;
            typedef boost::graph_traits<RegionGraph>::vertex_iterator VertexIter;
            typedef boost::property_map<RegionGraph, boost::vertex_index_t>::type VertexIndexMap;
            typedef boost::graph_traits<RegionGraph>::edge_iterator EdgeIter;

            /** \brief Initializes default values for a given Region. */
            void initRegion(Region& r);

            /** \brief Computes volume estimates for a given Region. */
            void setupRegionEstimates(void);

            /** \brief Recomputes coverage and selection estimates for a given Region. */
            void updateRegion(Region& r);

            /** \brief Initializes a given Adjacency between a source Region and a destination Region. */
            void initEdge(Adjacency& a, Region* source, Region* target);

            /** \brief Initializes default values for each Adjacency. */
            void setupEdgeEstimates(void);

            /** \brief Updates the edge cost for a given Adjacency according to Syclop's list of edge cost factors. */
            void updateEdge(Adjacency& a);

            /** \brief Given that a State s has been added to the tree,
                update the coverage estimate (if needed) for its corresponding Region. */
            bool updateCoverageEstimate(Region& r, const base::State* s);

            /** \brief Given that an edge has been added to the tree of motions from a state in Region c to
                the State s in Region d, update the corresponding Adjacency's cost and connection estimates. */
            bool updateConnectionEstimate(const Region& c, const Region& d, const base::State* s);

            /** \brief Build a RegionGraph according to the Decomposition assigned to Syclop,
                creating Region and Adjacency objects for each node and edge. */
            void buildGraph(void);

            /** \brief Initialize default values for Region and Adjacency objects in the RegionGraph.
                Initialize the low-level tree with the start state from the problem definition. */
            void initGraph(void);

            /** \brief Clear all Region and Adjacency objects in the graph. */
            void clearGraphDetails(void);

            /** \brief Computes a lead, which is a sequence of adjacent Regions from start to goal in the Decomposition. */
            void computeLead(void);

            /** \brief Select a Region in which to promote expansion of the low-level tree. */
            int selectRegion(void);

            /** \brief Compute the set of Regions available for selection. */
            void computeAvailableRegions(void);

            /** \brief Default edge cost factor, which is used by Syclop for edge weights between adjacent Regions. */
            double defaultEdgeCost(int r, int s);

            std::vector<int> lead_;
            std::set<int> avail_;
            PDF<int> availDist_;
            std::vector<EdgeCostFactorFn> edgeCostFactors_;
            CoverageGrid covGrid_;
            RegionGraph graph_;
            bool graphReady_;
            boost::unordered_map<std::pair<int,int>, Adjacency*> regionsToEdge_;
        };
    }
}

#endif
