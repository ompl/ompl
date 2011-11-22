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

#include <boost/graph/astar_search.hpp>
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
#include <vector>

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

           @par Planner parameters
           - free_volume_samples: Number of states to sample when estimating free volume in the decomposition.
           - num_region_expansions: Number of times a new region will be chosen and expanded from a single lead.
           - num_tree_expansions: Number of times the tree is expanded in a specific decomposition region.
           - prob_abandon_lead_early: Probability that a lead will be abandoned early.
           - prob_add_available_regions: Probability that the set of available regions will be updated after each lead computation.
           - prob_shortest_path_lead: Probability that a lead will be computed as the shortest-path instead of random.
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

            /** \brief Constructor. Requires a Decomposition, which Syclop uses to create high-level leads. */
            Syclop(const SpaceInformationPtr& si, DecompositionPtr& d, const std::string& plannerName) : ompl::base::Planner(si, plannerName),
                numFreeVolSamples_(Defaults::NUM_FREEVOL_SAMPLES),
                probShortestPath_(Defaults::PROB_SHORTEST_PATH),
                probKeepAddingToAvail_(Defaults::PROB_KEEP_ADDING_TO_AVAIL),
                numRegionExpansions_(Defaults::NUM_REGION_EXPANSIONS),
                numTreeSelections_(Defaults::NUM_TREE_SELECTIONS),
                probAbandonLeadEarly_(Defaults::PROB_ABANDON_LEAD_EARLY),
                siC_(si.get()),
                decomp_(d),
                covGrid_(Defaults::COVGRID_LENGTH, decomp_),
                graphReady_(false),
                numMotions_(0)
            {
                specs_.approximateSolutions = true;

                Planner::declareParam<int>   ("free_volume_samples", this, &Syclop::setNumFreeVolumeSamples, &Syclop::getNumFreeVolumeSamples);
                Planner::declareParam<int>   ("num_region_expansions", this, &Syclop::setNumRegionExpansions, &Syclop::getNumRegionExpansions);
                Planner::declareParam<int>   ("num_tree_expansions", this, &Syclop::setNumTreeExpansions, &Syclop::getNumTreeExpansions);
                Planner::declareParam<double>("prob_abandon_lead_early", this, &Syclop::setProbAbandonLeadEarly, &Syclop::getProbAbandonLeadEarly);
                Planner::declareParam<double>("prob_add_available_regions", this, &Syclop::setProbAddingToAvailableRegions, &Syclop::getProbAddingToAvailableRegions);
                Planner::declareParam<double>("prob_shortest_path_lead", this, &Syclop::setProbShortestPathLead, &Syclop::getProbShortestPathLead);
            }

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

            /// @name Tunable parameters
            /// @{
            
            /// \brief Get the number of states to sample when estimating free volume in the Decomposition.
            int getNumFreeVolumeSamples (void) const
            {
                return numFreeVolSamples_;
            }

            /// \brief Set the number of states to sample when estimating free
            ///  volume in the Decomposition.
            void setNumFreeVolumeSamples (int numSamples)
            {
                numFreeVolSamples_ = numSamples;
            }

            /// \brief Get the probability [0,1] that a lead will be computed as
            ///  a shortest-path instead of a random-DFS.
            double getProbShortestPathLead (void) const
            {
                return probShortestPath_;
            }

            /// \brief Set the probability [0,1] that a lead will be computed as
            ///  a shortest-path instead of a random-DFS.
            void setProbShortestPathLead (double probability)
            {
                probShortestPath_ = probability;
            }

            /// \brief Get the probability [0,1] that the set of available
            ///  regions will be augmented.
            double getProbAddingToAvailableRegions (void) const
            {
                return probKeepAddingToAvail_;
            }

            /// \brief Set the probability [0,1] that the set of available
            ///  regions will be augmented.
            void setProbAddingToAvailableRegions (double probability)
            {
                probKeepAddingToAvail_ = probability;
            }

            /// \brief Get the number of times a new region will be chosen and
            ///  promoted for expansion from a given lead.
            int getNumRegionExpansions (void) const
            {
                return numRegionExpansions_;
            }

            /// \brief Set the number of times a new region will be chosen and
            ///  promoted for expansion from a given lead.
            void setNumRegionExpansions (int regionExpansions)
            {
                numRegionExpansions_ = regionExpansions;
            }

            /// \brief Get the number of calls to selectAndExtend() in the
            ///  low-level tree planner for a given lead and region.
            int getNumTreeExpansions (void) const
            {
                return numTreeSelections_;
            }

            /// \brief Set the number of calls to selectAndExtend() in the
            ///  low-level tree planner for a given lead and region.
            void setNumTreeExpansions (int treeExpansions)
            {
                numTreeSelections_ = treeExpansions;
            }

            /// \brief Get the probability [0,1] that a lead will be abandoned
            ///  early, before a new region is chosen for expansion.
            double getProbAbandonLeadEarly (void) const
            {
                return probAbandonLeadEarly_;
            }

            /// \brief The probability that a lead will be abandoned early,
            ///  before a new region is chosen for expansion.
            void setProbAbandonLeadEarly (double probability)
            {
                probAbandonLeadEarly_ = probability;
            }
            /// @}

            /** \brief Contains default values for Syclop parameters. */
            struct Defaults
            {
                static const int    NUM_FREEVOL_SAMPLES         = 100000;
                static const int    COVGRID_LENGTH              = 128;
                static const int    NUM_REGION_EXPANSIONS       = 100;
                static const int    NUM_TREE_SELECTIONS         = 50;
                static const double PROB_ABANDON_LEAD_EARLY     = 0.25;
                static const double PROB_KEEP_ADDING_TO_AVAIL   = 0.95;
                static const double PROB_SHORTEST_PATH          = 0.95;
            };

        protected:

            #pragma pack(push, 4)  // push default byte alignment to stack and align the following structure to 4 byte boundary
            /** \brief Representation of a motion

                A motion contains pointers to its state, its parent motion, and the control
                that was applied to get from its parent to its state. */
            class Motion
            {
            public:
                Motion(void) : state(NULL), control(NULL), parent(NULL), steps(0)
                {
                }
                /** \brief Constructor that allocates memory for the state and the control */
                Motion(const SpaceInformation* si) : state(si->allocState()), control(si->allocControl()), parent(NULL), steps(0)
                {
                }
                virtual ~Motion(void)
                {
                }
                /** \brief The state contained by the motion */
                base::State* state;
                /** \brief The control contained by the motion */
                Control* control;
                /** \brief The parent motion in the tree */
                const Motion* parent;
                /** \brief The number of steps for which the control is applied */
                int steps;
            };
            #pragma pack (pop)  // Restoring default byte alignment

            #pragma pack(push, 4) // push default byte alignment to stack and align the following structure to 4 byte boundary
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

                std::set<int> covGridCells;
                std::vector<Motion*> motions;
                double volume;
                double freeVolume;
                double percentValidCells;
                double weight;
                double alpha;
                int index;
                int numSelections;
            };
            #pragma pack (pop)  // Restoring default byte alignment

            #pragma pack(push, 4)  // push default byte alignment to stack and align the following structure to 4 byte boundary
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
                const Region* source;
                const Region* target;
                double cost;
                int numLeadInclusions;
                int numSelections;
                bool empty;
            };
            #pragma pack (pop) // Restoring default byte alignment

            /** \brief Initialize the low-level tree rooted at State s, and return the Motion corresponding to s. */
            virtual Motion* initializeTree(const base::State* s) = 0;

            /** \brief Select a Motion from the given Region, and extend the tree from the Motion.
                Add any new motions created to newMotions. */
            virtual void selectAndExtend(Region& region, std::vector<Motion*>& newMotions) = 0;

            /** \brief Returns a reference to the Region object with the given index. Assumes the index is valid. */
            inline const Region& getRegionFromIndex(const int rid) const
            {
                return graph_[boost::vertex(rid,graph_)];
            }

            /** \brief The number of states to sample to estimate free volume in the Decomposition. */
            int numFreeVolSamples_;

            /** \brief The probability that a lead will be computed as a shortest-path instead of a random-DFS. */
            double probShortestPath_;

            /** \brief The probability that the set of available regions will be augmented. */
            double probKeepAddingToAvail_;

            /** \brief The number of times a new region will be chosen and promoted for expansion from a given lead. */
            int numRegionExpansions_;

            /** \brief The number of calls to selectAndExtend() in the low-level tree planner for a given lead and region. */
            int numTreeSelections_;

            /** \brief The probability that a lead will be abandoned early, before a new region is chosen for expansion. */
            double probAbandonLeadEarly_;

            /** \brief Handle to the control::SpaceInformation object */
            const SpaceInformation* siC_;

            /** \brief The high level decomposition used to focus tree expansion */
            DecompositionPtr decomp_;

            /** \brief Random number generator */
            RNG rng_;

        private:
            /** \brief Syclop uses a CoverageGrid to estimate coverage in its assigned Decomposition.
                The CoverageGrid should have finer resolution than the Decomposition. */
            class CoverageGrid : public GridDecomposition
            {
            public:
                CoverageGrid(const int len, DecompositionPtr& d) : GridDecomposition(len, d->getDimension(), d->getBounds()), decomp(d)
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
                virtual void sampleFromRegion(const int rid, base::StateSamplerPtr& sampler, base::State* s)
                {
                }

            protected:
                DecompositionPtr& decomp;
            };

            typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::directedS, Region, Adjacency> RegionGraph;
            typedef boost::graph_traits<RegionGraph>::vertex_descriptor Vertex;
            typedef boost::graph_traits<RegionGraph>::vertex_iterator VertexIter;
            typedef boost::property_map<RegionGraph, boost::vertex_index_t>::type VertexIndexMap;
            typedef boost::graph_traits<RegionGraph>::edge_iterator EdgeIter;

            /// @cond IGNORE
            friend class DecompositionHeuristic;

            class DecompositionHeuristic : public boost::astar_heuristic<RegionGraph, double>
            {
            public:
                DecompositionHeuristic(const Syclop* s, const Region& goal) : syclop(s), goalRegion(goal)
                {
                }

                double operator()(Vertex v)
                {
                    const Region& region = syclop->getRegionFromIndex(v);
                    return region.weight*goalRegion.weight;
                }
            private:
                const Syclop* syclop;
                const Region& goalRegion;
            };

            struct found_goal {};

            class GoalVisitor : public boost::default_astar_visitor
            {
            public:
                GoalVisitor(const unsigned int goal) : goalRegion(goal)
                {
                }
                void examine_vertex(Vertex v, const RegionGraph& g)
                {
                    if (v == goalRegion)
                        throw found_goal();
                }
            private:
                const unsigned int goalRegion;
            };
            /// @endcond

            /** \brief Initializes default values for a given Region. */
            void initRegion(Region& r);

            /** \brief Computes volume estimates for a given Region. */
            void setupRegionEstimates(void);

            /** \brief Recomputes coverage and selection estimates for a given Region. */
            void updateRegion(Region& r);

            /** \brief Initializes a given Adjacency between a source Region and a destination Region. */
            void initEdge(Adjacency& a, const Region* source, const Region* target);

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
            void computeLead(int startRegion, int goalRegion);

            /** \brief Select a Region in which to promote expansion of the low-level tree. */
            int selectRegion(void);

            /** \brief Compute the set of Regions available for selection. */
            void computeAvailableRegions(void);

            /** \brief Default edge cost factor, which is used by Syclop for edge weights between adjacent Regions. */
            double defaultEdgeCost(int r, int s);

            std::vector<int> lead_;
            PDF<int> availDist_;
            std::vector<EdgeCostFactorFn> edgeCostFactors_;
            CoverageGrid covGrid_;
            RegionGraph graph_;
            bool graphReady_;
            boost::unordered_map<std::pair<int,int>, Adjacency*> regionsToEdge_;
            unsigned int numMotions_;
        };
    }
}

#endif
