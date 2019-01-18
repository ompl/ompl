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
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <unordered_map>
#include "ompl/control/planners/PlannerIncludes.h"
#include "ompl/control/planners/syclop/Decomposition.h"
#include "ompl/control/planners/syclop/GridDecomposition.h"
#include "ompl/datastructures/PDF.h"
#include "ompl/util/Hash.h"
#include <functional>
#include <map>
#include <utility>
#include <vector>

namespace ompl
{
    namespace control
    {
        /**
           @anchor cSyclop
           @par Short description
           Syclop is a multi-layered planner that guides a low-level sampling-based tree planner
           through a sequence of sequence of discrete workspace regions from start to goal.
           Syclop is defined as an abstract base class whose pure virtual methods are defined
           by the chosen low-level sampling-based tree planner.
           @par External documentation
           E. Plaku, L.E. Kavraki, and M.Y. Vardi,
           Motion Planning with Dynamics by a Synergistic Combination of Layers of Planning,
           in <em>IEEE Transactions on Robotics</em>, 2010.
           DOI: [10.1109/TRO.2010.2047820](http://dx.doi.org/10.1109/TRO.2010.2047820)<br>
        */

        /** \brief Synergistic Combination of Layers of Planning. */
        class Syclop : public base::Planner
        {
        public:
            /** \brief Each edge weight between two adjacent regions in the Decomposition is defined
                as a product of edge cost factors. By default, given adjacent regions \f$r\f$ and \f$s\f$, Syclop uses
               the sole edge cost factor
                \f[
                    \frac{1 + \mbox{sel}^2(r,s)}{1 + \mbox{conn}^2(r,s)} \alpha(r) \alpha(s),
                \f]
                where for any region \f$t\f$,
                \f[
                    \alpha(t) = \frac{1}{\left(1 + \mbox{cov}(t)\right) \mbox{freeVol}^4(t)},
                \f]
                \f$\mbox{sel}(r,s)\f$ is the number of times \f$r\f$ and \f$s\f$ have been part of a lead or selected
               for exploration,
                \f$\mbox{conn}(r,s)\f$ estimates the progress made by the low-level planner in extending the tree from
               \f$r\f$ to \f$s\f$,
                \f$\mbox{cov}(t)\f$ estimates the tree coverage of the region \f$t\f$, and \f$\mbox{freeVol}(t)\f$
               estimates the free volume
                of \f$t\f$.
                Additional edge cost factors can be added
                with the addEdgeCostFactor() function, and Syclop's list of edge cost factors can be cleared using
               clearEdgeCostFactors() . */
            using EdgeCostFactorFn = std::function<double(int, int)>;

            /** \brief Leads should consist of a path of adjacent regions in the decomposition that start with the start
             * region and end at the end region.  Default is \f$A^\ast\f$ search. */
            using LeadComputeFn = std::function<void(int, int, std::vector<int> &)>;

            /** \brief Constructor. Requires a Decomposition, which Syclop uses to create high-level leads. */
            Syclop(const SpaceInformationPtr &si, DecompositionPtr d, const std::string &plannerName)
              : ompl::base::Planner(si, plannerName)
              , siC_(si.get())
              , decomp_(std::move(d))
              , covGrid_(Defaults::COVGRID_LENGTH, decomp_)
            {
                specs_.approximateSolutions = true;

                Planner::declareParam<int>("free_volume_samples", this, &Syclop::setNumFreeVolumeSamples,
                                           &Syclop::getNumFreeVolumeSamples, "10000:10000:500000");
                Planner::declareParam<int>("num_region_expansions", this, &Syclop::setNumRegionExpansions,
                                           &Syclop::getNumRegionExpansions, "10:10:500");
                Planner::declareParam<int>("num_tree_expansions", this, &Syclop::setNumTreeExpansions,
                                           &Syclop::getNumTreeExpansions, "0:1:100");
                Planner::declareParam<double>("prob_abandon_lead_early", this, &Syclop::setProbAbandonLeadEarly,
                                              &Syclop::getProbAbandonLeadEarly, "0.:.05:1.");
                Planner::declareParam<double>("prob_add_available_regions", this,
                                              &Syclop::setProbAddingToAvailableRegions,
                                              &Syclop::getProbAddingToAvailableRegions, "0.:.05:1.");
                Planner::declareParam<double>("prob_shortest_path_lead", this, &Syclop::setProbShortestPathLead,
                                              &Syclop::getProbShortestPathLead, "0.:.05:1.");
            }

            ~Syclop() override = default;

            /// @name ompl::base::Planner Interface
            /// @{

            void setup() override;

            void clear() override;

            /** \brief Continues solving until a solution is found or a given planner termination condition is met.
                Returns true if solution was found. */
            base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;
            /// @}

            /// @name Tunable parameters
            /// @{

            /** \brief Allows the user to override the lead computation function. */
            void setLeadComputeFn(const LeadComputeFn &compute);

            /** \brief Adds an edge cost factor to be used for edge weights between adjacent regions. */
            void addEdgeCostFactor(const EdgeCostFactorFn &factor);

            /** \brief Clears all edge cost factors, making all edge weights equivalent to 1. */
            void clearEdgeCostFactors();

            /// \brief Get the number of states to sample when estimating free volume in the Decomposition.
            int getNumFreeVolumeSamples() const
            {
                return numFreeVolSamples_;
            }

            /// \brief Set the number of states to sample when estimating free
            ///  volume in the Decomposition.
            void setNumFreeVolumeSamples(int numSamples)
            {
                numFreeVolSamples_ = numSamples;
            }

            /// \brief Get the probability [0,1] that a lead will be computed as
            ///  a shortest-path instead of a random-DFS.
            double getProbShortestPathLead() const
            {
                return probShortestPath_;
            }

            /// \brief Set the probability [0,1] that a lead will be computed as
            ///  a shortest-path instead of a random-DFS.
            void setProbShortestPathLead(double probability)
            {
                probShortestPath_ = probability;
            }

            /// \brief Get the probability [0,1] that the set of available
            ///  regions will be augmented.
            double getProbAddingToAvailableRegions() const
            {
                return probKeepAddingToAvail_;
            }

            /// \brief Set the probability [0,1] that the set of available
            ///  regions will be augmented.
            void setProbAddingToAvailableRegions(double probability)
            {
                probKeepAddingToAvail_ = probability;
            }

            /// \brief Get the number of times a new region will be chosen and
            ///  promoted for expansion from a given lead.
            int getNumRegionExpansions() const
            {
                return numRegionExpansions_;
            }

            /// \brief Set the number of times a new region will be chosen and
            ///  promoted for expansion from a given lead.
            void setNumRegionExpansions(int regionExpansions)
            {
                numRegionExpansions_ = regionExpansions;
            }

            /// \brief Get the number of calls to selectAndExtend() in the
            ///  low-level tree planner for a given lead and region.
            int getNumTreeExpansions() const
            {
                return numTreeSelections_;
            }

            /// \brief Set the number of calls to selectAndExtend() in the
            ///  low-level tree planner for a given lead and region.
            void setNumTreeExpansions(int treeExpansions)
            {
                numTreeSelections_ = treeExpansions;
            }

            /// \brief Get the probability [0,1] that a lead will be abandoned
            ///  early, before a new region is chosen for expansion.
            double getProbAbandonLeadEarly() const
            {
                return probAbandonLeadEarly_;
            }

            /// \brief The probability that a lead will be abandoned early,
            ///  before a new region is chosen for expansion.
            void setProbAbandonLeadEarly(double probability)
            {
                probAbandonLeadEarly_ = probability;
            }
            /// @}

            /** \brief Contains default values for Syclop parameters. */
            struct Defaults
            {
                static const int NUM_FREEVOL_SAMPLES = 100000;
                static const int COVGRID_LENGTH = 128;
                static const int NUM_REGION_EXPANSIONS = 100;
                static const int NUM_TREE_SELECTIONS = 1;
                // C++ standard prohibits non-integral static const member initialization
                // These constants are set in Syclop.cpp.  C++11 standard changes this
                // with the constexpr keyword, but for compatibility this is not done.
                static const double PROB_ABANDON_LEAD_EARLY /*= 0.25*/;
                static const double PROB_KEEP_ADDING_TO_AVAIL /*= 0.50*/;
                static const double PROB_SHORTEST_PATH /*= 0.95*/;
            };

        protected:
                       /** \brief Representation of a motion

                           A motion contains pointers to its state, its parent motion, and the control
                           that was applied to get from its parent to its state. */
            class Motion
            {
            public:
                Motion() = default;

                /** \brief Constructor that allocates memory for the state and the control */
                Motion(const SpaceInformation *si)
                  : state(si->allocState()), control(si->allocControl())
                {
                }
                virtual ~Motion() = default;
                /** \brief The state contained by the motion */
                base::State *state{nullptr};
                /** \brief The control contained by the motion */
                Control *control{nullptr};
                /** \brief The parent motion in the tree */
                const Motion *parent{nullptr};
                /** \brief The number of steps for which the control is applied */
                unsigned int steps{0};
            };

            /** \brief Representation of a region in the Decomposition assigned to Syclop. */
            class Region
            {
            public:
                Region() = default;
                virtual ~Region() = default;

                Region(const Region &) = default;
                Region &operator=(const Region &) = default;
                Region(Region &&) = default;
                Region &operator=(Region &&) = default;

                /** \brief Clears motions and coverage information from this region. */
                void clear()
                {
                    motions.clear();
                    covGridCells.clear();
                    pdfElem = nullptr;
                }

                /** \brief The cells of the underlying coverage grid that contain tree motions from this region */
                std::set<int> covGridCells;
                /** \brief The tree motions contained in this region */
                std::vector<Motion *> motions;
                /** \brief The volume of this region */
                double volume;
                /** \brief The free volume of this region */
                double freeVolume;
                /** \brief The percent of free volume of this region */
                double percentValidCells;
                /** \brief The probabilistic weight of this region, used when sampling from PDF */
                double weight;
                /** \brief The coefficient contributed by this region to edge weights in lead computations */
                double alpha;
                /** \brief The index of the graph node corresponding to this region */
                int index;
                /** \brief The number of times this region has been selected for expansion */
                unsigned int numSelections;
                /** \brief The Element corresponding to this region in the PDF of available regions. */
                PDF<int>::Element *pdfElem;
            };

            /** \brief Representation of an adjacency (a directed edge) between two regions
                in the Decomposition assigned to Syclop. */
            class Adjacency
            {
            public:
                Adjacency() = default;
                virtual ~Adjacency() = default;
                /** \brief Clears coverage information from this adjacency. */
                void clear()
                {
                    covGridCells.clear();
                }
                /** \brief The cells of the underlying coverage grid that contain tree motions originating from
                    direct connections along this adjacency */
                std::set<int> covGridCells;
                /** \brief The source region of this adjacency edge */
                const Region *source;
                /** \brief The target region of this adjacency edge */
                const Region *target;
                /** \brief The cost of this adjacency edge, used in lead computations */
                double cost;
                /** \brief The number of times this adjacency has been included in a lead */
                int numLeadInclusions;
                /** \brief The number of times the low-level tree planner has selected motions from the source region
                    when attempting to extend the tree toward the target region. */
                int numSelections;
                /** \brief This value is true if and only if this adjacency's source and target regions both contain
                 * zero tree motions. */
                bool empty;
            };

            /** \brief Add State s as a new root in the low-level tree, and return the Motion corresponding to s. */
            virtual Motion *addRoot(const base::State *s) = 0;

            /** \brief Select a Motion from the given Region, and extend the tree from the Motion.
                Add any new motions created to newMotions. */
            virtual void selectAndExtend(Region &region, std::vector<Motion *> &newMotions) = 0;

            /** \brief Returns a reference to the Region object with the given index. Assumes the index is valid. */
            inline const Region &getRegionFromIndex(const int rid) const
            {
                return graph_[boost::vertex(rid, graph_)];
            }

            /** \brief The number of states to sample to estimate free volume in the Decomposition. */
            int numFreeVolSamples_{Defaults::NUM_FREEVOL_SAMPLES};

            /** \brief The probability that a lead will be computed as a shortest-path instead of a random-DFS. */
            double probShortestPath_{Defaults::PROB_SHORTEST_PATH};

            /** \brief The probability that the set of available regions will be augmented. */
            double probKeepAddingToAvail_{Defaults::PROB_KEEP_ADDING_TO_AVAIL};

            /** \brief The number of times a new region will be chosen and promoted for expansion from a given lead. */
            int numRegionExpansions_{Defaults::NUM_REGION_EXPANSIONS};

            /** \brief The number of calls to selectAndExtend() in the low-level tree planner for a given lead and
             * region. */
            int numTreeSelections_{Defaults::NUM_TREE_SELECTIONS};

            /** \brief The probability that a lead will be abandoned early, before a new region is chosen for expansion.
             */
            double probAbandonLeadEarly_{Defaults::PROB_ABANDON_LEAD_EARLY};

            /** \brief Handle to the control::SpaceInformation object */
            const SpaceInformation *siC_;

            /** \brief The high level decomposition used to focus tree expansion */
            DecompositionPtr decomp_;

            /** \brief Random number generator */
            RNG rng_;

        private:
            /// @cond IGNORE
            /** \brief Hash function for std::pair<int,int> to be used in std::unordered_map */
            struct HashRegionPair
            {
                size_t operator()(const std::pair<int, int> &p) const
                {
                    std::size_t hash = std::hash<int>()(p.first);
                    hash_combine(hash, p.second);
                    return hash;
                }
            };
            /// @endcond

            /** \brief Syclop uses a CoverageGrid to estimate coverage in its assigned Decomposition.
                The CoverageGrid should have finer resolution than the Decomposition. */
            class CoverageGrid : public GridDecomposition
            {
            public:
                CoverageGrid(const int len, const DecompositionPtr &d)
                  : GridDecomposition(len, d->getDimension(), d->getBounds()), decomp(d)
                {
                }

                ~CoverageGrid() override = default;

                /** \brief Since the CoverageGrid is defined in the same space as the Decomposition,
                    it uses the Decomposition's projection function. */
                void project(const base::State *s, std::vector<double> &coord) const override
                {
                    decomp->project(s, coord);
                }

                /** \brief Syclop will not sample from the CoverageGrid. */
                void sampleFullState(const base::StateSamplerPtr & /*sampler*/, const std::vector<double> & /*coord*/,
                                     base::State * /*s*/) const override
                {
                }

            protected:
                const DecompositionPtr &decomp;
            };

            using RegionGraph = boost::adjacency_list<boost::vecS, boost::vecS, boost::directedS, Region, Adjacency>;
            using Vertex = boost::graph_traits<RegionGraph>::vertex_descriptor;
            using VertexIter = boost::graph_traits<RegionGraph>::vertex_iterator;
            using VertexIndexMap = boost::property_map<RegionGraph, boost::vertex_index_t>::type;
            using EdgeIter = boost::graph_traits<RegionGraph>::edge_iterator;

            /// @cond IGNORE
            friend class DecompositionHeuristic;

            class DecompositionHeuristic : public boost::astar_heuristic<RegionGraph, double>
            {
            public:
                DecompositionHeuristic(const Syclop *s, const Region &goal) : syclop(s), goalRegion(goal)
                {
                }

                double operator()(Vertex v)
                {
                    const Region &region = syclop->getRegionFromIndex(v);
                    return region.alpha * goalRegion.alpha;
                }

            private:
                const Syclop *syclop;
                const Region &goalRegion;
            };

            struct found_goal
            {
            };

            class GoalVisitor : public boost::default_astar_visitor
            {
            public:
                GoalVisitor(const int goal) : goalRegion(goal)
                {
                }
                void examine_vertex(Vertex v, const RegionGraph & /*g*/)
                {
                    if (static_cast<int>(v) == goalRegion)
                        throw found_goal();
                }

            private:
                const int goalRegion;
            };
            /// @endcond

            /// @cond IGNORE
            class RegionSet
            {
            public:
                int sampleUniform()
                {
                    if (empty())
                        return -1;
                    return regions.sample(rng.uniform01());
                }
                void insert(const int r)
                {
                    if (regToElem.count(r) == 0)
                        regToElem[r] = regions.add(r, 1);
                    else
                    {
                        PDF<int>::Element *elem = regToElem[r];
                        regions.update(elem, regions.getWeight(elem) + 1);
                    }
                }
                void clear()
                {
                    regions.clear();
                    regToElem.clear();
                }
                std::size_t size() const
                {
                    return regions.size();
                }
                bool empty() const
                {
                    return regions.empty();
                }

            private:
                RNG rng;
                PDF<int> regions;
                std::unordered_map<int, PDF<int>::Element *> regToElem;
            };
            /// @endcond

            /** \brief Initializes default values for a given Region. */
            void initRegion(Region &r);

            /** \brief Computes volume estimates for a given Region. */
            void setupRegionEstimates();

            /** \brief Recomputes coverage and selection estimates for a given Region. */
            void updateRegion(Region &r);

            /** \brief Initializes a given Adjacency between a source Region and a destination Region. */
            void initEdge(Adjacency &adj, const Region *source, const Region *target);

            /** \brief Initializes default values for each Adjacency. */
            void setupEdgeEstimates();

            /** \brief Updates the edge cost for a given Adjacency according to Syclop's list of edge cost factors. */
            void updateEdge(Adjacency &a);

            /** \brief Given that a State s has been added to the tree,
                update the coverage estimate (if needed) for its corresponding Region. */
            bool updateCoverageEstimate(Region &r, const base::State *s);

            /** \brief Given that an edge has been added to the tree of motions from a state in Region c to
                the State s in Region d, update the corresponding Adjacency's cost and connection estimates. */
            bool updateConnectionEstimate(const Region &c, const Region &d, const base::State *s);

            /** \brief Build a RegionGraph according to the Decomposition assigned to Syclop,
                creating Region and Adjacency objects for each node and edge. */
            void buildGraph();

            /** \brief Clear all Region and Adjacency objects in the graph. */
            void clearGraphDetails();

            /** \brief Select a Region in which to promote expansion of the low-level tree. */
            int selectRegion();

            /** \brief Compute the set of Regions available for selection. */
            void computeAvailableRegions();

            /** \brief Default lead computation. A lead is a sequence of adjacent Regions from start to goal in the
             * Decomposition. */
            void defaultComputeLead(int startRegion, int goalRegion, std::vector<int> &lead);

            /** \brief Default edge cost factor, which is used by Syclop for edge weights between adjacent Regions. */
            double defaultEdgeCost(int r, int s);

            /** \brief Lead computaton std::function object */
            LeadComputeFn leadComputeFn;
            /** \brief The current computed lead */
            std::vector<int> lead_;
            /** \brief Used to sample regions in which to promote expansion */
            PDF<int> availDist_;
            /** \brief Stores all factor functions used to compute adjacency edge cost for lead computation */
            std::vector<EdgeCostFactorFn> edgeCostFactors_;
            /** \brief An underlying grid used to estimate coverage */
            CoverageGrid covGrid_;
            /** \brief A graph structure whose nodes and edges correspond to regions and adjacencies in the given
             * Decomposition */
            RegionGraph graph_;
            /** \brief This value stores whether the graph structure has been built */
            bool graphReady_{false};
            /** \brief Maps pairs of regions to adjacency objects */
            std::unordered_map<std::pair<int, int>, Adjacency *, HashRegionPair> regionsToEdge_;
            /** \brief The total number of motions in the low-level tree */
            unsigned int numMotions_{0};
            /** \brief The set of all regions that contain start states */
            RegionSet startRegions_;
            /** \brief The set of all regions that contain goal states */
            RegionSet goalRegions_;
        };
    }
}

#endif
