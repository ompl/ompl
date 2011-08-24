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
           @anchor Syclop

           @par Short description
           Syclop is a multi-layered planner that guides a low-level sampling-based tree planner
           through a sequence of sequence of workspace regions from start to goal.

           @par External documentation
           E. Plaku, L.E. Kavraki, and M.Y. Vardi,
           Motion Planning with Dynamics by a Synergistic Combination of Layers of Planning,
           in <em>IEEE Transactions on Robotics</em>, 2010.<br>
           <a href="http://kavrakilab.org/node/737">Abstract</a>
        */

        /** \brief Synergistic Combination of Layers of Planning.
            This is a base class which requires a low-level planner. */
        class Syclop : public base::Planner
        {
        public: //TODO documentation, triangular decomposition, fit syntax to style guide
            /** \brief Constructor. Requires a Decomposition,
                which Syclop uses to create high-level guides. */
            typedef boost::function2<double, int, int> EdgeCostFactorFn;

            Syclop(const SpaceInformationPtr& si, DecompositionPtr& d, const std::string& name) : ompl::base::Planner(si, name),
                siC_(si.get()), decomp_(d), graphReady_(false), covGrid_(COVGRID_LENGTH, 2, d)
            {
                specs_.approximateSolutions = true;
            }
            virtual ~Syclop()
            {
            }
            virtual void setup(void);
            virtual void clear(void);
            virtual bool solve(const base::PlannerTerminationCondition& ptc);
            void addEdgeCostFactor(const EdgeCostFactorFn& factor);
            void clearEdgeCostFactors(void);

            /// read-only access to the most recently computed lead.
            std::vector<int> getLead();

        protected:
            static const int NUM_FREEVOL_SAMPLES = 100000;
            static const double PROB_SHORTEST_PATH = 0.95; //0.95
            static const int COVGRID_LENGTH = 512;
            static const double PROB_KEEP_ADDING_TO_AVAIL = 0.95; //0.875
            static const int NUM_AVAIL_EXPLORATIONS = 100; //100
            static const int NUM_TREE_SELECTIONS = 50; //50
            static const double PROB_ABANDON_LEAD_EARLY = 0.25; //0.05

            class Motion
            {
            public:
                Motion(void) : state(NULL), control(NULL), steps(0), parent(NULL)
                {
                }
                Motion(const SpaceInformation* si) : state(si->allocState()), control(si->allocControl()), steps(0), parent(NULL)
                {
                }
                virtual ~Motion(void)
                {
                }
                base::State* state;
                Control* control;
                std::size_t steps;
                Motion* parent;
            };

            class Region
            {
            public:
                Region(void)
                {
                }
                virtual ~Region(void)
                {
                }
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

            class Adjacency
            {
            public:
                Adjacency(void)
                {
                }
                virtual ~Adjacency(void)
                {
                }
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

            //TODO Consider vertex/edge storage options other than vecS.
            typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::directedS, Region, Adjacency> RegionGraph;
            typedef boost::graph_traits<RegionGraph>::vertex_iterator VertexIter;
            typedef boost::property_map<RegionGraph, boost::vertex_index_t>::type VertexIndexMap;
            typedef boost::graph_traits<RegionGraph>::edge_iterator EdgeIter;

            void initRegion(Region& r);
            void setupRegionEstimates(void);
            void updateRegion(Region& r);

            void initEdge(Adjacency& a, Region* r, Region* s);
            void setupEdgeEstimates(void);
            void updateEdge(Adjacency& a);

            /* Given that State s has been added to the tree and belongs in Region r,
                update r's coverage estimate if needed. */
            bool updateCoverageEstimate(Region& r, const base::State* s);
            /* Given that an edge has been added to the tree, leading to the new state s,
                update the corresponding edge's connection estimates. */
            bool updateConnectionEstimate(const Region& c, const Region& d, const base::State* s);

            /* Sets up RegionGraph from decomposition. */
            void buildGraph(void);
            void initGraph(void);
            void clearGraphDetails(void);

            void computeLead(void);
            int selectRegion(void);
            void computeAvailableRegions(void);

            /* Initialize a tree rooted at start state s; return the Motion corresponding to s. */
            virtual Motion* initializeTree(const base::State* s) = 0;
            /* Select a vertex v from region, extend tree from v, add any new motions created to newMotions. */
            virtual void selectAndExtend(Region& region, std::set<Motion*>& newMotions) = 0;

            const SpaceInformation* siC_;
            DecompositionPtr decomp_;
            RegionGraph graph_;
            boost::unordered_map<std::pair<int,int>, Adjacency*> regionsToEdge_;
            RNG rng_;
            int startRegion_;
            int goalRegion_;
            std::vector<int> lead_;
            std::set<int> avail_;
            PDF<int> availDist_;
            std::vector<EdgeCostFactorFn> edgeCostFactors_;
            bool graphReady_;

        private:
            class CoverageGrid : public GridDecomposition
            {
            public:
                CoverageGrid(const int len, const int dim, DecompositionPtr& d) : GridDecomposition(len,dim,d->getBounds()), decomp(d)
                {
                }

                virtual ~CoverageGrid()
                {
                }

                virtual void project(const base::State* s, std::valarray<double>& coord) const
                {
                    decomp->project(s, coord);
                }

            protected:
                DecompositionPtr& decomp;
            };

            double defaultEdgeCost(int r, int s);

            CoverageGrid covGrid_;
        };
    }
}

#endif
