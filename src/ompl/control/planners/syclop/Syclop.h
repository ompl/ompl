#ifndef SYCLOP_H
#define SYCLOP_H

#define BOOST_NO_HASH
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/graphviz.hpp>
#include <boost/graph/adjacency_list.hpp>
#include "ompl/control/planners/PlannerIncludes.h"
#include "ompl/control/planners/syclop/Decomposition.h"
#include "ompl/control/planners/syclop/GridDecomposition.h"
#include "ompl/datastructures/PDF.h"
#include <map>

namespace ompl
{
    namespace control
    {
        //TODO documentation, triangular decomposition, fit syntax to style guide
        class Syclop : public base::Planner
        {
        public:
            Syclop(const SpaceInformationPtr& si, Decomposition& d, const std::string& name) : ompl::base::Planner(si, name),
                siC_(si.get()), decomp(d), graph(decomp.getNumRegions()), covGrid(COVGRID_LENGTH, 2, d)
            {
            }
            virtual ~Syclop()
            {
            }
            virtual void setup(void);
            virtual void clear(void);
            virtual bool solve(const base::PlannerTerminationCondition& ptc);

            void printRegions(void);
            void printEdges(void);

        protected:
            static const int NUM_FREEVOL_SAMPLES = 100000;
            static const double PROB_SHORTEST_PATH = 0.95; //0.95
            static const int COVGRID_LENGTH = 512;
            static const double PROB_KEEP_ADDING_TO_AVAIL = 0.95; //0.875
            static const int NUM_AVAIL_EXPLORATIONS = 100; //100
            static const int NUM_TREE_SELECTIONS = 50; //50
            static const double PROB_ABANDON_LEAD_EARLY = 0.25; //0.05

            struct Motion
            {
                Motion(void) : state(NULL), control(NULL), steps(0), parent(NULL)
                {
                }
                Motion(const SpaceInformation* si) : state(si->allocState()), control(si->allocControl()), steps(0), parent(NULL)
                {
                }
                ~Motion(void)
                {
                }
                base::State* state;
                Control* control;
                std::size_t steps;
                Motion* parent;
            };

            struct Region
            {
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

            struct Adjacency
            {
                std::set<int> covGridCells;
                bool empty;
                int numLeadInclusions;
                int numSelections;
                double cost;
            };

            class CoverageGrid : public GridDecomposition
            {
            public:
                CoverageGrid(const int len, const int dim, Decomposition& d) : GridDecomposition(len,dim,d.getBounds()), decomp(d)
                {
                }

                virtual ~CoverageGrid()
                {
                }

                virtual void stateToCoord(const base::State* s, std::vector<double>& coord)
                {
                    decomp.stateToCoord(s,coord);
                }

                virtual int locateRegion(const base::State* s)
                {
                    std::vector<double> coord;
                    stateToCoord(s, coord);
                    return GridDecomposition::locateRegion(coord);
                }

            protected:
                Decomposition& decomp;
            };

            //TODO Consider vertex/edge storage options other than vecS.
            typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::directedS, Region, Adjacency> RegionGraph;
            typedef boost::graph_traits<RegionGraph>::vertex_iterator VertexIter;
            typedef boost::property_map<RegionGraph, boost::vertex_index_t>::type VertexIndexMap;
            typedef boost::graph_traits<RegionGraph>::edge_iterator EdgeIter;

            /* Initialize edge between regions r and s. */
            void initEdge(Adjacency& a, Region* r, Region* s);
            void updateEdgeEstimates(void);
            // TODO: Consider moving this into Region constructor.
            void initRegion(Region& r);
            void setupRegionEstimates(void);
            /* Given that State s has been added to the tree and belongs in Region r,
                update r's coverage estimate if needed. */
            bool updateCoverageEstimate(Region& r, const base::State* s);
            /* Given that an edge has been added to the tree, leading to the new state s,
                update the corresponding edge's connection estimates. */
            bool updateConnectionEstimate(const Region& c, const Region& d, const base::State* s);
            void updateRegionEstimates(void);
            /* Sets up RegionGraph from decomposition. */
            void buildGraph(void);
            void computeLead(void);
            int selectRegion(void);
            void computeAvailableRegions(void);
            // temporary replacement for boost::dijkstra
            void dijkstra(std::vector<int>& parents, std::vector<double>& dist);

            /* Initialize a tree rooted at start state s; return the Motion corresponding to s. */
            virtual Motion* initializeTree(const base::State* s) = 0;
            /* Select a vertex v from region, extend tree from v, add any new motions created to newMotions. */
            virtual void selectAndExtend(Region& region, std::set<Motion*>& newMotions) = 0;

            const SpaceInformation* siC_;
            Decomposition &decomp;
            RegionGraph graph;
            //instead of a map, consider holding Adjacency* in Motion object
            std::map<std::pair<int,int>, Adjacency*> regionsToEdge;
            CoverageGrid covGrid;
            RNG rng;
            int startRegion;
            int goalRegion;
            std::vector<int> lead;
            std::set<int> avail;
            PDF<int> availDist;
        };
    }
}

#endif
