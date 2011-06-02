#ifndef SYCLOP_H
#define SYCLOP_H

#define BOOST_NO_HASH
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/graphviz.hpp>
#include <boost/graph/adjacency_list.hpp>
#include "ompl/control/planners/PlannerIncludes.h"
#include "ompl/control/planners/syclop/Decomposition.h"

namespace ompl {
	namespace control {
		class Syclop : public base::Planner {
			public:

			/* Does the ordering of an initializer list determine the order of initializations, or is that up to the compiler? */
			Syclop(const SpaceInformationPtr &si, Decomposition &d) : base::Planner(si, "Syclop"), decomp(d), graph(decomp.getNumRegions()) {
			}

			virtual ~Syclop() {
			}

			virtual void setup(void) {
				base::Planner::setup();
				buildGraph();
			}

			virtual bool solve(const base::PlannerTerminationCondition &ptc) {
				return false;
			}

			protected:			

			class Region {
				public:	
				std::set<base::State*> states;
				int numSelections;
				int coverage;
				std::set<int> coverageCells;
				double volume;
				double freeVolume;
				int numValid;
				int numInvalid;

				Region() : numSelections(0), coverage(0), numValid(0), numInvalid(0) {
				}
				virtual ~Region() {
				}
			};

			class Adjacency {
				public:
				int ncells;
				int selections;

				Adjacency() : ncells(0), selections(0) {
				}
				virtual ~Adjacency() {
				}
			};
			
			typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, Region, Adjacency> RegionGraph;
			typedef boost::graph_traits<RegionGraph>::vertex_iterator VertexIter;
			typedef boost::property_map<RegionGraph, boost::vertex_index_t>::type VertexIndexMap;
			typedef boost::graph_traits<RegionGraph>::edge_iterator EdgeIter;
			

			/* Sets up RegionGraph from decomposition. */
			virtual void buildGraph(void) {
				/* The below code builds a boost::graph corresponding to the decomposition.
					It creates Region and Adjacency property objects for each vertex and edge.
					TODO: Correctly initialize the fields for each Region and Adjacency object. */
				VertexIndexMap index = get(boost::vertex_index, graph);
				VertexIter vi, vend;
				std::vector<int> neighbors;
				//Iterate over all vertices.
				for (boost::tie(vi,vend) = boost::vertices(graph); vi != vend; ++vi) {
					/* Initialize this vertex's Region object. */
					graph[*vi].volume = -1;

					/* Create an edge between this vertex and each of its neighboring regions in the decomposition,
						and initialize the edge's Adjacency object. */
					decomp.getNeighbors(index[*vi], neighbors);
					for (std::vector<int>::const_iterator j = neighbors.begin(); j != neighbors.end(); ++j) {
						RegionGraph::edge_descriptor edge;
						bool ignore;
						boost::tie(edge, ignore) = boost::add_edge(*vi, vertex(*j,graph), graph);
						graph[edge].ncells = 1;
					}
					neighbors.clear();
				}
			}

			virtual void computeLead(std::vector<Region*>& lead) {
				
			}

			virtual Region* selectRegion(const std::set<Region*>& regions);

			/* Finding available regions depends on the tree of the low-level planner. */
			virtual void computeAvailableRegions(const std::vector<Region*>& lead, std::set<Region*>& avail) = 0;			

			Decomposition &decomp;
			RegionGraph graph;
		};
	}
}

#endif
