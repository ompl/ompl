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

namespace ompl {
	namespace control {
		class Syclop : public base::Planner {
			public:

			/* Does the ordering of an initializer list determine the order of initializations, or is that up to the compiler? */
			Syclop(const SpaceInformationPtr &si, Decomposition &d) : base::Planner(si, "Syclop"), decomp(d), graph(decomp.getNumRegions()), covGrid(COVGRID_LENGTH, 2, d) {
			}

			virtual ~Syclop() {
			}

			virtual void setup(void) {
				base::Planner::setup();
				buildGraph();
				//TODO: locate start and goal states
				startRegion = 1;
				goalRegion = 2;
				setupRegionEstimates();
				updateRegionEstimates();
				updateEdgeEstimates();

				printRegions();
				printEdges();

				std::vector<Region*> lead;
				computeLead(lead);
				std::cerr << "Lead: ";
				for (int i = 0; i < lead.size(); ++i)
					std::cerr << lead[i] << " ";
				std::cerr << std::endl;
			}

			void printRegions(void) {
				for (int i = 0; i < decomp.getNumRegions(); ++i) {
					Region& r = graph[boost::vertex(i, graph)];
					std::cout << "Region " << r.index << ": ";
					std::cout << "nselects=" << r.numSelections << ",";
					std::cout << "vol=" << r.volume << ",";
					std::cout << "freeVol=" << r.freeVolume << ",";
					std::cout << "pcentValid=" << r.percentValidCells << ",";
					std::cout << "numCells=" << r.covGridCells.size() << ",";
					std::cout << "weight=" << r.weight << ",";
					std::cout << "alpha=" << r.alpha << "";
					std::cout << std::endl;
				}
			}

			void printEdges(void) {
				EdgeIter ei, end;
				VertexIndexMap index = get(boost::vertex_index, graph);
				for (boost::tie(ei,end) = boost::edges(graph); ei != end; ++ei) {
					const Adjacency& a = graph[*ei];
					std::cout << "Edge (" << index[boost::source(*ei, graph)] << "," << index[boost::target(*ei, graph)] << "): ";
					std::cout << "numCells=" << a.cells.size() << ",";
					std::cout << "nselects=" << a.numSelections << ",";
					std::cout << "cost=" << a.cost << std::endl;
				}
			}

			protected:			

			struct Region {
				int index;
				//std::set<base::State*> states;
				int numSelections;
				double volume;
				double freeVolume;
				double percentValidCells;
				double weight;
				double alpha;
				std::set<int> covGridCells;
				bool needUpdate; //maybe unnecessary
			};

			struct Adjacency {
				std::pair<Region*,Region*> regions; //unnecessary; just pull source() and target() from edge descriptor
				std::set<int> covGridCells;
				int numSelections;
				double cost;
			};

			//TODO Consider vertex/edge storage options other than vecS.
			typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, Region, Adjacency> RegionGraph;
			typedef boost::graph_traits<RegionGraph>::vertex_iterator VertexIter;
			typedef boost::property_map<RegionGraph, boost::vertex_index_t>::type VertexIndexMap;
			typedef boost::graph_traits<RegionGraph>::edge_iterator EdgeIter;

			static const int NUM_FREEVOL_SAMPLES = 10000;
			static const double PROB_SHORTEST_PATH = 1.0; //0.95
			static const int COVGRID_LENGTH = 8;

			/* Initialize edge between regions r and s. */
			virtual void initEdge(Adjacency& a, Region* r, Region* s) {
				a.regions.first = r;
				a.regions.second = s;
				a.numSelections = 0;
			}

			virtual void updateEdgeEstimates() {
				EdgeIter ei, end;
				VertexIndexMap index = get(boost::vertex_index, graph);
				for (boost::tie(ei,end) = boost::edges(graph); ei != end; ++ei) {
					Adjacency& a = graph[*ei];
					a.cost = (1 + a.numSelections*a.numSelections) / (1 + a.cells.size());
					a.cost *= a.regions.first->alpha * a.regions.second->alpha;
				}
			}

			// TODO: Consider moving this into Region constructor.
			virtual void initRegion(Region& r) {
				r.numSelections = 0;
				r.volume = 1.0;
				r.percentValidCells = 1.0;
				r.freeVolume = 1.0;
				r.needUpdate = false;
			}

			virtual void setupRegionEstimates() {
				std::vector<int> numTotal(decomp.getNumRegions(), 0);
				std::vector<int> numValid(decomp.getNumRegions(), 0);
				base::SpaceInformationPtr si = getSpaceInformation();
				base::StateValidityCheckerPtr checker = si->getStateValidityChecker();
				base::StateSamplerPtr sampler = si->allocStateSampler();
				base::State *s = si->allocState();
				for (int i = 0; i < NUM_FREEVOL_SAMPLES; ++i) {
					sampler->sampleUniform(s);
					int rid = decomp.locateRegion(s);
					if (checker->isValid(s))
						++numValid[rid];
					++numTotal[rid];
				}
				si->freeState(s);
				
				for (int i = 0; i < decomp.getNumRegions(); ++i) {
						Region& r = graph[boost::vertex(i, graph)];
						r.volume = decomp.getRegionVolume(i);
						r.percentValidCells = ((double) numValid[i]) / numTotal[i];
						r.freeVolume = r.percentValidCells * r.volume;
				}
			}

			/* Given that State s has been added to the tree and belongs in Region r,
				update r's coverage estimate if needed. */
			virtual void updateCoverageEstimate(Region& r, const base::State *s) {
				const int covCell = covGrid.locateRegion(s);
				r.covGridCells.insert(covCell);
			}

			/* Given that an edge has been added to the tree, leading to the new state s,
				update the corresponding edge's connection estimates. */
			virtual void updateConnectionEstimate(Adjacency& a, const base::State *s) {
				const int covCell = covGrid.locateRegion(s);
				a.covGridCells.insert(covCell);
			}

			virtual void updateRegionEstimates() {
				for (int i = 0; i < decomp.getNumRegions(); ++i) {
					Region& r = graph[boost::vertex(i, graph)];
					const double f = r.freeVolume*r.freeVolume*r.freeVolume*r.freeVolume;
					r.alpha = 1 / ((1 + r.covGridCells.size()) * f);
					r.weight = f / ((1 + r.covGridCells.size())*(1 + r.numSelections*r.numSelections));
				}
			}

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
					initRegion(graph[*vi]);
					graph[*vi].index = index[*vi];

					/* Create an edge between this vertex and each of its neighboring regions in the decomposition,
						and initialize the edge's Adjacency object. */
					decomp.getNeighbors(index[*vi], neighbors);
					for (std::vector<int>::const_iterator j = neighbors.begin(); j != neighbors.end(); ++j) {
						RegionGraph::edge_descriptor edge;
						bool ignore;
						boost::tie(edge, ignore) = boost::add_edge(*vi, boost::vertex(*j,graph), graph);
						initEdge(graph[edge], &graph[*vi], &graph[boost::vertex(*j,graph)]);
					}
					neighbors.clear();
				}
			}

			virtual void computeLead(std::vector<Region*>& lead) {
				if (rng.uniform01() < PROB_SHORTEST_PATH) {
					std::vector<RegionGraph::vertex_descriptor> parents(decomp.getNumRegions());
					std::vector<double> distances(decomp.getNumRegions());
					boost::dijkstra_shortest_paths(graph, boost::vertex(startRegion, graph),
						boost::weight_map(get(&Adjacency::cost, graph)).distance_map(
							boost::make_iterator_property_map(distances.begin(), get(boost::vertex_index, graph)
						)).predecessor_map(
							boost::make_iterator_property_map(parents.begin(), get(boost::vertex_index, graph))
						)
					);
					int region = goalRegion;
					int leadLength = 1;
					while (region != startRegion) {
						region = parents[region];
						++leadLength;
					}
					lead.resize(leadLength);
					region = goalRegion;
					for (int i = leadLength-1; i >= 0; --i) {
						lead[i] = &graph[boost::vertex(region, graph)];
						region = parents[region];
					}
				}
			}

			virtual Region* selectRegion(const std::set<Region*>& regions) {
				return NULL;
			}

			/* Finding available regions depends on the tree of the low-level planner. */
			virtual void computeAvailableRegions(const std::vector<Region*>& lead, std::set<Region*>& avail) = 0;

			class CoverageGrid : public GridDecomposition {
				public:
				CoverageGrid(const int len, const int dim, Decomposition& d) : GridDecomposition(len,dim,d.getBounds()), decomp(d) {
				}

				virtual ~CoverageGrid() {
				}

				virtual void stateToCoord(const base::State *s, std::vector<double>& coord) {
					decomp.stateToCoord(s,coord);
				}

				virtual int locateRegion(const base::State *s) {
					std::vector<double> coord;
					stateToCoord(s, coord);
					return GridDecomposition::locateRegion(coord);
				}

				protected:
				Decomposition& decomp;
			};

			Decomposition &decomp;
			RegionGraph graph;
			CoverageGrid covGrid;
			RNG rng;
			int startRegion;
			int goalRegion;
		};
	}
}

#endif
