#include <ompl/base/ScopedState.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/State.h>
#include <ompl/control/planners/syclop/SyclopRRT.h>
#include <ompl/control/planners/syclop/GridDecomposition.h>
#include <ompl/util/RandomNumbers.h>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/graphviz.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <iostream>
#include <vector>

namespace ob = ompl::base;

class TestDecomposition : public ompl::GridDecomposition {
	public:
	TestDecomposition(const int length, ob::RealVectorBounds &bounds) : ompl::GridDecomposition(length, bounds) {
	}
	virtual ~TestDecomposition() {
	}

	virtual int locateRegion(const ob::State *s) {
		const ob::CompoundState *cs = s->as<ob::CompoundState>();
		const ob::SE2StateSpace::StateType *ws = cs->as<ob::SE2StateSpace::StateType>(0);
		std::vector<double> coord(2);
		coord[0] = ws->getX();
		coord[1] = ws->getY();
		return GridDecomposition::locateRegion(coord);
	}
};

struct NodeEstimate {
	int id;
	double weight;
};

struct EdgeEstimate {
	double weight;
};

void createGraphs(void) {
	typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::directedS,
		NodeEstimate, EdgeEstimate> Graph;
	const int n = 5;
	Graph g(n);
	typedef boost::graph_traits<Graph>::vertex_iterator vertex_iter;
	ompl::RNG randGen;
	int i = 0;
	for (std::pair<vertex_iter,vertex_iter> vp = boost::vertices(g); vp.first != vp.second; ++vp.first) {
		g[*vp.first].id = i++;
		g[*vp.first].weight = 0.25;
		if (vp.first+1 != vp.second) {
			for (vertex_iter v = vp.first; v != vp.second; ++v) {
				std::pair<Graph::edge_descriptor,bool> ep = boost::add_edge(*vp.first, *v, g);
				g[ep.first].weight = randGen.uniformReal(1, 10);
			}
		}
	}

	/* Get the property map for vertex indices. */
	typedef boost::property_map<Graph, boost::vertex_index_t>::type IndexMap;
	IndexMap index = get(boost::vertex_index, g);
	
	for (std::pair<vertex_iter,vertex_iter> vp = boost::vertices(g); vp.first != vp.second; ++vp.first)
		std::cerr << index[*vp.first] << " ";
	std::cerr << std::endl;

	/* Iterate over edges. */
	typedef boost::graph_traits<Graph>::edge_iterator edge_iter;
	for (std::pair<edge_iter, edge_iter> ep = boost::edges(g); ep.first != ep.second; ++ep.first) {
		std::cerr << "(" << index[boost::source(*ep.first, g)] << ",";
		std::cerr << index[boost::target(*ep.first, g)] << ")[" << g[*ep.first].weight << "]" << std::endl;
	}
	std::cerr << std::endl;
	//boost::write_graphviz(std::cout, g);
	std::vector<Graph::vertex_descriptor> parents(boost::num_vertices(g));
	std::vector<double> distances(boost::num_vertices(g));
	boost::dijkstra_shortest_paths(g, *boost::vertices(g).first,
		boost::weight_map(get(&EdgeEstimate::weight, g)).distance_map(
			boost::make_iterator_property_map(distances.begin(), get(boost::vertex_index, g)
		)).predecessor_map(
			boost::make_iterator_property_map(parents.begin(), get(boost::vertex_index, g))
		)
	);
	vertex_iter vi, vend;
	IndexMap idMap = get(&NodeEstimate::id, g);
	for (boost::tie(vi,vend) = boost::vertices(g); vi != vend; ++vi) {
		std::cerr << "distance[" << idMap[*vi] << "] = " << distances[idMap[*vi]] << std::endl;
		std::cerr << "parents[" << idMap[*vi] << "] = " << parents[idMap[*vi]] << std::endl;
	}
}

int main(void) {
	ompl::base::RealVectorBounds bounds(2);
	bounds.setLow(-1);
	bounds.setHigh(1);
	TestDecomposition grid(4, bounds);

	ob::StateSpacePtr manifold(new ob::CompoundStateSpace());
	ob::StateSpacePtr locSpace(new ob::SE2StateSpace());
	ob::StateSpacePtr velSpace(new ob::RealVectorStateSpace(1));
	manifold->as<ob::CompoundStateSpace>()->addSubSpace(locSpace, 0.8);
	manifold->as<ob::CompoundStateSpace>()->addSubSpace(velSpace, 0.2);

	ob::ScopedState<ob::CompoundStateSpace> init(manifold);
	ob::SE2StateSpace::StateType *se = init->as<ob::SE2StateSpace::StateType>(0);
	se->setX(-0.75);
	se->setY(0.8);
	se->setYaw(0);
	init->as<ob::RealVectorStateSpace::StateType>(1)->values[0] = 0;

	ob::ScopedState<ob::CompoundStateSpace> goal(init);
	se = goal->as<ob::SE2StateSpace::StateType>(0);
	se->setX(0.65);
	se->setY(-0.7);

	int initRegion = grid.locateRegion(init.get());
	std::cerr << "initial state located in region " << initRegion << std::endl;
	int goalRegion = grid.locateRegion(goal.get());
	std::cerr << "goal state located in region " << goalRegion << std::endl;

	createGraphs();
	
	return 0;
}
