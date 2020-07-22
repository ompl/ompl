#include <ompl/multilevel/planners/explorer/datastructures/PathSpace.h>
#include <ompl/multilevel/datastructures/PlannerDataVertexAnnotated.h>
#include <ompl/multilevel/planners/explorer/datastructures/PathVisibilityChecker.h>
#include <boost/foreach.hpp>

#define foreach BOOST_FOREACH

using namespace ompl::multilevel;

ompl::multilevel::PathSpace::PathSpace(BundleSpaceGraph* bundleSpaceGraph):
  bundleSpaceGraph_(bundleSpaceGraph)
{
}
ompl::multilevel::PathSpace::~PathSpace()
{
    if(pathVisibilityChecker_)
    {
        delete pathVisibilityChecker_;
    }
}
PathVisibilityChecker* PathSpace::getPathVisibilityChecker()
{
    if(!pathVisibilityChecker_)
    {
        pathVisibilityChecker_ = new PathVisibilityChecker(bundleSpaceGraph_->getBundle());
    }
    return pathVisibilityChecker_;
}

unsigned int ompl::multilevel::PathSpace::getSelectedPath()
{
    return selectedPath_;
}
void ompl::multilevel::PathSpace::setSelectedPath(unsigned int selectedPath)
{
    selectedPath_ = selectedPath;
}

void ompl::multilevel::PathSpace::updatePath(unsigned int k, VertexPath p, double cost)
{
    if(k >= criticalPaths_.size()) return;
    criticalPaths_.at(k) = p;
    criticalPathsCost_.at(k) = cost;
}

void ompl::multilevel::PathSpace::addPath(VertexPath p, double cost)
{
    criticalPaths_.push_back(p);
    criticalPathsCost_.push_back(cost);
}

double ompl::multilevel::PathSpace::getPathCost(unsigned int k) const
{
    if(k < criticalPathsCost_.size())
    {
        return criticalPathsCost_.at(k);
    }else{
        throw Exception("Path does not exist.");
    }
}

std::vector<BundleSpaceGraph::Vertex>& ompl::multilevel::PathSpace::getCriticalPath(unsigned int k)
{
    if(k < criticalPaths_.size())
    {
        return criticalPaths_.at(k);
    }else{
        throw Exception("Path does not exist.");
    }
}

unsigned int ompl::multilevel::PathSpace::getNumberOfPaths() const
{
    return criticalPaths_.size();
}

void ompl::multilevel::PathSpace::getPathIndices(
    const std::vector<BundleSpaceGraph::Vertex> &vertices, 
    std::vector<int> &idxPath) const
{
    if (!bundleSpaceGraph_->hasParent())
    {
        std::reverse(idxPath.begin(), idxPath.end());
        return;
    }
    else
    {
      idxPath.push_back(0);
        // parent->getPathIndices(vertices, idxPath);
    }
}

void ompl::multilevel::PathSpace::getPlannerData(base::PlannerData &data, BundleSpaceGraph* bundleGraph) const
{
    BundleSpaceGraph::Graph graph = bundleGraph->getGraph();
    base::SpaceInformationPtr si = bundleGraph->getBundle();

    std::vector<int> idxPath;
    if (criticalPaths_.size() > 0)
    {
        OMPL_DEVMSG1("%s has %d solutions.", bundleGraph->getName().c_str(), criticalPaths_.size());

        for (uint i = 0; i < criticalPaths_.size(); i++)
        {
            const std::vector<BundleSpaceGraph::Vertex> vertices = criticalPaths_.at(i);

            idxPath.clear();
            getPathIndices(vertices, idxPath);
            idxPath.push_back(i);

            std::cout << "[";
            for (uint k = 0; k < idxPath.size(); k++)
            {
                std::cout << idxPath.at(k) << " ";
            }
            std::cout << "]" << std::endl;
            //############################################################################

            int i1 = vertices.at(0);
            base::State *s1 = graph[i1]->state;
            multilevel::PlannerDataVertexAnnotated *p1 = 
              new multilevel::PlannerDataVertexAnnotated(si->cloneState(s1));

            p1->setLevel(bundleGraph->getLevel());
            p1->setPath(idxPath);
            data.addStartVertex(*p1);

            // si->printState(s1);
            for (uint k = 0; k < vertices.size() - 1; k++)
            {
                base::State *s2 = graph[vertices.at(k+1)]->state;
                // si->printState(s2);

                multilevel::PlannerDataVertexAnnotated *p2 = 
                  new multilevel::PlannerDataVertexAnnotated(si->cloneState(s2));
                p2->setLevel(bundleGraph->getLevel());
                p2->setPath(idxPath);

                if (k == vertices.size() - 2)
                {
                    data.addGoalVertex(*p2);
                }
                else
                {
                    data.addVertex(*p2);
                }
                data.addEdge(*p1, *p2);

                p1 = p2;
            }
            std::cout << "Cost: " << getPathCost(i) << std::endl;
        }
    }
    foreach (const BundleSpaceGraph::Edge e, boost::edges(graph))
    {
        const BundleSpaceGraph::Vertex v1 = boost::source(e, graph);
        const BundleSpaceGraph::Vertex v2 = boost::target(e, graph);

        multilevel::PlannerDataVertexAnnotated p1(graph[v1]->state);
        multilevel::PlannerDataVertexAnnotated p2(graph[v2]->state);
        p1.setPath(idxPath);
        p2.setPath(idxPath);
        data.addEdge(p1, p2);
    }
    foreach (const BundleSpaceGraph::Vertex v, boost::vertices(graph))
    {
        multilevel::PlannerDataVertexAnnotated p(graph[v]->state);
        p.setPath(idxPath);
        data.addVertex(p);
    }
}
