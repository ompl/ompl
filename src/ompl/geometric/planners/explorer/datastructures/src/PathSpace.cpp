#include <ompl/geometric/planners/explorer/datastructures/PathSpace.h>
#include <ompl/geometric/planners/multilevel/datastructures/PlannerDataVertexAnnotated.h>
#include <boost/foreach.hpp>

#define foreach BOOST_FOREACH

using namespace ompl::geometric;

ompl::geometric::PathSpace::PathSpace()
{
}
ompl::geometric::PathSpace::~PathSpace()
{
}

int ompl::geometric::PathSpace::getSelectedPath()
{
    return selectedPath_;
}
void ompl::geometric::PathSpace::setSelectedPath(int selectedPath)
{
    selectedPath_ = selectedPath;
}

void ompl::geometric::PathSpace::updatePath(int k, VertexPath p, double length)
{
    if(k >= criticalPaths_.size()) return;
    criticalPaths_.at(k) = p;
    criticalPathsLength_.at(k) = length;
}

void ompl::geometric::PathSpace::addPath(VertexPath p, double length)
{
    criticalPaths_.push_back(p);
    criticalPathsLength_.push_back(length);
}

double ompl::geometric::PathSpace::getPathLength(int k) const
{
    if(k < criticalPathsLength_.size())
    {
        return criticalPathsLength_.at(k);
    }
}
std::vector<BundleSpaceGraph::Vertex>& ompl::geometric::PathSpace::getCriticalPath(int k)
{
    if(k < criticalPaths_.size())
    {
        return criticalPaths_.at(k);
    }
}

unsigned int ompl::geometric::PathSpace::getNumberOfPaths() const
{
    return criticalPaths_.size();
}

void ompl::geometric::PathSpace::getPlannerData(base::PlannerData &data, BundleSpaceGraph* bundleGraph) const
{

    BundleSpaceGraph::Graph graph = bundleGraph->getGraph();
    base::SpaceInformationPtr si = bundleGraph->getBundle();


    std::vector<int> idxPathI;
    if (criticalPaths_.size() > 0)
    {
        OMPL_DEVMSG1("%s has %d solutions.", bundleGraph->getName().c_str(), criticalPaths_.size());

        for (uint i = 0; i < criticalPaths_.size(); i++)
        {
            const std::vector<BundleSpaceGraph::Vertex> vertices = criticalPaths_.at(i);

            idxPathI.clear();
            idxPathI.push_back(i);

            //############################################################################
            // DEBUG
            std::cout << "[";
            for (uint k = 0; k < idxPathI.size(); k++)
            {
                std::cout << idxPathI.at(k) << " ";
            }
            std::cout << "]" << std::endl;
            //############################################################################

            int i1 = vertices.at(0);
            base::State *s1 = graph[i1]->state;
            base::PlannerDataVertexAnnotated *p1 = new base::PlannerDataVertexAnnotated(
                si->cloneState(s1));
            p1->setLevel(bundleGraph->getLevel());
            p1->setPath(idxPathI);
            data.addStartVertex(*p1);

            si->printState(s1);
            for (uint k = 0; k < vertices.size() - 1; k++)
            {
                base::State *s2 = graph[vertices.at(k+1)]->state;
                si->printState(s2);

                base::PlannerDataVertexAnnotated *p2 = new base::PlannerDataVertexAnnotated(
                  si->cloneState(s2));
                p2->setLevel(bundleGraph->getLevel());
                p2->setPath(idxPathI);

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
            std::cout << "Length: " << getPathLength(i) << std::endl;
        }
    }
    foreach (const BundleSpaceGraph::Edge e, boost::edges(graph))
    {
        const BundleSpaceGraph::Vertex v1 = boost::source(e, graph);
        const BundleSpaceGraph::Vertex v2 = boost::target(e, graph);

        base::PlannerDataVertexAnnotated p1(graph[v1]->state);
        base::PlannerDataVertexAnnotated p2(graph[v2]->state);
        p1.setPath(idxPathI);
        p2.setPath(idxPathI);
        data.addEdge(p1, p2);
    }
    foreach (const BundleSpaceGraph::Vertex v, boost::vertices(graph))
    {
        base::PlannerDataVertexAnnotated p(graph[v]->state);
        p.setPath(idxPathI);
        data.addVertex(p);
    }
}
