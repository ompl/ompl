#include <ompl/multilevel/planners/explorer/datastructures/LocalMinimaTree.h>
#include <ompl/util/Console.h>
#include <thread>
#include <ompl/geometric/PathSimplifier.h>

using namespace ompl::multilevel;

LocalMinimaTree::LocalMinimaTree(std::vector<base::SpaceInformationPtr> siVec) : siVec_(siVec)
{
    OMPL_DEBUG("Number of threads supported by implementation: %d", std::thread::hardware_concurrency());
    for (uint k = 0; k < siVec.size(); k++)
    {
        std::vector<LocalMinimaNode *> kthLevel;
        tree_.push_back(kthLevel);
    }
}

LocalMinimaTree::~LocalMinimaTree()
{
}

void LocalMinimaTree::clear()
{
    hasChanged_ = false;
    selectedMinimum_.clear();

    for (uint k = 0; k < tree_.size(); k++)
    {
        std::vector<LocalMinimaNode *> &treeLevelk = tree_.at(k);
        for (uint j = 0; j < treeLevelk.size(); j++)
        {
            LocalMinimaNode *node = treeLevelk.at(j);
            delete node;
            treeLevelk.at(j) = nullptr;
        }
        treeLevelk.clear();
    }

    numberOfMinima_ = 0;
    levels_ = 0;
}

unsigned int LocalMinimaTree::getNumberOfMinima(unsigned int level) const
{
    return tree_.at(level).size();
}

unsigned int LocalMinimaTree::getNumberOfMinima() const
{
    return numberOfMinima_;
}

unsigned int LocalMinimaTree::getNumberOfLevel() const
{
    return tree_.size();
}

std::vector<int> LocalMinimaTree::getSelectedPathIndex() const
{
    return selectedMinimum_;
}

void LocalMinimaTree::setSelectedPathIndex(std::vector<int> selectedMinimum)
{
    std::lock_guard<std::recursive_mutex> guard(lock_);
    selectedMinimum_ = selectedMinimum;
}

std::recursive_mutex &LocalMinimaTree::getLock()
{
    return lock_;
}

unsigned int LocalMinimaTree::getNumberOfLevelContainingMinima() const
{
    unsigned int ctr = 0;

    while (ctr <= (tree_.size() - 1) && tree_.at(ctr).size() > 0)
    {
        ctr++;
    }
    return ctr;
}

bool LocalMinimaTree::hasChanged()
{
    std::lock_guard<std::recursive_mutex> guard(lock_);
    if (hasChanged_)
    {
        hasChanged_ = false;
        return true;
    }
    else
    {
        return false;
    }
}
LocalMinimaNode *LocalMinimaTree::updatePath(base::PathPtr path, double cost, int level, int index)
{
    std::lock_guard<std::recursive_mutex> guard(lock_);
    sanityCheckLevelIndex(level, index);

    LocalMinimaNode *node = tree_.at(level).at(index);
    node->setPathPtr(path);
    node->setLevel(level);
    node->setCost(cost);
    hasChanged_ = true;

    if ((int)selectedMinimum_.size() == level - 1)
    {
        selectedMinimum_.back() = index;
    }
    return node;
}

LocalMinimaNode *LocalMinimaTree::addPath(base::PathPtr path, double cost, int level)
{
    std::lock_guard<std::recursive_mutex> guard(lock_);

    // base::OptimizationObjectivePtr obj =
    //   std::make_shared<ompl::base::PathLengthOptimizationObjective>(siVec_.at(level));
    // geometric::PathSimplifier shortcutter(siVec_.at(level), base::GoalPtr(), obj);

    geometric::PathGeometricPtr gpath = std::dynamic_pointer_cast<geometric::PathGeometric>(path);
    if (gpath != nullptr)
    {
        gpath->interpolate();
    }

    LocalMinimaNode *node = new LocalMinimaNode(siVec_.at(level), path);
    node->setLevel(level);
    node->setCost(cost);
    tree_.at(level).push_back(node);
    numberOfMinima_++;
    hasChanged_ = true;

    setSelectedMinimumExpand();
    selectedMinimum_.back() = tree_.at(level).size() - 1;
    return node;
}

LocalMinimaNode *LocalMinimaTree::getPath(int level, int index) const
{
    sanityCheckLevelIndex(level, index);
    return tree_.at(level).at(index);
}

LocalMinimaNode *LocalMinimaTree::getSelectedPath() const
{
    int level = selectedMinimum_.size() - 1;
    if (level < 0)
        return nullptr;

    int index = selectedMinimum_.back();
    return getPath(level, index);
}

std::vector<LocalMinimaNode *> LocalMinimaTree::getSelectedPathSiblings() const
{
    std::vector<LocalMinimaNode *> nodeVector;

    int level = selectedMinimum_.size() - 1;
    if (level < 0)
        return nodeVector;

    int indexSelectedPath = selectedMinimum_.back();

    // std::cout << level << " has " << getNumberOfMinima(level) << " paths." << std::endl;
    for (uint k = 0; k < getNumberOfMinima(level); k++)
    {
        if ((int)k == indexSelectedPath)
            continue;
        LocalMinimaNode *nodek = getPath(level, k);
        nodeVector.push_back(nodek);
    }
    return nodeVector;
}

void LocalMinimaTree::sanityCheckLevelIndex(int level, int index) const
{
    if (level < 0 || level > (int)getNumberOfLevel() - 1)
    {
        OMPL_ERROR("Tried getting path cost for non existing path.");
        throw ompl::Exception("Nonexisting");
    }
    if (index < 0 || index > (int)getNumberOfMinima(level) - 1)
    {
        OMPL_ERROR("Tried getting path cost for non existing path.");
        throw ompl::Exception("Nonexisting");
    }
}

double LocalMinimaTree::getPathCost(int level, int index) const
{
    sanityCheckLevelIndex(level, index);
    LocalMinimaNode *node = tree_.at(level).at(index);
    return node->getCost();
}

void LocalMinimaTree::setSelectedMinimumPrev()
{
    std::lock_guard<std::recursive_mutex> guard(lock_);

    if (selectedMinimum_.size() > 0)
    {
        int level = selectedMinimum_.size() - 1;
        int maxMinima = getNumberOfMinima(level);
        if (maxMinima > 0)
        {
            if (selectedMinimum_.back() > 0)
            {
                selectedMinimum_.back()--;
            }
            else
            {
                selectedMinimum_.back() = maxMinima - 1;
            }

            OMPL_DEVMSG1("Selected local minimum %d/%d (level %d, cost %.2f)", selectedMinimum_.back() + 1, maxMinima,
                         level, tree_.at(level).at(selectedMinimum_.back())->getCost());
        }
    }
    hasChanged_ = true;
}

void LocalMinimaTree::setSelectedMinimumNext()
{
    std::lock_guard<std::recursive_mutex> guard(lock_);

    if (selectedMinimum_.size() > 0)
    {
        int maxMinima = getNumberOfMinima(selectedMinimum_.size() - 1);
        if (maxMinima > 0)
        {
            if (selectedMinimum_.back() < maxMinima - 1)
                selectedMinimum_.back()++;
            else
                selectedMinimum_.back() = 0;
            OMPL_DEVMSG1("Selected local minimum %d/%d (level %d)", selectedMinimum_.back() + 1, maxMinima,
                         selectedMinimum_.size() - 1);
        }
    }
    hasChanged_ = true;
}

void LocalMinimaTree::setSelectedMinimumCollapse()
{
    std::lock_guard<std::recursive_mutex> guard(lock_);

    if (selectedMinimum_.size() > 0)
    {
        selectedMinimum_.erase(selectedMinimum_.end() - 1);
    }
    hasChanged_ = true;
}

void LocalMinimaTree::setSelectedMinimumExpand()
{
    std::lock_guard<std::recursive_mutex> guard(lock_);

    unsigned int maxLevel = getNumberOfLevelContainingMinima();
    if (selectedMinimum_.size() < maxLevel)
    {
        selectedMinimum_.push_back(0);
    }
    hasChanged_ = true;
}

void LocalMinimaTree::setSelectedMinimumExpandFull()
{
    std::lock_guard<std::recursive_mutex> guard(lock_);

    unsigned int maxLevel = getNumberOfLevelContainingMinima();
    while (selectedMinimum_.size() < maxLevel)
    {
        selectedMinimum_.push_back(0);
    }
    hasChanged_ = true;
}

void LocalMinimaTree::printSelectedMinimum()
{
    for (uint k = 0; k < selectedMinimum_.size(); k++)
    {
        std::cout << selectedMinimum_.at(k);
        std::cout << (k < selectedMinimum_.size() - 1 ? " > " : "");
    }
    std::cout << std::endl;
}

const std::vector<ompl::base::State *> &LocalMinimaTree::getSelectedMinimumAsStateVector(int level) const
{
    int selectedMinimumOnLevel = selectedMinimum_.at(level);
    LocalMinimaNode *node = tree_.at(level).at(selectedMinimumOnLevel);
    return node->asStates();
}
const ompl::base::PathPtr &LocalMinimaNode::asPathPtr() const
{
    if (hasPathPtrRepresentation)
        return path_;

    OMPL_ERROR("NYI");
    throw ompl::Exception("NYI");
}

const StatesPath &LocalMinimaNode::asStates() const
{
    if (hasStatesRepresentation)
        return spath_;

    OMPL_ERROR("NYI");
    throw ompl::Exception("NYI");
}
const VertexPath &LocalMinimaNode::asVertices() const
{
    if (hasVertexRepresentation)
        return vpath_;
    OMPL_ERROR("NYI");
    throw ompl::Exception("NYI");
}
LocalMinimaNode::LocalMinimaNode(base::SpaceInformationPtr si, StatesPath &states)
{
    si_ = si;
    spath_ = states;
    hasStatesRepresentation = true;
}
LocalMinimaNode::LocalMinimaNode(base::SpaceInformationPtr si, VertexPath &vertices)
{
    si_ = si;
    vpath_ = vertices;
    hasVertexRepresentation = true;
}
LocalMinimaNode::LocalMinimaNode(base::SpaceInformationPtr si, base::PathPtr path)
{
    si_ = si;
    path_ = path;
    hasPathPtrRepresentation = true;
}
