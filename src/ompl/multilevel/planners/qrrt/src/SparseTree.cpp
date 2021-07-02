#include <ompl/multilevel/planners/qrrt/SparseTree.h>
#include <ompl/tools/config/SelfConfig.h>

using namespace ompl::multilevel;

unsigned int SparseTree::counter_ = 0;

SparseTree::SparseTree(TreeData& data, ompl::base::SpaceInformationPtr si):
  data_(data), si_(si)
{
    id_ = counter_++;
    double maxExt = si_->getMaximumExtent();
    initialRadius_ = 0.1 * maxExt; //0.02 for 6d ?
    maximumExtensions_ = 100;
}

void SparseTree::setup()
{
}

void SparseTree::clear()
{
    if(data_) data_->clear();
    while (!treeElementsPriorityQueue_.empty())
    {
      treeElementsPriorityQueue_.pop();
    }
    analytics_.clear();
    indexConfigToAnalytics_.clear();
}

BundleSpaceGraph::Configuration* 
SparseTree::nearest(Configuration *x)
{
    return data_->nearest(x);
}

bool SparseTree::isConverged()
{
    return data_->size() > 0 && treeElementsPriorityQueue_.size() <= 0;
}

ConfigurationAnalytics* SparseTree::getAnalytics(Configuration *x)
{
    return analytics_.at(indexConfigToAnalytics_.at(x->index));
}

void SparseTree::add(Configuration *x)
{
    data_->add(x);

    treeElementsPriorityQueue_.push(x);
    ConfigurationAnalytics *configAnalysis = new ConfigurationAnalytics();
    configAnalysis->config = x;
    configAnalysis->radius = initialRadius_;

    analytics_.push_back(configAnalysis);
    indexConfigToAnalytics_.insert({x->index, analytics_.size()-1});

    //TODO: Remove sanity check
    int idx = getAnalytics(x)->config->index;
    if(x->index != idx)
    {
      OMPL_ERROR("Wrong Indexing. config has index %d, but analytics vector returned index %d", x->index, idx);
      throw "WrongIndex";
    }
}

BundleSpaceGraph::Configuration* 
SparseTree::pop()
{
    Configuration *x = treeElementsPriorityQueue_.top();
    if(treeElementsPriorityQueue_.size() > 0)
    {
        treeElementsPriorityQueue_.pop();
    }
    return x;
}

std::size_t SparseTree::size()
{
    return data_->size();
}

std::size_t SparseTree::sizeConvergedNodes()
{
    return (size() - treeElementsPriorityQueue_.size());
}

double SparseTree::getRadius(Configuration *x)
{
    return getAnalytics(x)->radius;
}

void SparseTree::push(Configuration *x, ExtensionReturnType type)
{
    ConfigurationAnalytics *xstats = getAnalytics(x);
    xstats->numberOfExtensions++;
    // if(id_ == 0) xstats->print();
    if(type == EXTENSION_SUCCESS)
    {
        xstats->numberOfSuccessfulExtensions++;
        xstats->numberOfUnsuccessfulSubsequentExtensions = 0;
    }else if(type == EXTENSION_FAILURE_INVALID_STATE)
    {
        xstats->numberOfUnsuccessfulSubsequentExtensions++;
        xstats->extensionInvalidState++;
    }else if(type == EXTENSION_FAILURE_INSIDE_COVER)
    {
        xstats->numberOfUnsuccessfulSubsequentExtensions++;
        xstats->extensionInsideCover++;
    }else if(type == EXTENSION_FAILURE_SHELL_SAMPLING)
    {
        xstats->numberOfUnsuccessfulSubsequentExtensions+= 0.1*maximumExtensions_;
        xstats->extensionInsideCover++;
    }else if(type == EXTENSION_FAILURE_NO_CONNECTION)
    {
        //This is interesting. Try to gather more information by either
        //decreasing radius or by starting a sub-tree
        //
        //Simple principle: If we cannot connect, we are too coarse resolution.
        //In that case, we need to focus our attention on a smaller region
        //(finer resolution), because the state space geometry is apparently
        //much more intricate at that part of the state space. 
        //
        //Attention-focus or so? Adaptive sparsity
        //
        if(xstats->extensionNoConnection < 2)
        {
            xstats->radius *= 0.5;
        }
        // std::cout << "Reduced radius for " << x->index << " to " << xstats->radius << std::endl;
        xstats->numberOfUnsuccessfulSubsequentExtensions++;
        xstats->extensionNoConnection++;
    }else{
        OMPL_ERROR("ExtensionReturnType unknown.");
        throw "UnknownType";
    }

    if(xstats->numberOfUnsuccessfulSubsequentExtensions < maximumExtensions_)
    {
        treeElementsPriorityQueue_.push(x);
        if(treeElementsPriorityQueue_.size() > size())
        {
            OMPL_ERROR("Priority queue holds more elements than nodes in tree.");
            throw "QueueOverflow";
        }
    }else{
        // OMPL_INFORM("Removed index %d from active list.", x->index);
        // xstats->print();
    }
}
