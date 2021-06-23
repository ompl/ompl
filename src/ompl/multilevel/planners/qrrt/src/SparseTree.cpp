#include <ompl/multilevel/planners/qrrt/SparseTree.h>
#include <ompl/tools/config/SelfConfig.h>

using namespace ompl::multilevel;

SparseTree::SparseTree(TreeData& data):
  data_(data)
{
}

void SparseTree::setup()
{
}

void SparseTree::clear()
{
    if(data_) data_->clear();
}

BundleSpaceGraph::Configuration* 
SparseTree::nearest(Configuration *x)
{
    return data_->nearest(x);
}

bool SparseTree::isConverged()
{
    return false;
}

ConfigurationAnalytics* SparseTree::getAnalyticsFromConfiguration(Configuration *x)
{
    return analytics_.at(indexConfigToAnalytics_.at(x->index));
}

void SparseTree::add(Configuration *x)
{
    data_->add(x);
    treeElementsPriorityQueue_.push(x);
    ConfigurationAnalytics *configAnalysis = new ConfigurationAnalytics();
    configAnalysis->config = x;
    analytics_.push_back(configAnalysis);
    indexConfigToAnalytics_.insert({x->index, analytics_.size()-1});

    //TODO: Remove sanity check
    int idx = getAnalyticsFromConfiguration(x)->config->index;
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
    x->importance /= 2.0;
    treeElementsPriorityQueue_.pop();
    return x;
}

std::size_t SparseTree::size()
{
    return data_->size();
}

void SparseTree::push(Configuration *x, ExtensionReturnType type)
{
    if(type == EXTENSION_SUCCESS)
    {
    }else if(type==EXTENSION_FAILURE_INVALID_STATE)
    {
    }else if(type==EXTENSION_FAILURE_INSIDE_COVER)
    {
    }else{
      OMPL_ERROR("ExtensionReturnType unknown.");
      throw "UnknownType";
    }
    treeElementsPriorityQueue_.push(x);
}
