#include <ompl/multilevel/datastructures/PlannerMultiLevel.h>

ompl::multilevel::PlannerMultiLevel::PlannerMultiLevel(
    std::vector<ompl::base::SpaceInformationPtr> &siVec, std::string type)
  : BaseT(siVec.back(), type), siVec_(siVec)
{
}

ompl::multilevel::PlannerMultiLevel::~PlannerMultiLevel()
{
}

void ompl::multilevel::PlannerMultiLevel::clear()
{
    BaseT::clear();
    solutions_.clear();
    pdef_->clearSolutionPaths();
    for(uint k = 0; k < pdefVec_.size(); k++)
    {
        pdefVec_.at(k)->clearSolutionPaths();
    }
}


std::vector<int> ompl::multilevel::PlannerMultiLevel::getDimensionsPerLevel() const
{
    std::vector<int> dimensionsPerLevel;
    for (unsigned int k = 0; k < siVec_.size(); k++)
    {
        unsigned int Nk = siVec_.at(k)->getStateDimension();
        dimensionsPerLevel.push_back(Nk);
    }
    return dimensionsPerLevel;
}

int ompl::multilevel::PlannerMultiLevel::getLevels() const
{
    return siVec_.size();
}

const ompl::base::ProblemDefinitionPtr &
ompl::multilevel::PlannerMultiLevel::getProblemDefinition(int level) const
{
    return pdefVec_.at(level);
}

const std::vector<ompl::base::ProblemDefinitionPtr>&
ompl::multilevel::PlannerMultiLevel::getProblemDefinitionVector() const
{
    return pdefVec_;
}
