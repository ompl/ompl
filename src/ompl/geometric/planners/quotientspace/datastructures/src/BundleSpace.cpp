#include <ompl/geometric/planners/quotientspace/datastructures/BundleSpace.h>

#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/SO3StateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/TimeStateSpace.h>
#include <ompl/base/spaces/DiscreteStateSpace.h>
#include <ompl/tools/config/MagicConstants.h>

#include <ompl/util/Exception.h>

unsigned int ompl::geometric::BundleSpace::counter_ = 0;

ompl::geometric::BundleSpace::BundleSpace(const base::SpaceInformationPtr &si, BundleSpace *parent_)
  : base::Planner(si, "BundleSpace"), Bundle(si), parent_(parent_)
{
    id_ = counter_++;

    //############################################################################
    //Check for dynamic spaces
    //############################################################################
    ompl::control::SpaceInformation *siC = dynamic_cast<ompl::control::SpaceInformation*>(getBundle().get());
    if(siC==nullptr) {
      isDynamic_ = false;
    }else{
      isDynamic_ = true;
    }
    OMPL_DEBUG("BundleSpace %d%s", id_, (isDynamic_?" (dynamic)":""));
    //############################################################################

    if (!hasBaseSpace())
    {
        components_ = componentFactory_.MakeBundleSpaceComponents(Bundle);
    }
    else
    {
        parent_->setChild(this);
        Base = parent_->getBundle();

        components_ = componentFactory_.MakeBundleSpaceComponents(Bundle, Base);

        MakeFiberSpace();
    }

    sanityChecks();

    std::cout << std::string(80, '-') << std::endl;
    std::cout << *this << std::endl;
    std::cout << std::string(80, '-') << std::endl;

    if (!Bundle_valid_sampler_)
    {
        Bundle_valid_sampler_ = Bundle->allocValidStateSampler();
    }
    if (!Bundle_sampler_)
    {
        Bundle_sampler_ = Bundle->allocStateSampler();
    }
    if (hasParent())
    {
        xBaseTmp_ = Base->allocState();
        if (getFiberDimension() > 0)
            xFiberTmp_ = Fiber->allocState();
    }
}

ompl::geometric::BundleSpace::~BundleSpace()
{
    if (hasParent())
    {
        if (xBaseTmp_)
            Base->freeState(xBaseTmp_);
        if (Fiber && xFiberTmp_)
            Fiber->freeState(xFiberTmp_);
    }
    components_.clear();
}

bool ompl::geometric::BundleSpace::hasParent() const
{
    return !(parent_ == nullptr);
}
bool ompl::geometric::BundleSpace::hasBaseSpace() const
{
    return hasParent();
}

bool ompl::geometric::BundleSpace::hasChild() const
{
    return !(child_ == nullptr);
}

bool ompl::geometric::BundleSpace::isDynamic() const
{
    return isDynamic_;
}

void ompl::geometric::BundleSpace::setup()
{
    BaseT::setup();
    hasSolution_ = false;
    firstRun_ = true;
    if(pdef_) goal_ = pdef_->getGoal().get();
}

void ompl::geometric::BundleSpace::clear()
{
    BaseT::clear();

    hasSolution_ = false;
    firstRun_ = true;
    if (!hasParent() && getFiberDimension() > 0)
        Fiber_sampler_.reset();

    pdef_->clearSolutionPaths();
}

void ompl::geometric::BundleSpace::MakeFiberSpace()
{
    base::StateSpacePtr Fiber_space = nullptr;
    if(components_.size() > 1)
    {
        Fiber_space = std::make_shared<base::CompoundStateSpace>();
        for(uint m = 0; m < components_.size(); m++){
            base::StateSpacePtr FiberM = components_.at(m)->getFiberSpace();
            double weight = (FiberM->getDimension() > 0 ? 1.0 : 0.0);
            std::static_pointer_cast<base::CompoundStateSpace>(Fiber_space)->addSubspace(FiberM, weight);
        }
    }else{
        Fiber_space = components_.front()->getFiberSpace();
    }

    if (Fiber_space != nullptr)
    {
        Fiber = std::make_shared<base::SpaceInformation>(Fiber_space);
        Fiber_sampler_ = Fiber->allocStateSampler();
    }
}

void ompl::geometric::BundleSpace::sanityChecks() const
{

  const base::StateSpacePtr Bundle_space = Bundle->getStateSpace();
  checkBundleSpaceMeasure("Bundle", Bundle_space);

  if(Base != nullptr)
  {
      const base::StateSpacePtr Base_space = Base->getStateSpace();
      checkBundleSpaceMeasure("Base", Base_space);
  }
  if(Fiber != nullptr)
  {
      const base::StateSpacePtr Fiber_space = Fiber->getStateSpace();
      checkBundleSpaceMeasure("Fiber", Fiber_space);
      if ((getBaseDimension() + getFiberDimension() != getBundleDimension()))
      {
          OMPL_ERROR("Dimensions %d (Base) + %d (Fiber) != %d (Bundle)", 
              getBaseDimension(), getFiberDimension(), getBundleDimension());
          throw ompl::Exception("BundleSpace Dimensions are wrong.");
      }
  }

}

void ompl::geometric::BundleSpace::checkBundleSpaceMeasure(std::string name, const base::StateSpacePtr space) const
{
    OMPL_DEVMSG1("%s dimension: %d measure: %f", name, space->getDimension(), space->getMeasure());
    if ((space->getMeasure() >= std::numeric_limits<double>::infinity()) ||
        (space->getMeasure() <= 0))
    {
        throw ompl::Exception("Space has zero or infinite measure.");
    }
}

ompl::base::PlannerStatus ompl::geometric::BundleSpace::solve(const base::PlannerTerminationCondition&)
{
    throw ompl::Exception("A Bundle-Space cannot be solved alone. Use class MultiBundle to solve Bundle-Spaces.");
}

void ompl::geometric::BundleSpace::setProblemDefinition(const base::ProblemDefinitionPtr &pdef)
{
    BaseT::setProblemDefinition(pdef);

    if (pdef_->hasOptimizationObjective())
    {
        opt_ = pdef_->getOptimizationObjective();
    }
    else
    {
        opt_ = std::make_shared<base::PathLengthOptimizationObjective>(getBundle());
    }
}

void ompl::geometric::BundleSpace::resetCounter()
{
    BundleSpace::counter_ = 0;
}

void ompl::geometric::BundleSpace::liftPath(
      const std::vector<base::State*> pathBase,
      const base::State* xFiberStart,
      const base::State* xFiberGoal,
      std::vector<base::State*> pathBundle) const
{
    if(pathBase.size() != pathBundle.size())
    {
      OMPL_ERROR("Size of paths has to be identical in order to lift them.");
      throw Exception("Path size inequivalent.");
    }
    if(getFiberDimension() > 0)
    {
        double lengthTotalPathBase = 0;
        std::vector<double> lengthIntermediatePathBase;

        for(uint k = 1; k < pathBase.size(); k++){
            double lengthKthSegment = getBase()->distance(pathBase.at(k-1), pathBase.at(k));
            lengthIntermediatePathBase.push_back(lengthKthSegment); 
            lengthTotalPathBase += lengthKthSegment;
        }

        double lengthCurrent = 0;

        base::State *xFiberCur = getFiber()->allocState();

        for(uint k = 0; k < pathBase.size(); k++)
        {
            double step = lengthCurrent / lengthTotalPathBase;
            getFiber()->getStateSpace()->interpolate(xFiberStart, xFiberGoal, step, xFiberCur);

            liftState(pathBase.at(k), xFiberCur, pathBundle.at(k));

            if(k < pathBase.size() - 1)
            {
                lengthCurrent += lengthIntermediatePathBase.at(k);
            }
        }
        getFiber()->freeState(xFiberCur);

    }else{
        for(uint k = 0; k < pathBase.size(); k++){
            getBundle()->copyState(pathBundle.at(k), pathBase.at(k));
        }
    }
}

void ompl::geometric::BundleSpace::liftState(const base::State *xBase, const base::State *xFiber, base::State *xBundle) const
{
    unsigned int M = components_.size();

    if( M > 1)
    {
        for(uint m = 0; m < M; m++)
        {
            const base::State *xmBase = xBase->as<base::CompoundState>()->as<base::State>(m);
            const base::State *xmFiber = xFiber->as<base::CompoundState>()->as<base::State>(m);
            base::State *xmBundle = xBundle->as<base::CompoundState>()->as<base::State>(m);
            components_.at(m)->liftState(xmBase, xmFiber, xmBundle);
        }
    }else{
        components_.front()->liftState(xBase, xFiber, xBundle);
    }
}

void ompl::geometric::BundleSpace::projectFiber(const base::State *xBundle, base::State *xFiber) const
{
    unsigned int M = components_.size();

    if( M > 1){
        for(uint m = 0; m < M; m++)
        {
            if(components_.at(m)->getFiberDimension() > 0)
            {
                const base::State *xmBundle = xBundle->as<base::CompoundState>()->as<base::State>(m);
                base::State *xmFiber = xFiber->as<base::CompoundState>()->as<base::State>(m);
                components_.at(m)->projectFiber(xmBundle, xmFiber);
            }
        }
    }else{
        components_.front()->projectFiber(xBundle, xFiber);
    }
}

void ompl::geometric::BundleSpace::projectBase(const base::State *xBundle, base::State *xBase) const
{
    unsigned int M = components_.size();

    if( M > 1){
        for(uint m = 0; m < M; m++)
        {
            if(components_.at(m)->getBaseDimension() > 0)
            {
                const base::State *xmBundle = xBundle->as<base::CompoundState>()->as<base::State>(m);
                base::State *xmBase = xBase->as<base::CompoundState>()->as<base::State>(m);
                components_.at(m)->projectBase(xmBundle, xmBase);
            }
        }
    }else{
        components_.front()->projectBase(xBundle, xBase);
    }
}

void ompl::geometric::BundleSpace::allocIdentityState(base::State *s, base::StateSpacePtr space) const
{
  if(space->isCompound()){
    base::CompoundStateSpace *cspace = space->as<base::CompoundStateSpace>();
    const std::vector<base::StateSpacePtr> compounds = cspace->getSubspaces();
    for(unsigned int k = 0; k < compounds.size(); k++){
      base::StateSpacePtr spacek = compounds.at(k);
      base::State *xk = s->as<base::CompoundState>()->as<base::State>(k);
      allocIdentityState(xk, spacek);
    }
  }else{
    int stype = space->getType();
    switch (stype) 
    {
      case base::STATE_SPACE_SO3:
      {
        static_cast<base::SO3StateSpace::StateType *>(s)->setIdentity();
        break;
      }
      case base::STATE_SPACE_SO2:
      {
        static_cast<base::SO2StateSpace::StateType *>(s)->setIdentity();
        break;
      }
      case base::STATE_SPACE_TIME:
      {
        static_cast<base::TimeStateSpace::StateType *>(s)->position = 0;
        break;
      }
      case base::STATE_SPACE_DISCRETE:
      {
        base::DiscreteStateSpace *space_Discrete = space->as<base::DiscreteStateSpace>();
        int lb = space_Discrete->getLowerBound();
        static_cast<base::DiscreteStateSpace::StateType *>(s)->value = lb;
        break;
      }
      case base::STATE_SPACE_REAL_VECTOR:
      {
        base::RealVectorStateSpace::StateType *sRN = s->as<base::RealVectorStateSpace::StateType>();
        for(uint k = 0; k < space->getDimension(); k++){
          sRN->values[k] = 0;
        }
        break;
      }
      default:
      {
        OMPL_ERROR("Type: %d not recognized.", stype);
        throw Exception("Type not recognized.");
      }
    }
  }
}

ompl::base::State* ompl::geometric::BundleSpace::allocIdentityState(base::StateSpacePtr space) const
{
  if(space != nullptr){
    base::State *s = space->allocState();
    allocIdentityState(s, space);
    return s;
  }else{
    return nullptr;
  }
}

ompl::base::State* ompl::geometric::BundleSpace::allocIdentityStateFiber() const
{
  return allocIdentityState(getFiber()->getStateSpace());
}

ompl::base::State* ompl::geometric::BundleSpace::allocIdentityStateBundle() const
{
  return allocIdentityState(getBundle()->getStateSpace());
}

ompl::base::State* ompl::geometric::BundleSpace::allocIdentityStateBase() const
{
  return allocIdentityState(getBase()->getStateSpace());
}

const ompl::base::SpaceInformationPtr &ompl::geometric::BundleSpace::getFiber() const
{
    return Fiber;
}

const ompl::base::SpaceInformationPtr &ompl::geometric::BundleSpace::getBundle() const
{
    return Bundle;
}

const ompl::base::SpaceInformationPtr &ompl::geometric::BundleSpace::getBase() const
{
    return Base;
}

unsigned int ompl::geometric::BundleSpace::getFiberDimension() const
{
    if(getFiber()) return getFiber()->getStateDimension();
    else return 0;
}

unsigned int ompl::geometric::BundleSpace::getBaseDimension() const
{
    if(getBase()) return getBase()->getStateDimension();
    else return 0;
}

unsigned int ompl::geometric::BundleSpace::getBundleDimension() const
{
    return getBundle()->getStateDimension();
}

const ompl::base::StateSamplerPtr &ompl::geometric::BundleSpace::getFiberSamplerPtr() const
{
    return Fiber_sampler_;
}

const ompl::base::StateSamplerPtr &ompl::geometric::BundleSpace::getBundleSamplerPtr() const
{
    return Bundle_sampler_;
}

bool ompl::geometric::BundleSpace::isInfeasible()
{
    return false;
}

bool ompl::geometric::BundleSpace::hasSolution()
{
    if (!hasSolution_)
    {
        base::PathPtr path;
        hasSolution_ = getSolution(path);
    }
    return hasSolution_;
}

ompl::geometric::BundleSpace *ompl::geometric::BundleSpace::getParent() const
{
    return parent_;
}

ompl::geometric::BundleSpace *ompl::geometric::BundleSpace::getChild() const
{
    return child_;
}

void ompl::geometric::BundleSpace::setChild(ompl::geometric::BundleSpace *child)
{
    child_ = child;
}

void ompl::geometric::BundleSpace::setParent(ompl::geometric::BundleSpace *parent)
{
    parent_ = parent;
}

unsigned int ompl::geometric::BundleSpace::getLevel() const
{
    return level_;
}

void ompl::geometric::BundleSpace::setLevel(unsigned int level)
{
    level_ = level;
}

ompl::base::OptimizationObjectivePtr ompl::geometric::BundleSpace::getOptimizationObjectivePtr() const
{
    return opt_;
}

void ompl::geometric::BundleSpace::sampleFiber(base::State *xFiber)
{
    Fiber_sampler_->sampleUniform(xFiber);
}

bool ompl::geometric::BundleSpace::sampleBundleValid(base::State *xRandom)
{
		bool found = false;

    unsigned int attempts = 0;
    do
    {
        sampleBundle(xRandom);
        found = getBundle()->getStateValidityChecker()->isValid(xRandom);
        attempts++;
    } while (attempts < magic::FIND_VALID_STATE_ATTEMPTS_WITHOUT_TERMINATION_CHECK && !found);
    return found;
}

void ompl::geometric::BundleSpace::sampleBundle(base::State *xRandom)
{
    if (!hasParent())
    {
        Bundle_sampler_->sampleUniform(xRandom);
    }
    else
    {
        if (getFiberDimension() > 0)
        {
            // Adjusted sampling function: Sampling in G0 x Fiber
            parent_->sampleFromDatastructure(xBaseTmp_);
            sampleFiber(xFiberTmp_);
            liftState(xBaseTmp_, xFiberTmp_, xRandom);
        }
        else
        {
            parent_->sampleFromDatastructure(xRandom);
        }
    }
}

std::vector<int> ompl::geometric::BundleSpace::getIndexLevel() const
{
    std::vector<int> idxPathI;
    BundleSpace *pparent = getParent();
    while (pparent != nullptr)
    {
        idxPathI.push_back(0);
        pparent = pparent->getParent();
    }
    idxPathI.push_back(0);
    return idxPathI;
}

void ompl::geometric::BundleSpace::debugInvalidState(const base::State *x)
{
    const base::StateSpacePtr space = Bundle->getStateSpace();
    bool bounds = space->satisfiesBounds(x);
    if(!bounds)
    {
        std::vector<base::StateSpacePtr> Bundle_decomposed;
        if (!space->isCompound())
        {
            Bundle_decomposed.push_back(space);
        }else{
            base::CompoundStateSpace *Bundle_compound = space->as<base::CompoundStateSpace>();
            Bundle_decomposed = Bundle_compound->getSubspaces();
        }

        for(unsigned int m = 0; m < Bundle_decomposed.size(); m++)
        {
            base::StateSpacePtr spacek = Bundle_decomposed.at(m);
            int type = spacek->getType();
            switch (type) 
            {
                case base::STATE_SPACE_REAL_VECTOR:
                {
                    auto *RN = spacek->as<base::RealVectorStateSpace>();
                    const base::RealVectorStateSpace::StateType *xk = 
                      x->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(m);
                    std::vector<double> bl =  RN->getBounds().low;
                    std::vector<double> bh =  RN->getBounds().high;
                    for(unsigned int k = 0; k < bl.size(); k++)
                    {
                      double qk = xk->values[k];
                      double qkl = bl.at(k);
                      double qkh = bh.at(k);
                      if(qk < qkl || qk > qkh){
                          std::cout << "Out Of Bounds [" 
                            << "component " << m << ", "
                            << "link " << k 
                            << "] " 
                            << bl.at(k) << " <= " << qk << " <= " << bh.at(k) << std::endl;
                      }
                    }
                    break;
                }
                case base::STATE_SPACE_SO2:
                {
                    double value = 0;
                    if (!space->isCompound())
                    {
                        const base::SO2StateSpace::StateType *xk = 
                          x->as<base::CompoundState>()->as<base::SO2StateSpace::StateType>(m);
                        value = xk->value;
                    }else{
                        const base::SO2StateSpace::StateType *xk = 
                          x->as<base::SO2StateSpace::StateType>();
                        value = xk->value;
                    }
                    std::cout << "Invalid: -pi <= " << value << " <= +pi" << std::endl;
                    break;
                }
                default:
                {

                    OMPL_ERROR("Could not debug state type %d.", type);
                    break;
                }
            }
        }
    }else{
        std::cout << "Bounds satisfied. Must be collision problem." << std::endl;
    }
}

void ompl::geometric::BundleSpace::print(std::ostream &out) const
{
    unsigned int M = components_.size();
    out << "[";
    for(unsigned int m = 0; m < M; m++){
        out << components_.at(m)->getTypeAsString() 
            // << (components_.at(m)->isDynamic()?" (dyn)":"");
            << (isDynamic_?"(dyn)":"")
            << (m<M-1?" | ":"");
    }
    out << "]";
}

namespace ompl
{
    namespace geometric
    {
        std::ostream &operator<<(std::ostream &out, const BundleSpace &bundleSpace)
        {
            bundleSpace.print(out);
            return out;
        }
    } 
}
