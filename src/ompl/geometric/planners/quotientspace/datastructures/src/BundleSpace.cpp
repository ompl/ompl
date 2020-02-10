#include <ompl/geometric/planners/quotientspace/datastructures/BundleSpace.h>

#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/SO3StateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>

#include <ompl/util/Exception.h>

unsigned int ompl::geometric::BundleSpace::counter_ = 0;

ompl::geometric::BundleSpace::BundleSpace(const base::SpaceInformationPtr &si, BundleSpace *parent_)
  : base::Planner(si, "BundleSpace"), Bundle(si), parent_(parent_)
{
    id_ = counter_++;

    //############################################################################
    //Check for dynamic spaces
    //############################################################################
    ompl::control::SpaceInformation *siC = dynamic_cast<ompl::control::SpaceInformation*>(si_.get());
    if(siC==nullptr) {
      isDynamic_ = false;
    }else{
      isDynamic_ = true;
    }
    OMPL_DEVMSG1("BundleSpace %d%s", id_, (isDynamic_?" (dynamic)":""));
    //############################################################################

    const base::StateSpacePtr Bundle_space = Bundle->getStateSpace();
    int bundleSpaceComponents = GetNumberOfComponents(Bundle_space);

    if (!hasParent())
    {
        if(bundleSpaceComponents > 1){
          base::CompoundStateSpace *Bundle_compound = 
            Bundle_space->as<base::CompoundStateSpace>();
          const std::vector<base::StateSpacePtr> Bundle_decomposed = Bundle_compound->getSubspaces();

          for(int m = 0; m < bundleSpaceComponents; m++){
            base::StateSpacePtr BundleM = Bundle_decomposed.at(m);
            BundleSpaceComponentPtr componentM = 
              componentFactory.MakeBundleSpaceComponent(BundleM, nullptr);
            components_.push_back(componentM);
          }
        }else{
            BundleSpaceComponentPtr component = componentFactory.MakeBundleSpaceComponent(Bundle_space, nullptr);
            components_.push_back(component);
        }
    }
    else
    {
        parent_->setChild(this);

        Base = parent_->getBundle();
        const base::StateSpacePtr Base_space = Base->getStateSpace();

        base::StateSpacePtr Fiber_space = nullptr;
        if(bundleSpaceComponents > 1){

          base::CompoundStateSpace *Bundle_compound = 
            Bundle_space->as<base::CompoundStateSpace>();
          base::CompoundStateSpace *Base_compound = 
            Base_space->as<base::CompoundStateSpace>();

          int baseSpaceComponents = GetNumberOfComponents(Base_space);
          if(baseSpaceComponents != bundleSpaceComponents)
          {
            OMPL_ERROR("Base Space has %d, but Bundle Space has %d components.", 
                baseSpaceComponents, bundleSpaceComponents);
            throw Exception("Different Number Of Components");
          }

          const std::vector<base::StateSpacePtr> Bundle_decomposed = Bundle_compound->getSubspaces();
          const std::vector<base::StateSpacePtr> Base_decomposed = Base_compound->getSubspaces();

          Fiber_space = std::make_shared<base::CompoundStateSpace>();
          for(int m = 0; m < bundleSpaceComponents; m++){
            base::StateSpacePtr BaseM = Base_decomposed.at(m);
            base::StateSpacePtr BundleM = Bundle_decomposed.at(m);
            BundleSpaceComponentPtr componentM = 
              componentFactory.MakeBundleSpaceComponent(BundleM, BaseM);
            components_.push_back(componentM);
            
            base::StateSpacePtr FiberM = componentM->getFiberSpace();
            double weight = (FiberM->getDimension() > 0 ? 1.0 : 0.0);
            std::static_pointer_cast<base::CompoundStateSpace>(Fiber_space)->addSubspace(FiberM, weight);
          }

        }else{
          BundleSpaceComponentPtr component = 
            componentFactory.MakeBundleSpaceComponent(Bundle_space, Base_space);
          components_.push_back(component);
          Fiber_space = component->getFiberSpace();
        }

        if (Fiber_space != nullptr)
        {
            Fiber = std::make_shared<base::SpaceInformation>(Fiber_space);
            Fiber_sampler_ = Fiber->allocStateSampler();

            if (Base_space->getDimension() + Fiber_space->getDimension() != Bundle_space->getDimension())
            {
                throw ompl::Exception("BundleSpace Dimensions are wrong.");
            }
            OMPL_DEVMSG1("Base dimension: %d measure: %f", Base_space->getDimension(), Base_space->getMeasure());
            OMPL_DEVMSG1("Fiber dimension: %d measure: %f", Fiber_space->getDimension(), Fiber_space->getMeasure());
            OMPL_DEVMSG1("Bundle dimension: %d measure: %f", Bundle_space->getDimension(), Bundle_space->getMeasure());
            if ((Base_space->getMeasure() <= 0) || (Fiber_space->getMeasure() <= 0) || (Bundle_space->getMeasure() <= 0))
            {
                throw ompl::Exception("Zero-measure BundleSpace detected.");
            }
            checkSpaceHasFiniteMeasure(Fiber_space);
        }
        else
        {
            OMPL_DEVMSG1("Base dimension: %d measure: %f", Base_space->getDimension(), Base_space->getMeasure());
            OMPL_DEVMSG1("Bundle dimension: %d measure: %f", Bundle_space->getDimension(), Bundle_space->getMeasure());
        }
        checkSpaceHasFiniteMeasure(Base_space);
    }

    std::cout << *this << std::endl;
    checkSpaceHasFiniteMeasure(Bundle_space);

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

int ompl::geometric::BundleSpace::GetNumberOfComponents(base::StateSpacePtr space)
{
  int nrComponents = 0;

  if(space->isCompound()){
    base::CompoundStateSpace *compound = space->as<base::CompoundStateSpace>();
    nrComponents = compound->getSubspaceCount();
    if(nrComponents == 2)
    {
      int type = space->getType();

      if((type == base::STATE_SPACE_SE2) || (type == base::STATE_SPACE_SE3))
      {
        nrComponents = 1;
      }else{
        const std::vector<base::StateSpacePtr> decomposed = compound->getSubspaces();
        int t0 = decomposed.at(0)->getType();
        int t1 = decomposed.at(1)->getType();
        if(
            (t0 == base::STATE_SPACE_SO2 && t1 == base::STATE_SPACE_REAL_VECTOR) ||
            (t0 == base::STATE_SPACE_SO3 && t1 == base::STATE_SPACE_REAL_VECTOR) ||
            (t0 == base::STATE_SPACE_SE2 && t1 == base::STATE_SPACE_REAL_VECTOR) ||
            (t0 == base::STATE_SPACE_SE3 && t1 == base::STATE_SPACE_REAL_VECTOR) 
          )
        {
          nrComponents = 1;
        }
      }
    }
  }else{
    nrComponents = 1;
  }
  return nrComponents;
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
}

bool ompl::geometric::BundleSpace::hasParent() const
{
    return !(parent_ == nullptr);
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

void ompl::geometric::BundleSpace::checkSpaceHasFiniteMeasure(const base::StateSpacePtr space) const
{
    if (space->getMeasure() >= std::numeric_limits<double>::infinity())
    {
        const base::StateSpacePtr Base_space = Base->getStateSpace();
        const base::StateSpacePtr Bundle_space = Bundle->getStateSpace();
        OMPL_ERROR("Base dimension: %d measure: %f", Base_space->getDimension(), Base_space->getMeasure());
        OMPL_ERROR("Bundle dimension: %d measure: %f", Bundle_space->getDimension(), Bundle_space->getMeasure());
        if (Fiber != nullptr)
        {
            const base::StateSpacePtr Fiber_space = Fiber->getStateSpace();
            OMPL_ERROR("Fiber dimension: %d measure: %f", Fiber_space->getDimension(), Fiber_space->getMeasure());
        }
        throw ompl::Exception("BundleSpace has no bounds");
    }
}

ompl::base::PlannerStatus ompl::geometric::BundleSpace::solve(const base::PlannerTerminationCondition &ptc)
{
    (void)ptc;
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
        opt_ = std::make_shared<base::PathLengthOptimizationObjective>(si_);
    }
}

void ompl::geometric::BundleSpace::resetCounter()
{
    BundleSpace::counter_ = 0;
}

void ompl::geometric::BundleSpace::mergeStates(const base::State *xBase, const base::State *xFiber, base::State *xBundle) const
{
    unsigned int M = components_.size();

    if( M > 1)
    {
        for(uint m = 0; m < M; m++)
        {
            const base::State *xmBase = xBase->as<base::CompoundState>()->as<base::State>(m);
            const base::State *xmFiber = xFiber->as<base::CompoundState>()->as<base::State>(m);
            base::State *xmBundle = xBundle->as<base::CompoundState>()->as<base::State>(m);
            components_.at(m)->mergeStates(xmBase, xmFiber, xmBundle);
        }
    }else{
        components_.front()->mergeStates(xBase, xFiber, xBundle);
    }
}

void ompl::geometric::BundleSpace::projectFiber(const base::State *xBundle, base::State *xFiber) const
{
    unsigned int M = components_.size();

    if( M > 1){
        for(uint m = 0; m < M; m++){
            const base::State *xmBundle = xBundle->as<base::CompoundState>()->as<base::State>(m);
            base::State *xmFiber = xFiber->as<base::CompoundState>()->as<base::State>(m);
            components_.at(m)->projectFiber(xmBundle, xmFiber);
        }
    }else{
        components_.front()->projectFiber(xBundle, xFiber);
    }
}

void ompl::geometric::BundleSpace::projectBase(const base::State *xBundle, base::State *xBase) const
{
    unsigned int M = components_.size();

    if( M > 1){
        for(uint m = 0; m < M; m++){
            const base::State *xmBundle = xBundle->as<base::CompoundState>()->as<base::State>(m);
            base::State *xmBase = xBase->as<base::CompoundState>()->as<base::State>(m);
            components_.at(m)->projectBase(xmBundle, xmBase);
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
      base::State *sk = s->as<base::CompoundState>()->as<base::State>(k);
      allocIdentityState(sk, spacek);
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
        exit(0);
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
  return allocIdentityState(Fiber->getStateSpace());
}

ompl::base::State* ompl::geometric::BundleSpace::allocIdentityStateBundle() const
{
  return allocIdentityState(Bundle->getStateSpace());
}

ompl::base::State* ompl::geometric::BundleSpace::allocIdentityStateBase() const
{
  return allocIdentityState(Base->getStateSpace());
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

unsigned int ompl::geometric::BundleSpace::getDimension() const
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

bool ompl::geometric::BundleSpace::sampleBase(base::State *xRandom)
{
    Bundle_sampler_->sampleUniform(xRandom);
    return true;
}

bool ompl::geometric::BundleSpace::sample(base::State *xRandom)
{
    bool valid = false;
    if (!hasParent())
    {
        // return Bundle_valid_sampler->sample(q_random);
        Bundle_sampler_->sampleUniform(xRandom);
        valid = Bundle->isValid(xRandom);
    }
    else
    {
        if (getFiberDimension() > 0)
        {
            // Adjusted sampling function: Sampling in G0 x Fiber
            Fiber_sampler_->sampleUniform(xFiberTmp_);
            parent_->sampleBase(xBaseTmp_);
            mergeStates(xBaseTmp_, xFiberTmp_, xRandom);
        }
        else
        {
            parent_->sampleBase(xRandom);
        }
        valid = Bundle->isValid(xRandom);
    }

    return valid;
}

void ompl::geometric::BundleSpace::print(std::ostream &out) const
{
    unsigned int M = components_.size();
    out << "[";
    for(unsigned int m = 0; m < M; m++){
        out << components_.at(m)->getTypeAsString() << (m<M-1?" |":"");
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
//void ompl::geometric::BundleSpace::mergeStates(
//        case SE3RN_SE3:
//        {
//            base::SE3StateSpace::StateType *sBundle_SE3 =
//                qBundle->as<base::CompoundState>()->as<base::SE3StateSpace::StateType>(0);
//            base::SO3StateSpace::StateType *sBundle_SE3_rotation = &sBundle_SE3->rotation();
//            base::RealVectorStateSpace::StateType *sBundle_RN =
//                qBundle->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

//            const base::SE3StateSpace::StateType *sBase = qBase->as<base::SE3StateSpace::StateType>();
//            const base::SO3StateSpace::StateType *sBase_rotation = &sBase->rotation();
//            const base::RealVectorStateSpace::StateType *sFiber = qFiber->as<base::RealVectorStateSpace::StateType>();

//            sBundle_SE3->setXYZ(sBase->getX(), sBase->getY(), sBase->getZ());
//            sBundle_SE3_rotation->x = sBase_rotation->x;
//            sBundle_SE3_rotation->y = sBase_rotation->y;
//            sBundle_SE3_rotation->z = sBase_rotation->z;
//            sBundle_SE3_rotation->w = sBase_rotation->w;

//            for (unsigned int k = 0; k < Fiber_dimension_; k++)
//            {
//                sBundle_RN->values[k] = sFiber->values[k];
//            }

//            break;
//        }

// void ompl::geometric::BundleSpace::projectBase(

//         case SE3RN_SE3:
//         {
//             const base::SE3StateSpace::StateType *sBundle_SE3 =
//                 q->as<base::CompoundState>()->as<base::SE3StateSpace::StateType>(0);
//             const base::SO3StateSpace::StateType *sBundle_SE3_rotation = &sBundle_SE3->rotation();

//             base::SE3StateSpace::StateType *sBase = qBase->as<base::SE3StateSpace::StateType>();
//             base::SO3StateSpace::StateType *sBase_rotation = &sBase->rotation();

//             sBase->setXYZ(sBundle_SE3->getX(), sBundle_SE3->getY(), sBundle_SE3->getZ());
//             sBase_rotation->x = sBundle_SE3_rotation->x;
//             sBase_rotation->y = sBundle_SE3_rotation->y;
//             sBase_rotation->z = sBundle_SE3_rotation->z;
//             sBase_rotation->w = sBundle_SE3_rotation->w;

//             break;
//         }
//             break;
//         }

// projectFiber
    // switch (type_)
    // {
    //     case SE2RN_SE2:
    //     case SE3RN_SE3:
    //     case SO2RN_SO2:
    //     {
    //         const base::RealVectorStateSpace::StateType *sBundle_RN =
    //             q->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);
    //         base::RealVectorStateSpace::StateType *sFiber = qFiber->as<base::RealVectorStateSpace::StateType>();

    //         for (unsigned int k = 0; k < Fiber_dimension_; k++)
    //         {
    //             sFiber->values[k] = sBundle_RN->values[k];
    //         }

    //         break;
    //     }
    //
    //
// const ompl::base::StateSpacePtr 
// ompl::geometric::BundleSpace::computeFiberSpace(
//         case SE2RN_SE2:
//         case SE3RN_SE3:
//         case SO2RN_SO2:
//         {
//             base::CompoundStateSpace *Bundle_compound = Bundle_in->as<base::CompoundStateSpace>();
//             const std::vector<base::StateSpacePtr> Bundle_decomposed = Bundle_compound->getSubspaces();

//             unsigned int NX = Bundle_decomposed.at(1)->getDimension();

//             Fiber_local = std::make_shared<base::RealVectorStateSpace>(NX);
//             std::static_pointer_cast<base::RealVectorStateSpace>(Fiber_local)->setBounds(
//                 std::static_pointer_cast<base::RealVectorStateSpace>(Bundle_decomposed.at(1))->getBounds());

//         }
//         case SE2RN_SE2RM:
//         case SO2RN_SO2RM:
//         case SE3RN_SE3RM:
//         {
//             base::CompoundStateSpace *Bundle_compound = Bundle_in->as<base::CompoundStateSpace>();
//             const std::vector<base::StateSpacePtr> Bundle_decomposed = Bundle_compound->getSubspaces();
//             base::CompoundStateSpace *Base_compound = Base_in->as<base::CompoundStateSpace>();
//             const std::vector<base::StateSpacePtr> Base_decomposed = Base_compound->getSubspaces();

//             unsigned int N = Bundle_decomposed.at(1)->getDimension();
//             unsigned int M = Base_decomposed.at(1)->getDimension();
//             unsigned int NX = N - M;
//             Fiber_local = std::make_shared<base::RealVectorStateSpace>(NX);

//             base::RealVectorBounds Bundle_bounds =
//                 std::static_pointer_cast<base::RealVectorStateSpace>(Bundle_decomposed.at(1))->getBounds();
//             std::vector<double> low;
//             low.resize(NX);
//             std::vector<double> high;
//             high.resize(NX);
//             base::RealVectorBounds Fiber_bounds(NX);
//             for (unsigned int k = 0; k < NX; k++)
//             {
//                 Fiber_bounds.setLow(k, Bundle_bounds.low.at(k + M));
//                 Fiber_bounds.setHigh(k, Bundle_bounds.high.at(k + M));
//             }
//             std::static_pointer_cast<base::RealVectorStateSpace>(Fiber_local)->setBounds(Fiber_bounds);
//             break;
//         }

