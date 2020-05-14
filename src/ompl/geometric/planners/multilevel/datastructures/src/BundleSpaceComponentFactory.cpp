#include <ompl/geometric/planners/multilevel/datastructures/BundleSpaceComponentFactory.h>

//XRN -> X
#include <ompl/geometric/planners/multilevel/datastructures/components/XRN_X_SO2.h>
#include <ompl/geometric/planners/multilevel/datastructures/components/XRN_X_SO3.h>
#include <ompl/geometric/planners/multilevel/datastructures/components/XRN_X_SE2.h>
#include <ompl/geometric/planners/multilevel/datastructures/components/XRN_X_SE3.h>

//XRN -> XRM
#include <ompl/geometric/planners/multilevel/datastructures/components/XRN_XRM_SO2.h>
#include <ompl/geometric/planners/multilevel/datastructures/components/XRN_XRM_SO3.h>
#include <ompl/geometric/planners/multilevel/datastructures/components/XRN_XRM_SE2.h>
#include <ompl/geometric/planners/multilevel/datastructures/components/XRN_XRM_SE3.h>

#include <ompl/geometric/planners/multilevel/datastructures/components/SE3_R3.h>
#include <ompl/geometric/planners/multilevel/datastructures/components/SE3RN_R3.h>
#include <ompl/geometric/planners/multilevel/datastructures/components/SE2_R2.h>
#include <ompl/geometric/planners/multilevel/datastructures/components/SE2RN_R2.h>

#include <ompl/geometric/planners/multilevel/datastructures/components/RN_RM.h>
#include <ompl/geometric/planners/multilevel/datastructures/components/RNSO2_RN.h>

#include <ompl/geometric/planners/multilevel/datastructures/components/None.h>
#include <ompl/geometric/planners/multilevel/datastructures/components/EmptySet.h>
#include <ompl/geometric/planners/multilevel/datastructures/components/Identity.h>
#include <ompl/geometric/planners/multilevel/datastructures/components/Relaxation.h>

#include <ompl/util/Exception.h>

std::vector<ompl::geometric::BundleSpaceComponentPtr> 
ompl::geometric::BundleSpaceComponentFactory::MakeBundleSpaceComponents(
    base::SpaceInformationPtr Bundle)
{
    const base::StateSpacePtr Bundle_space = Bundle->getStateSpace();
    int bundleSpaceComponents = GetNumberOfComponents(Bundle_space);

    std::vector<BundleSpaceComponentPtr> components;

    OMPL_DEBUG("Bundle components: %d", bundleSpaceComponents);

    if(bundleSpaceComponents > 1)
    {
        base::CompoundStateSpace *Bundle_compound = 
          Bundle_space->as<base::CompoundStateSpace>();
        const std::vector<base::StateSpacePtr> Bundle_decomposed = Bundle_compound->getSubspaces();

        for(int m = 0; m < bundleSpaceComponents; m++){
            base::StateSpacePtr BundleM = Bundle_decomposed.at(m);
            BundleSpaceComponentPtr componentM = MakeBundleSpaceComponent(BundleM);
            components.push_back(componentM);
        }
    }else{
        BundleSpaceComponentPtr component = MakeBundleSpaceComponent(Bundle_space);
        components.push_back(component);
    }

    return components;
}

std::vector<ompl::geometric::BundleSpaceComponentPtr> 
ompl::geometric::BundleSpaceComponentFactory::MakeBundleSpaceComponents(
    base::SpaceInformationPtr Bundle,
    base::SpaceInformationPtr Base)
{
    const base::StateSpacePtr Bundle_space = Bundle->getStateSpace();
    int bundleSpaceComponents = GetNumberOfComponents(Bundle_space);
    const base::StateSpacePtr Base_space = Base->getStateSpace();
    int baseSpaceComponents = GetNumberOfComponents(Base_space);

    if(baseSpaceComponents != bundleSpaceComponents)
    {
      Base->printSettings();
      OMPL_ERROR("Base Space has %d, but Bundle Space has %d components.", 
          baseSpaceComponents, bundleSpaceComponents);
      throw Exception("Different Number Of Components");
    }

    std::vector<BundleSpaceComponentPtr> components;

    //Check if planning spaces are equivalent, i.e. if (X, \phi) == (Y, \phi)
    bool areValidityCheckersEquivalent = false;
    if(*(Base->getStateValidityChecker().get()) == *(Bundle->getStateValidityChecker().get()))
    {
        areValidityCheckersEquivalent = true;
    }

    if(bundleSpaceComponents > 1){

      base::CompoundStateSpace *Bundle_compound = 
        Bundle_space->as<base::CompoundStateSpace>();
      base::CompoundStateSpace *Base_compound = 
        Base_space->as<base::CompoundStateSpace>();

      const std::vector<base::StateSpacePtr> Bundle_decomposed = Bundle_compound->getSubspaces();
      const std::vector<base::StateSpacePtr> Base_decomposed = Base_compound->getSubspaces();

      for(int m = 0; m < bundleSpaceComponents; m++){
        base::StateSpacePtr BaseM = Base_decomposed.at(m);
        base::StateSpacePtr BundleM = Bundle_decomposed.at(m);
        BundleSpaceComponentPtr componentM = MakeBundleSpaceComponent(BundleM, BaseM, 
            areValidityCheckersEquivalent);
        components.push_back(componentM);
      }

    }else{
      BundleSpaceComponentPtr component = MakeBundleSpaceComponent(Bundle_space, Base_space, 
          areValidityCheckersEquivalent);
      components.push_back(component);
    }
    return components;
}

ompl::geometric::BundleSpaceComponentPtr
ompl::geometric::BundleSpaceComponentFactory::MakeBundleSpaceComponent(
    const base::StateSpacePtr Bundle)
{
  return MakeBundleSpaceComponent(Bundle, nullptr, false);
}

ompl::geometric::BundleSpaceComponentPtr
ompl::geometric::BundleSpaceComponentFactory::MakeBundleSpaceComponent(
    const base::StateSpacePtr Bundle, 
    const base::StateSpacePtr Base,
    bool areValidityCheckersEquivalent)
{
    BundleSpaceComponentType type = identifyBundleSpaceComponentType(Bundle, Base);
    if(type == BUNDLE_SPACE_IDENTITY_PROJECTION && !areValidityCheckersEquivalent)
    {
        type = BUNDLE_SPACE_CONSTRAINED_RELAXATION;
    }

    BundleSpaceComponentPtr component;

    if(type == BUNDLE_SPACE_NO_PROJECTION){
      component = std::make_shared<BundleSpaceComponent_None>(Bundle, Base);
    }else if(type == BUNDLE_SPACE_EMPTY_SET_PROJECTION){
      component = std::make_shared<BundleSpaceComponent_EmptySet>(Bundle, Base);
    }else if(type == BUNDLE_SPACE_IDENTITY_PROJECTION){
      component = std::make_shared<BundleSpaceComponent_Identity>(Bundle, Base);
    }else if(type == BUNDLE_SPACE_CONSTRAINED_RELAXATION){
      component = std::make_shared<BundleSpaceComponent_Relaxation>(Bundle, Base);

    }else if(type == BUNDLE_SPACE_RN_RM){
      component = std::make_shared<BundleSpaceComponent_RN_RM>(Bundle, Base);
    }else if(type == BUNDLE_SPACE_RNSO2_RN){
      component = std::make_shared<BundleSpaceComponent_RNSO2_RN>(Bundle, Base);

    }else if(type == BUNDLE_SPACE_SE2_R2){
      component = std::make_shared<BundleSpaceComponent_SE2_R2>(Bundle, Base);
    }else if(type == BUNDLE_SPACE_SE2RN_R2){
      component = std::make_shared<BundleSpaceComponent_SE2RN_R2>(Bundle, Base);
    }else if(type == BUNDLE_SPACE_SE2RN_SE2){
      component = std::make_shared<BundleSpaceComponent_SE2RN_SE2>(Bundle, Base);
    }else if(type == BUNDLE_SPACE_SE2RN_SE2RM){
      component = std::make_shared<BundleSpaceComponent_SE2RN_SE2RM>(Bundle, Base);

    }else if(type == BUNDLE_SPACE_SO2RN_SO2){
      component = std::make_shared<BundleSpaceComponent_SO2RN_SO2>(Bundle, Base);
    }else if(type == BUNDLE_SPACE_SO2RN_SO2RM){
      component = std::make_shared<BundleSpaceComponent_SO2RN_SO2RM>(Bundle, Base);

    }else if(type == BUNDLE_SPACE_SO3RN_SO3){
      component = std::make_shared<BundleSpaceComponent_SO3RN_SO3>(Bundle, Base);
    }else if(type == BUNDLE_SPACE_SO3RN_SO3RM){
      component = std::make_shared<BundleSpaceComponent_SO3RN_SO3RM>(Bundle, Base);

    }else if(type == BUNDLE_SPACE_SE3_R3){
      component = std::make_shared<BundleSpaceComponent_SE3_R3>(Bundle, Base);
    }else if(type == BUNDLE_SPACE_SE3RN_R3){
      component = std::make_shared<BundleSpaceComponent_SE3RN_R3>(Bundle, Base);
    }else if(type == BUNDLE_SPACE_SE3RN_SE3){
      component = std::make_shared<BundleSpaceComponent_SE3RN_SE3>(Bundle, Base);
    }else if(type == BUNDLE_SPACE_SE3RN_SE3RM){
      component = std::make_shared<BundleSpaceComponent_SE3RN_SE3RM>(Bundle, Base);
    }else{
      OMPL_ERROR("NYI: %d", type);
      throw Exception("BundleSpaceType not yet implemented.");
    }
    component->setType(type);
    component->initFiberSpace();
    return component;
}

ompl::geometric::BundleSpaceComponentType
ompl::geometric::BundleSpaceComponentFactory::identifyBundleSpaceComponentType(const base::StateSpacePtr Bundle, const base::StateSpacePtr Base)
{
    if(Base == nullptr)
    {
        return BUNDLE_SPACE_NO_PROJECTION;
    }

    if(isMapping_Identity(Bundle, Base))
    {
        return BUNDLE_SPACE_IDENTITY_PROJECTION;
    }

    if(isMapping_EmptyProjection(Bundle, Base))
    {
        return BUNDLE_SPACE_EMPTY_SET_PROJECTION;
    }

    //RN ->
    if(isMapping_RN_to_RM(Bundle, Base))
    {
        return BUNDLE_SPACE_RN_RM;
    }
    if(isMapping_RNSO2_to_RN(Bundle, Base))
    {
        return BUNDLE_SPACE_RNSO2_RN;
    }

    //SE3 ->
    if(isMapping_SE3_to_R3(Bundle, Base))
    {
        return BUNDLE_SPACE_SE3_R3;
    }
    if(isMapping_SE3RN_to_SE3(Bundle, Base))
    {
        return BUNDLE_SPACE_SE3RN_SE3;
    }
    if(isMapping_SE3RN_to_R3(Bundle, Base))
    {
        return BUNDLE_SPACE_SE3RN_R3;
    }
    if(isMapping_SE3RN_to_SE3RM(Bundle, Base))
    {
        return BUNDLE_SPACE_SE3RN_SE3RM;
    }

    //SE2 ->
    if(isMapping_SE2_to_R2(Bundle, Base))
    {
        return BUNDLE_SPACE_SE2_R2;
    }
    if(isMapping_SE2RN_to_SE2(Bundle, Base))
    {
        return BUNDLE_SPACE_SE2RN_SE2;
    }
    if(isMapping_SE2RN_to_R2(Bundle, Base))
    {
        return BUNDLE_SPACE_SE2RN_R2;
    }
    if(isMapping_SE2RN_to_SE2RM(Bundle, Base))
    {
        return BUNDLE_SPACE_SE2RN_SE2RM;
    }

    //SO2 ->
    if(isMapping_SO2RN_to_SO2(Bundle, Base))
    {
        return BUNDLE_SPACE_SO2RN_SO2;
    }
    if(isMapping_SO2RN_to_SO2RM(Bundle, Base))
    {
        return BUNDLE_SPACE_SO2RN_SO2RM;
    }

    //SO3 ->
    if(isMapping_SO3RN_to_SO3(Bundle, Base))
    {
        return BUNDLE_SPACE_SO3RN_SO3;
    }
    if(isMapping_SO3RN_to_SO3RM(Bundle, Base))
    {
        return BUNDLE_SPACE_SO3RN_SO3RM;
    }

    OMPL_ERROR("Fiber Bundle unknown.");
    return BUNDLE_SPACE_UNKNOWN;
}


bool ompl::geometric::BundleSpaceComponentFactory::isMapping_Identity(
    const base::StateSpacePtr Bundle, 
    const base::StateSpacePtr Base)
{
    if(Bundle->isCompound())
    {
      if(Base->isCompound())
      {
          base::CompoundStateSpace *Bundle_compound = Bundle->as<base::CompoundStateSpace>();
          const std::vector<base::StateSpacePtr> Bundle_decomposed = Bundle_compound->getSubspaces();
          base::CompoundStateSpace *Base_compound = Base->as<base::CompoundStateSpace>();
          const std::vector<base::StateSpacePtr> Base_decomposed = Base_compound->getSubspaces();

          if (Bundle_decomposed.size() == Base_decomposed.size())
          {
              for(uint k = 0; k < Bundle_decomposed.size(); k++)
              {
                  if(!isMapping_Identity(Bundle_decomposed.at(k), Base_decomposed.at(k)))
                  {
                      return false;
                  }
              }
          }
          return true;
      }
    }else{
      if((Base->getType() == Bundle->getType()) &&
          (Base->getDimension() == Bundle->getDimension()))
      {
          return true;
      }
    }
    return false;
}

bool ompl::geometric::BundleSpaceComponentFactory::isMapping_RN_to_RM(
    const base::StateSpacePtr Bundle, 
    const base::StateSpacePtr Base)
{
    if(Bundle->isCompound()) return false;

    if (Bundle->getType() == base::STATE_SPACE_REAL_VECTOR)
    {
        unsigned int n = Bundle->getDimension();
        if (Base->getType() == base::STATE_SPACE_REAL_VECTOR)
        {
            unsigned int m = Base->getDimension();
            if (n > m && m > 0)
            {
                return true;
            }
        }
    }
    return false;
}

bool ompl::geometric::BundleSpaceComponentFactory::isMapping_SE3_to_R3(
    const base::StateSpacePtr Bundle, 
    const base::StateSpacePtr Base)
{
    if(!Bundle->isCompound()) return false;

    if (Bundle->getType() == base::STATE_SPACE_SE3)
    {
        if (Base->getType() == base::STATE_SPACE_REAL_VECTOR)
        {
            if (Base->getDimension() == 3)
            {
                return true;
            }
        }
    }
    return false;
}
bool ompl::geometric::BundleSpaceComponentFactory::isMapping_SE3RN_to_R3(
    const base::StateSpacePtr Bundle, 
    const base::StateSpacePtr Base)
{
    if(!Bundle->isCompound()) return false;

    base::CompoundStateSpace *Bundle_compound = Bundle->as<base::CompoundStateSpace>();
    const std::vector<base::StateSpacePtr> Bundle_decomposed = Bundle_compound->getSubspaces();
    unsigned int Bundle_subspaces = Bundle_decomposed.size();
    if (Bundle_subspaces == 2)
    {
        if (Bundle_decomposed.at(0)->getType() == base::STATE_SPACE_SE3 &&
            Bundle_decomposed.at(1)->getType() == base::STATE_SPACE_REAL_VECTOR)
        {
            if (Base->getType() == base::STATE_SPACE_REAL_VECTOR)
            {
                unsigned int m = Base->getDimension();
                if (m == 3)
                {
                    return true;
                }
            }
        }
    }
    return false;
}

bool ompl::geometric::BundleSpaceComponentFactory::isMapping_SE2_to_R2(
    const base::StateSpacePtr Bundle, 
    const base::StateSpacePtr Base)
{
    if(!Bundle->isCompound()) return false;

    if (Bundle->getType() == base::STATE_SPACE_SE2)
    {
        if (Base->getType() == base::STATE_SPACE_REAL_VECTOR)
        {
            if (Base->getDimension() == 2)
            {
                return true;
            }
        }
    }
    return false;
}

bool ompl::geometric::BundleSpaceComponentFactory::isMapping_RNSO2_to_RN(
    const base::StateSpacePtr Bundle, 
    const base::StateSpacePtr Base)
{
    if(!Bundle->isCompound()) return false;

    base::CompoundStateSpace *Bundle_compound = Bundle->as<base::CompoundStateSpace>();
    const std::vector<base::StateSpacePtr> Bundle_decomposed = Bundle_compound->getSubspaces();
    unsigned int Bundle_subspaces = Bundle_decomposed.size();
    if (Bundle_subspaces == 2)
    {
        if (Bundle_decomposed.at(0)->getType() == base::STATE_SPACE_REAL_VECTOR &&
            Bundle_decomposed.at(1)->getType() == base::STATE_SPACE_SO2)
        {
            if (Base->getType() == base::STATE_SPACE_REAL_VECTOR)
            {
                unsigned int n = Bundle_decomposed.at(0)->getDimension();
                unsigned int m = Base->getDimension();
                if (m == n)
                {
                    return true;
                }
            }
        }
    }
    return false;
}

bool ompl::geometric::BundleSpaceComponentFactory::isMapping_SE2RN_to_R2(
    const base::StateSpacePtr Bundle, 
    const base::StateSpacePtr Base)
{
    if(!Bundle->isCompound()) return false;

    base::CompoundStateSpace *Bundle_compound = Bundle->as<base::CompoundStateSpace>();
    const std::vector<base::StateSpacePtr> Bundle_decomposed = Bundle_compound->getSubspaces();
    unsigned int Bundle_subspaces = Bundle_decomposed.size();
    if (Bundle_subspaces == 2)
    {
        if (Bundle_decomposed.at(0)->getType() == base::STATE_SPACE_SE2 &&
            Bundle_decomposed.at(1)->getType() == base::STATE_SPACE_REAL_VECTOR)
        {
            if (Base->getType() == base::STATE_SPACE_REAL_VECTOR)
            {
                unsigned int m = Base->getDimension();
                if (m == 2)
                {
                    return true;
                }
            }
        }
    }
    return false;
}

bool ompl::geometric::BundleSpaceComponentFactory::isMapping_SE2RN_to_SE2(
    const base::StateSpacePtr Bundle, 
    const base::StateSpacePtr Base)
{
    return isMapping_XRN_to_X(Bundle, Base, base::STATE_SPACE_SE2);
}

bool ompl::geometric::BundleSpaceComponentFactory::isMapping_SE3RN_to_SE3(
    const base::StateSpacePtr Bundle, 
    const base::StateSpacePtr Base)
{
    return isMapping_XRN_to_X(Bundle, Base, base::STATE_SPACE_SE3);
}

bool ompl::geometric::BundleSpaceComponentFactory::isMapping_SO2RN_to_SO2(
    const base::StateSpacePtr Bundle, 
    const base::StateSpacePtr Base)
{
    return isMapping_XRN_to_X(Bundle, Base, base::STATE_SPACE_SO2);
}

bool ompl::geometric::BundleSpaceComponentFactory::isMapping_SO3RN_to_SO3(
    const base::StateSpacePtr Bundle, 
    const base::StateSpacePtr Base)
{
    return isMapping_XRN_to_X(Bundle, Base, base::STATE_SPACE_SO3);
}


bool ompl::geometric::BundleSpaceComponentFactory::isMapping_SE2RN_to_SE2RM(
    const base::StateSpacePtr Bundle, 
    const base::StateSpacePtr Base)
{
    return isMapping_XRN_to_XRM(Bundle, Base, base::STATE_SPACE_SE2);
}

bool ompl::geometric::BundleSpaceComponentFactory::isMapping_SE3RN_to_SE3RM(
    const base::StateSpacePtr Bundle, 
    const base::StateSpacePtr Base)
{
    return isMapping_XRN_to_XRM(Bundle, Base, base::STATE_SPACE_SE3);
}

bool ompl::geometric::BundleSpaceComponentFactory::isMapping_SO2RN_to_SO2RM(
    const base::StateSpacePtr Bundle, 
    const base::StateSpacePtr Base)
{
    return isMapping_XRN_to_XRM(Bundle, Base, base::STATE_SPACE_SO2);
}

bool ompl::geometric::BundleSpaceComponentFactory::isMapping_SO3RN_to_SO3RM(
    const base::StateSpacePtr Bundle, 
    const base::StateSpacePtr Base)
{
    return isMapping_XRN_to_XRM(Bundle, Base, base::STATE_SPACE_SO3);
}

bool ompl::geometric::BundleSpaceComponentFactory::isMapping_XRN_to_X(
    const base::StateSpacePtr Bundle, 
    const base::StateSpacePtr Base,
    const base::StateSpaceType type)
{
    if(!Bundle->isCompound()) return false;

    base::CompoundStateSpace *Bundle_compound = Bundle->as<base::CompoundStateSpace>();
    const std::vector<base::StateSpacePtr> Bundle_decomposed = Bundle_compound->getSubspaces();
    unsigned int Bundle_subspaces = Bundle_decomposed.size();
    if (Bundle_subspaces == 2)
    {
        if (Bundle_decomposed.at(0)->getType() == type &&
            Bundle_decomposed.at(1)->getType() == base::STATE_SPACE_REAL_VECTOR)
        {
            if (Base->getType() == type)
            {
                return true;
            }
        }
    }
    return false;
}

bool ompl::geometric::BundleSpaceComponentFactory::isMapping_XRN_to_XRM(
    const base::StateSpacePtr Bundle, 
    const base::StateSpacePtr Base,
    const base::StateSpaceType type)
{
    if(!Bundle->isCompound()) return false;

    base::CompoundStateSpace *Bundle_compound = Bundle->as<base::CompoundStateSpace>();
    const std::vector<base::StateSpacePtr> Bundle_decomposed = Bundle_compound->getSubspaces();
    if (Bundle_decomposed.size() == 2)
    {
        if (Bundle_decomposed.at(0)->getType() == type &&
            Bundle_decomposed.at(1)->getType() == base::STATE_SPACE_REAL_VECTOR)
        {
            if(!Base->isCompound()) return false;
            unsigned int n = Bundle_decomposed.at(1)->getDimension();

            base::CompoundStateSpace *Base_compound = Base->as<base::CompoundStateSpace>();
            const std::vector<base::StateSpacePtr> Base_decomposed = Base_compound->getSubspaces();
            if (Base_decomposed.size() == 2)
            {
                if (Base_decomposed.at(0)->getType() == type &&
                    Base_decomposed.at(1)->getType() == base::STATE_SPACE_REAL_VECTOR)
                {
                    unsigned int m = Base_decomposed.at(1)->getDimension();
                    if (n > m && m > 0)
                    {
                        return true;
                    }
                }
            }
        }
    }
    return false;
}


bool ompl::geometric::BundleSpaceComponentFactory::isMapping_EmptyProjection(
    const base::StateSpacePtr, 
    const base::StateSpacePtr Base)
{
    if(Base == nullptr || Base->getDimension() <= 0)
    {
        return true;
    }
    return false;
}
int ompl::geometric::BundleSpaceComponentFactory::GetNumberOfComponents(base::StateSpacePtr space)
{
  int nrComponents = 0;

  if(space->isCompound()){
    base::CompoundStateSpace *compound = space->as<base::CompoundStateSpace>();
    nrComponents = compound->getSubspaceCount();
    if(nrComponents == 2)
    {
      int type = space->getType();

      if(
          (type == base::STATE_SPACE_SE2) || 
          (type == base::STATE_SPACE_SE3) || 
          (type == base::STATE_SPACE_DUBINS) || 
          (type == base::STATE_SPACE_DUBINS_AIRPLANE)
        )
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
          if(decomposed.at(1)->getDimension() > 0)
          {
            nrComponents = 1;
          }
        }
      }
    }
  }else{
    nrComponents = 1;
  }
  return nrComponents;
}
