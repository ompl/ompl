#include <ompl/geometric/planners/multilevel/datastructures/pathrestriction/PathRestriction.h>
#include <ompl/geometric/planners/multilevel/datastructures/graphsampler/GraphSampler.h>

#include <ompl/base/Path.h>
#include <ompl/geometric/PathGeometric.h>

#define SANITY_CHECK(X) X

ompl::geometric::BundleSpacePathRestriction::BundleSpacePathRestriction(BundleSpaceGraph* bundleSpaceGraph):
  bundleSpaceGraph_(bundleSpaceGraph)
{
    if(bundleSpaceGraph_->getFiberDimension() > 0)
    {
        xFiberStart_ = bundleSpaceGraph_->getFiber()->allocState();
        xFiberGoal_ = bundleSpaceGraph_->getFiber()->allocState();
        xFiberTmp_ = bundleSpaceGraph_->getFiber()->allocState();
    }
    if(bundleSpaceGraph_->getBaseDimension() > 0)
    {
        xBaseTmp_ = bundleSpaceGraph_->getBase()->allocState();
    }
    xBundleTmp_ = bundleSpaceGraph_->getBundle()->allocState();

    lastValid_.first = bundleSpaceGraph_->getBundle()->allocState();
}

ompl::geometric::BundleSpacePathRestriction::~BundleSpacePathRestriction()
{
    if(bundleSpaceGraph_->isDynamic()) return;

    if(bundleSpaceGraph_->getFiberDimension() > 0)
    {
        bundleSpaceGraph_->getFiber()->freeState(xFiberStart_);
        bundleSpaceGraph_->getFiber()->freeState(xFiberGoal_);
        bundleSpaceGraph_->getFiber()->freeState(xFiberTmp_);
    }
    if(bundleSpaceGraph_->getBaseDimension() > 0)
    {
        bundleSpaceGraph_->getBase()->freeState(xBaseTmp_);
    }
    bundleSpaceGraph_->getBundle()->freeState(lastValid_.first);
    bundleSpaceGraph_->getBundle()->freeState(xBundleTmp_);
}

void ompl::geometric::BundleSpacePathRestriction::setBasePath(ompl::base::PathPtr path)
{
    PathGeometricPtr geometricBasePath = std::static_pointer_cast<PathGeometric>(path);
    setBasePath(geometricBasePath->getStates());
}

void ompl::geometric::BundleSpacePathRestriction::setBasePath(
    std::vector<base::State*> basePath)
{
    basePath_ = basePath;

    lengthBasePath_ = 0.0;
    intermediateLengthsBasePath_.clear();
    for(uint k = 1; k < basePath_.size(); k++){
      double lk = 
        bundleSpaceGraph_->getBase()->distance(
            basePath_.at(k-1), 
            basePath_.at(k));
      intermediateLengthsBasePath_.push_back(lk);
      lengthBasePath_ += lk;
    }
    OMPL_INFORM("Set new base path with %d states and length %f.", 
        basePath_.size(), 
        lengthBasePath_);
}

#include <ompl/base/spaces/SO3StateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <unsupported/Eigen/Splines>
#include <Eigen/Geometry> 

void QuaternionToDirectionVector(
  const Eigen::Quaternion<double> q,
  Eigen::Vector3d &v)
{
  Eigen::Vector3d ex(1,0,0);
  v = q.toRotationMatrix()*ex;
}
void DirectionVectorToQuaternion(
  const Eigen::Vector3d v,
  Eigen::Quaternion<double> &q)
{
  Eigen::Vector3d ex(1,0,0);
  q = Eigen::Quaternion<double>::FromTwoVectors(ex, v);
}

std::vector<ompl::base::State*> 
ompl::geometric::BundleSpacePathRestriction::interpolateQuasiSectionSpline(
      const base::State* xFiberStart,
      const base::State* xFiberGoal,
      const std::vector<base::State*> basePath) 
{
    
  if(bundleSpaceGraph_->getBase()->getStateSpace()->getType() != base::STATE_SPACE_REAL_VECTOR)
  {
      throw Exception("NYI");
  }

  const int dim = 3;
  typedef Eigen::Spline<double, dim> SplineNd;
  typedef Eigen::SplineFitting<SplineNd> SplineFittingNd;

  typedef SplineNd::PointType PointType;
  typedef SplineNd::KnotVectorType KnotVectorType;
  typedef SplineNd::ControlPointVectorType ControlPointVectorType;

  // T1 exp(t log(T1^-1 T2))

  //############################################################################
  //BezierCurveFit through BasePath
  //############################################################################

  // double spacingBetweenPoints = 1.0/double(basePath.size()-1);
  // Eigen::RowVectorXd time = Eigen::RowVectorXd::Constant(basePath.size() - 1, spacingBetweenPoints);

  Eigen::MatrixXd states = Eigen::MatrixXd::Zero(dim, basePath.size());
  for(uint j = 0; j < basePath.size(); j++)
  {
      base::State* bj = basePath.at(j);
      double*& v = bj->as<base::RealVectorStateSpace::StateType>()->values;
      for(uint k = 0; k < dim; k++){
          states(k,j) = v[k];
      }
  }
  std::cout << states << std::endl;

  //############################################################################
  //Extract Derivatives
  //############################################################################
  std::cout << std::string(80, '-') << std::endl;
  bundleSpaceGraph_->getFiber()->printState(xFiberStart);
  bundleSpaceGraph_->getFiber()->printState(xFiberGoal);
  std::cout << std::string(80, '-') << std::endl;

  const base::SO3StateSpace::StateType *xFiberStart_SO3 =
       xFiberStart->as<base::CompoundState>()->as<base::SO3StateSpace::StateType>(0);
  const base::SO3StateSpace::StateType *xFiberGoal_SO3 =
       xFiberGoal->as<base::CompoundState>()->as<base::SO3StateSpace::StateType>(0);

  Eigen::Quaternion<double> qStart(
      xFiberStart_SO3->w, 
      xFiberStart_SO3->x,
      xFiberStart_SO3->y,
      xFiberStart_SO3->z);
  Eigen::Quaternion<double> qGoal(
      xFiberGoal_SO3->w, 
      xFiberGoal_SO3->x,
      xFiberGoal_SO3->y,
      xFiberGoal_SO3->z);

  Eigen::Vector3d vStart, vGoal;
  QuaternionToDirectionVector(qStart, vStart);
  QuaternionToDirectionVector(qGoal, vGoal);

  //############################################################################
  //Eval Spline
  //############################################################################

  Eigen::MatrixXd derivatives = Eigen::MatrixXd::Zero(dim, 2);

  derivatives.col(0) = vStart;
  derivatives.col(1) = vGoal;

  Eigen::VectorXi indices(2);
      indices << 0, basePath.size() - 1;

  const SplineNd spline = 
    SplineFittingNd::InterpolateWithDerivatives(states, derivatives, indices, 3);


  //############################################################################
  //Convert to OMPL
  //############################################################################
  int N = 100;
  std::vector<base::State*> bundlePath;
  bundlePath.resize(N);
  bundleSpaceGraph_->getBundle()->allocStates(bundlePath);

  double d = 0;
  double dstep = 1.0/double(N-1);
  for(uint j = 0; j < N; j++)
  {
      PointType pt = spline(d);

      base::State* bj = bundlePath.at(j);

      base::SE3StateSpace::StateType *xBundle_SE3 =
         bj->as<base::CompoundState>()->as<base::SE3StateSpace::StateType>(0);
      xBundle_SE3->setXYZ(pt(0), pt(1), pt(2));

      base::SO3StateSpace::StateType *xBundle_SO3 = &xBundle_SE3->rotation();

      // PointType deriv = spline.derivatives(d, 1);
      Eigen::Vector3d dx;
      dx = (spline(d + dstep) - spline(d))/dstep;
      dx = dx.normalized();

      std::cout << "Derivative:" << std::endl;
      std::cout << dx << std::endl;

      Eigen::Quaternion<double> qj;
      DirectionVectorToQuaternion(dx, qj);

      xBundle_SO3->x = qj.x();
      xBundle_SO3->y = qj.y();
      xBundle_SO3->z = qj.z();
      xBundle_SO3->w = qj.w();

      base::RealVectorStateSpace::StateType *xBundle_RN =
         bj->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

      int K = bundleSpaceGraph_->getBundleDimension() - 6;
      for (unsigned int k = 0; k < K; k++)
      {
          xBundle_RN->values[k] = 0;
      }
      xBundle_RN->values[0] = 1;

      std::cout << std::string(80, '-') << std::endl;
      bundleSpaceGraph_->getBundle()->printState(bj);

      d += dstep;
  }
  return bundlePath;

}
std::vector<ompl::base::State*> 
ompl::geometric::BundleSpacePathRestriction::interpolateSectionL1(
      const base::State* xFiberStart,
      const base::State* xFiberGoal,
      const std::vector<base::State*> basePath) 
{
    std::vector<base::State*> bundlePath;

    if(bundleSpaceGraph_->getFiberDimension() > 0)
    {
        bundlePath.resize(basePath.size() + 1);
        bundleSpaceGraph_->getBundle()->allocStates(bundlePath);
        for(uint k = 0; k < bundlePath.size()-1; k++)
        {
            bundleSpaceGraph_->liftState(basePath.at(k), xFiberStart, bundlePath.at(k));
        }
        bundleSpaceGraph_->liftState(basePath.back(), xFiberGoal, bundlePath.back());

    }else{
        bundlePath.resize(basePath.size());
        bundleSpaceGraph_->getBundle()->allocStates(bundlePath);
        for(uint k = 0; k < basePath.size(); k++){
            bundleSpaceGraph_->getBundle()->copyState(bundlePath.at(k), basePath.at(k));
        }
    }
    return bundlePath;
}

std::vector<ompl::base::State*> 
ompl::geometric::BundleSpacePathRestriction::interpolateSectionL1_FiberFirst(
      const base::State* xFiberStart,
      const base::State* xFiberGoal,
      const std::vector<base::State*> basePath)
{
    std::vector<base::State*> bundlePath;

    if(bundleSpaceGraph_->getFiberDimension() > 0)
    {
        bundlePath.resize(basePath.size() + 1);
        bundleSpaceGraph_->getBundle()->allocStates(bundlePath);

        bundleSpaceGraph_->liftState(basePath.front(), xFiberStart, bundlePath.front());
        for(uint k = 1; k < bundlePath.size(); k++)
        {
            bundleSpaceGraph_->liftState(basePath.at(k-1), xFiberGoal, bundlePath.at(k));
        }

    }else{
        bundlePath.resize(basePath.size());
        bundleSpaceGraph_->getBundle()->allocStates(bundlePath);
        for(uint k = 0; k < basePath.size(); k++){
            bundleSpaceGraph_->getBundle()->copyState(bundlePath.at(k), basePath.at(k));
        }
    }
    return bundlePath;
}

std::vector<ompl::base::State*> 
ompl::geometric::BundleSpacePathRestriction::interpolateSectionL2(
      const base::State* xFiberStart,
      const base::State* xFiberGoal,
      const std::vector<base::State*> basePath) 
{
    std::vector<base::State*> bundlePath;
    bundlePath.resize(basePath.size());
    bundleSpaceGraph_->getBundle()->allocStates(bundlePath);

    double totalLengthBasePath = 0.0;
    for(uint k = 1; k < basePath.size(); k++){
        totalLengthBasePath += bundleSpaceGraph_->getBase()->distance(
            basePath.at(k-1), basePath.at(k));
    }

    if(bundleSpaceGraph_->getFiberDimension() > 0)
    {
        double lengthCurrent = 0;

        for(uint k = 0; k < basePath.size(); k++)
        {
            double step = lengthCurrent / totalLengthBasePath;

            bundleSpaceGraph_->getFiber()->getStateSpace()->interpolate(
                xFiberStart, xFiberGoal, step, xFiberTmp_);

            bundleSpaceGraph_->liftState(basePath.at(k), xFiberTmp_, bundlePath.at(k));

            if(k < basePath.size() - 1)
            {
                lengthCurrent += bundleSpaceGraph_->getBase()->distance(basePath.at(k), basePath.at(k+1));
            }
        }

    }else{
        for(uint k = 0; k < basePath.size(); k++){
            bundleSpaceGraph_->getBundle()->copyState(bundlePath.at(k), basePath.at(k));
        }
    }
    return bundlePath;
}

ompl::geometric::BundleSpaceGraph::Configuration* 
ompl::geometric::BundleSpacePathRestriction::addFeasibleSegment(
    const Configuration* xLast, 
    base::State *sNext)
{
    Configuration *x = new Configuration(bundleSpaceGraph_->getBundle(), sNext);
    bundleSpaceGraph_->addConfiguration(x);
    bundleSpaceGraph_->addBundleEdge(xLast, x);
    
    SANITY_CHECK(
        if(!bundleSpaceGraph_->getBundle()->checkMotion(xLast->state, x->state))
        {
            OMPL_ERROR("Not feasible from last");
            std::cout << std::string(80, '-') << std::endl;
            std::cout << "Last State" << std::endl;
            bundleSpaceGraph_->getBundle()->printState(xLast->state);
            std::cout << std::string(80, '-') << std::endl;
            std::cout << "Current State" << std::endl;
            bundleSpaceGraph_->getBundle()->printState(x->state);
            throw Exception("");
        }
    );
    return x;
}

void
ompl::geometric::BundleSpacePathRestriction::addFeasibleGoalSegment(
    Configuration* const xLast, 
    Configuration* const xGoal)
{
    if(xGoal->index <= 0)
    {
        bundleSpaceGraph_->vGoal_ = bundleSpaceGraph_->addConfiguration(xGoal);
    }
    bundleSpaceGraph_->addBundleEdge(xLast, xGoal);

    if(!bundleSpaceGraph_->getBundle()->checkMotion(xLast->state, xGoal->state))
    {
        std::cout << std::string(80, '-') << std::endl;
        std::cout << "Last State" << std::endl;
        bundleSpaceGraph_->getBundle()->printState(xLast->state);
        std::cout << std::string(80, '-') << std::endl;
        std::cout << "Current State" << std::endl;
        bundleSpaceGraph_->getBundle()->printState(xGoal->state);
        throw Exception("Infeasible goal segment.");
    }
}

bool ompl::geometric::BundleSpacePathRestriction::sideStepAlongFiber(const base::State* xBase, base::State* xBundle)
{
    int ctr = 0;
    bool found = false;
    while(ctr++ < 10 && !found)
    {
        //sample model fiber
        bundleSpaceGraph_->sampleFiber(xFiberStart_);
        bundleSpaceGraph_->liftState(xBase, xFiberStart_, xBundle);

        if(bundleSpaceGraph_->getBundle()->isValid(xBundle))
        {
            found = true;
        }
    }
    return found;
}

void ompl::geometric::BundleSpacePathRestriction::sanityCheckSection()
{
    if (!bundleSpaceGraph_->sameComponent(bundleSpaceGraph_->vStart_, bundleSpaceGraph_->vGoal_))
    {
        throw Exception("Reported feasible path section, \
            but start and goal are in different components.");
    }

    base::PathPtr solutionPath = bundleSpaceGraph_->getPath(bundleSpaceGraph_->vStart_, bundleSpaceGraph_->vGoal_);

    if(solutionPath == nullptr)
    {
        std::cout 
          << bundleSpaceGraph_->getName()
          << " failed on level " << bundleSpaceGraph_->getLevel() 
          << " dim " << bundleSpaceGraph_->getBundleDimension() 
          << "->" << bundleSpaceGraph_->getBaseDimension() 
          << std::endl;
        throw Exception("Reported feasible path section, \
            but path section is not existent.");
    }

    geometric::PathGeometric &gpath = static_cast<geometric::PathGeometric &>(*solutionPath);

    bool valid = gpath.check();
    if(!valid)
    {
        OMPL_ERROR("Path section is invalid.");
        std::vector<base::State*> gStates = gpath.getStates();
        for(uint k = 1; k < gStates.size(); k++){
          base::State *sk1 = gStates.at(k-1);
          base::State *sk2 = gStates.at(k-2);
          if(!bundleSpaceGraph_->getBundle()->checkMotion(sk1, sk2))
          {
            std::cout << "Error between states " << k-1 << " and " << k << std::endl;
          }
        }
        throw Exception("Reported feasible path section, \
            but path section is infeasible.");
    }
}

bool ompl::geometric::BundleSpacePathRestriction::hasFeasibleSection(
      Configuration* const xStart,
      Configuration* const xGoal) 
{
    if(bundleSpaceGraph_->isDynamic())
    {
        bundleSpaceGraph_->projectFiber(xStart->state, xFiberStart_);
        bundleSpaceGraph_->projectFiber(xGoal->state, xFiberGoal_);
        std::vector<base::State*> section = 
          interpolateQuasiSectionSpline(xFiberStart_, xFiberGoal_, basePath_);

        Configuration *xLast = xStart;

        for(uint k = 1; k < section.size(); k++)
        {
            if(k < section.size()-1)
            {
                // xLast = addFeasibleSegment(xLast, section.at(k));
                Configuration *x = new Configuration(bundleSpaceGraph_->getBundle(), section.at(k));
                bundleSpaceGraph_->addConfiguration(x);
                bundleSpaceGraph_->addBundleEdge(xLast, x);
                xLast = x;

            }else{
                if(xGoal->index <= 0)
                {
                    bundleSpaceGraph_->vGoal_ = bundleSpaceGraph_->addConfiguration(xGoal);
                }

                bundleSpaceGraph_->addBundleEdge(xLast, xGoal);
                // addFeasibleGoalSegment(xLast, xGoal);
                OMPL_DEBUG("Found feasible path section (%d edges added)", k);
                return true;
            }

        }
        return true;
    }else{

        bool foundFeasibleSection = checkSectionRecursiveRepair(xStart, xGoal, basePath_);
        if(!foundFeasibleSection)
        {
          //Try with inverse L1 
            foundFeasibleSection = checkSectionRecursiveRepair(xStart, xGoal, basePath_, false);
        }

        SANITY_CHECK(
            if(foundFeasibleSection)
            {
                sanityCheckSection();
            }
        );

        return foundFeasibleSection;
    }
}

bool ompl::geometric::BundleSpacePathRestriction::checkSection(
      Configuration* const xStart,
      Configuration* const xGoal)
{
    bundleSpaceGraph_->projectFiber(xStart->state, xFiberStart_);
    bundleSpaceGraph_->projectFiber(xGoal->state, xFiberGoal_);

    std::vector<base::State*> section = 
      interpolateSectionL2(xFiberStart_, xFiberGoal_, basePath_);

    Configuration *xLast = xStart;

    bool found = false;
    for(uint k = 1; k < section.size(); k++)
    {
        if(bundleSpaceGraph_->getBundle()->checkMotion(
              section.at(k-1), section.at(k), lastValid_))
        {
            if(k < section.size()-1)
            {
                xLast = addFeasibleSegment(xLast, section.at(k));
            }else{
                addFeasibleGoalSegment(xLast, xGoal);
                OMPL_DEBUG("Found feasible path section (%d edges added)", k);
                found = true;
                break;
            }
        }else{
            if(lastValid_.second > 0)
            {
                //add last valid into the bundle graph
                Configuration *xk = new Configuration(bundleSpaceGraph_->getBundle(), lastValid_.first);
                bundleSpaceGraph_->addConfiguration(xk);
                bundleSpaceGraph_->addBundleEdge(xLast, xk);
            }

            double length = std::accumulate(
                intermediateLengthsBasePath_.begin(), 
                intermediateLengthsBasePath_.begin()+(k-1), 0.0);

            length += lastValid_.second * 
              bundleSpaceGraph_->getBase()->distance(
                  basePath_.at(k-1), 
                  basePath_.at(k));

            static_cast<BundleSpaceGraph*>(bundleSpaceGraph_->getParent())->getGraphSampler()->setPathBiasStartSegment(length);
            break;
        }
    }
    bundleSpaceGraph_->getBundle()->freeStates(section);
    return found;
}

const unsigned int PATH_SECTION_TREE_MAX_DEPTH = 3;
const unsigned int PATH_SECTION_TREE_MAX_BRANCHING = 10;

bool ompl::geometric::BundleSpacePathRestriction::checkSectionRecursiveRepair(
    Configuration* const xStart,
    Configuration* const xGoal,
    const std::vector<base::State*> basePath,
    bool interpolateL1,
    unsigned int depth,
    double startLength)
{
    bundleSpaceGraph_->projectFiber(xStart->state, xFiberStart_);
    bundleSpaceGraph_->projectFiber(xGoal->state, xFiberGoal_);

    std::vector<base::State*> section;
    if(interpolateL1)
    {
        section = 
          interpolateSectionL1(xFiberStart_, xFiberGoal_, basePath);
    }else{
        section = 
          interpolateSectionL1_FiberFirst(xFiberStart_, xFiberGoal_, basePath);
    }

    Configuration *xLast = xStart;

    for(uint k = 1; k < section.size(); k++)
    {
        if(bundleSpaceGraph_->getBundle()->checkMotion(
              section.at(k-1), section.at(k), lastValid_))
        {
            if(k < section.size()-1)
            {
                xLast = addFeasibleSegment(xLast, section.at(k));
            }else{
                addFeasibleGoalSegment(xLast, xGoal);
                OMPL_DEBUG("Found feasible path section (%d edges added)", k);
                return true;
            }
        }else
        {
            //############################################################################
            //Get Last valid
            //############################################################################
            Configuration *xLastValid{nullptr};
            if(lastValid_.second > 0)
            {
                //add last valid into the bundle graph
                xLastValid = new Configuration(bundleSpaceGraph_->getBundle(), lastValid_.first);
                bundleSpaceGraph_->addConfiguration(xLastValid);
                bundleSpaceGraph_->addBundleEdge(xLast, xLastValid);
                xLast = xLastValid;
            }else{
                xLastValid = xLast;
            }

            //############################################################################
            //Get length until last Valid
            //############################################################################
            double locationOnBasePath = 0.0;
            for(uint j = 1; j < k; j++){
                double dj = bundleSpaceGraph_->getBase()->distance(basePath.at(j-1), basePath.at(j));
                locationOnBasePath += dj;
            }

            if(k < basePath.size())
            {
                locationOnBasePath += lastValid_.second 
                  * bundleSpaceGraph_->getBase()->distance(basePath.at(k-1), basePath.at(k));
            }

            static_cast<BundleSpaceGraph*>(bundleSpaceGraph_->getParent())
              ->getGraphSampler()->setPathBiasStartSegment(locationOnBasePath + startLength);

            if(depth + 1 >= PATH_SECTION_TREE_MAX_DEPTH) return false;

            //############################################################################
            //Side step randomly and interpolate from there towards goal
            //############################################################################
            unsigned int lastCtr = 
              bundleSpaceGraph_->interpolateAlongBasePath(basePath, locationOnBasePath, xBaseTmp_);

            std::vector<base::State*> basePathSegment = 
            {basePath.begin() + lastCtr, basePath.end()}; 
            basePathSegment.insert(basePathSegment.begin(), xBaseTmp_);

            //TODO: one side step should be the goal itself (or continuing
            //constant from last state (not last valid) )
            for(uint j = 0; j < PATH_SECTION_TREE_MAX_BRANCHING; j++)
            {
                //#############################################################
                //find feasible sample in current fiber
                //#############################################################
                if(!sideStepAlongFiber(xBaseTmp_, xBundleTmp_)) continue;

                //#############################################################
                //check that we can connect new sample with last states
                //#############################################################
                if(bundleSpaceGraph_->getBundle()->checkMotion(xLastValid->state, xBundleTmp_))
                {
                    Configuration *xSideStep = new Configuration(bundleSpaceGraph_->getBundle(), xBundleTmp_);
                    bundleSpaceGraph_->addConfiguration(xSideStep);
                    bundleSpaceGraph_->addBundleEdge(xLastValid, xSideStep);

                    //#########################################################
                    //side step was successful. 
                    //Now interpolate from there to goal
                    //#########################################################

                    bool feasibleSection = 
                      checkSectionRecursiveRepair(
                          xSideStep, 
                          xGoal, 
                          basePathSegment, 
                          !interpolateL1,
                          depth+1, 
                          locationOnBasePath);

                    if(feasibleSection)
                    {
                        return true;
                    }
                }
            }

            break;
        }
    }
    return false;
}

