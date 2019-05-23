#include <ompl/geometric/planners/quotientspace/QRRT.h>
#include <ompl/tools/config/SelfConfig.h>
#include <boost/foreach.hpp>

using namespace og;
using namespace ob;
#define foreach BOOST_FOREACH

QRRT::QRRT(const ob::SpaceInformationPtr &si, Quotient *parent_ ):
  BaseT(si, parent_)
{
  setName("QRRT"+std::to_string(id));
  Planner::declareParam<double>("range", this, &QRRT::setRange, &QRRT::getRange, "0.:1.:10000.");
  Planner::declareParam<double>("goal_bias", this, &QRRT::setGoalBias, &QRRT::getGoalBias, "0.:.1:1.");
  q_random = new Configuration(Q1);
}

QRRT::~QRRT()
{
  DeleteConfiguration(q_random);
}

void QRRT::setGoalBias(double goalBias_)
{
  goalBias = goalBias_;
}
double QRRT::getGoalBias() const
{
  return goalBias;
}
void QRRT::setRange(double maxDistance_)
{
  maxDistance = maxDistance_;
}
double QRRT::getRange() const
{
  return maxDistance;
}

void QRRT::setup()
{
  BaseT::setup();
  ompl::tools::SelfConfig sc(Q1, getName());
  sc.configurePlannerRange(maxDistance);
  //const double base = 2;
  //const double normalizer = powf(base, level);
  //epsilon = 0.1/normalizer;

  goal = pdef_->getGoal().get();
}
void QRRT::clear()
{
  BaseT::clear();
}

bool QRRT::GetSolution(ob::PathPtr &solution)
{
  if(hasSolution){
    bool baset_sol = BaseT::GetSolution(solution);
    if(baset_sol){
      shortestPathVertices = shortestVertexPath_;
    }
    return baset_sol;
  }else{
    return false;
  }
}

void QRRT::Grow(double t){
  if(firstRun){
    Init();
    firstRun = false;
  }

  if(hasSolution){
    //No Goal Biasing if we already found a solution on this quotient space
    Sample(q_random->state);
  }else{
    double s = rng_.uniform01();
    if(s < goalBias){
      Q1->copyState(q_random->state, q_goal->state);
    }else{
      Sample(q_random->state);
    }
  }

  const Configuration *q_nearest = Nearest(q_random);
  double d = Q1->distance(q_nearest->state, q_random->state);
  if(d > maxDistance){
    Q1->getStateSpace()->interpolate(q_nearest->state, q_random->state, maxDistance / d, q_random->state);
  }

  totalNumberOfSamples++;
  if(Q1->checkMotion(q_nearest->state, q_random->state))
  {
    totalNumberOfFeasibleSamples++;
    Configuration *q_next = new Configuration(Q1, q_random->state);
    Vertex v_next = AddConfiguration(q_next);
    if(!hasSolution){
      //only add edge if no solution exists
      AddEdge(q_nearest->index, v_next);

      double dist = 0.0;
      bool satisfied = goal->isSatisfied(q_next->state, &dist);
      if(satisfied)
      {
        v_goal = AddConfiguration(q_goal);
        AddEdge(q_nearest->index, v_goal);
        //solution_path = GetPath(v_start, v_goal);
        hasSolution = true;
      }
    }
  }
}

double QRRT::GetImportance() const{
  //Should depend on
  // (1) level : The higher the level, the more importance
  // (2) total samples: the more we already sampled, the less important it
  // becomes
  // (3) has solution: if it already has a solution, we should explore less
  // (only when nothing happens on other levels)
  // (4) vertices: the more vertices we have, the less important (let other
  // levels also explore)
  //
  //exponentially more samples on level i. Should depend on ALL levels.
  // const double base = 2;
  // const double normalizer = powf(base, level);
  //double N = (double)GetNumberOfVertices()/normalizer;
  double N = (double)GetNumberOfVertices();
  return 1.0/(N+1);
}

//Make it faster by removing the valid check
bool QRRT::Sample(ob::State *q_random)
{
  if(parent == nullptr){
    Q1_sampler->sampleUniform(q_random);
  }else{
    if(X1_dimension>0)
    {
      X1_sampler->sampleUniform(s_X1_tmp);
      parent->SampleQuotient(s_Q0_tmp);
      MergeStates(s_Q0_tmp, s_X1_tmp, q_random);
    }else{
      parent->SampleQuotient(q_random);
    }
  }
  return true;
}

bool QRRT::SampleQuotient(ob::State *q_random_graph)
{
  const Vertex v = boost::random_vertex(G, rng_boost);
  Q1->getStateSpace()->copyState(q_random_graph, G[v]->state);
  return true;
}
