#include "solver_interface.hpp"

/**

@file gurobi_interface.hpp

Gurobi backend

*/

struct _GRBmodel;
typedef struct _GRBmodel GRBmodel;

namespace sco {

class GurobiModel : public Model {
public:
  GRBmodel* m_model;
  vector<Var> m_vars;
  vector<Cnt> m_cnts;

  GurobiModel();

  Var addVar(const string& name);
  Var addVar(const string& name, double lower, double upper);

  Cnt addEqCnt(const AffExpr&, const string& name);
  Cnt addIneqCnt(const AffExpr&, const string& name);
  Cnt addIneqCnt(const QuadExpr&, const string& name);

  void removeVars(const vector<Var>&);
  void removeCnts(const vector<Cnt>&);

  void update();
  void setVarBounds(const std::vector<Var>&, const std::vector<double>& lower, const std::vector<double>& upper);
  vector<double> getVarValues(const vector<Var>&) const;

  CvxOptStatus optimize();
  /** Don't use this function, because it adds constraints that aren't tracked  */
  CvxOptStatus optimizeFeasRelax();

  void setObjective(const AffExpr&);
  void setObjective(const QuadExpr&);
  void writeToFile(const string& fname);

  VarVector getVars() const;

  ~GurobiModel();

};


}

