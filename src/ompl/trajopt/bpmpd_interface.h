
#include "ompl/trajopt/solver_interface.h"
#include "ompl/trajopt/macros.h"

namespace sco {

class BPMPDModel : public Model {
public:
  vector<Var> m_vars;
  vector<Cnt> m_cnts;
  vector<AffExpr> m_cntExprs;
  vector<ConstraintType> m_cntTypes;
  vector<double> m_soln;
  vector<double> m_lbs, m_ubs;

  QuadExpr m_objective;

  int m_pipeIn, m_pipeOut, m_pid;

  BPMPDModel();
  virtual ~BPMPDModel();

  Var addVar(const string& name);
  Cnt addEqCnt(const AffExpr&, const string& name);
  Cnt addIneqCnt(const AffExpr&, const string& name);
  Cnt addIneqCnt(const QuadExpr&, const string& name);
  void removeVars(const VarVector& vars);
  void removeCnts(const vector<Cnt>& cnts);

  void update();
  void setVarBounds(const vector<Var>& vars, const vector<double>& lower, const vector<double>& upper);
  vector<double> getVarValues(const VarVector& vars) const;
  virtual CvxOptStatus optimize();
  virtual void setObjective(const AffExpr&);
  virtual void setObjective(const QuadExpr&);
  virtual void addToObjective(const AffExpr&);
  virtual void addToObjective(const QuadExpr&);
  virtual void writeToFile(const string& fname);
  virtual VarVector getVars() const;

};

}
