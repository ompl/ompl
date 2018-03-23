#pragma once
#include <boost/shared_ptr.hpp>
#include <iosfwd>
#include <limits>
#include <string>
#include <vector>

#include "ompl/trajopt/sco_fwd.h"
/**
@brief Interface to convex solvers

  This is based on Gurobi's nice c++ API (though the SCO Gurobi backend uses the Gurobi c api).
  However, our intention is to allow for different solvers to be used as backends.
 */

namespace sco
{
    using std::string;
    using std::vector;
    using std::ostream;

    typedef vector<double> DblVec;
    typedef vector<int> IntVec;

    enum ConstraintType
    {
        EQ,
        INEQ
    };

    enum CvxOptStatus
    {
        CVX_SOLVED,
        CVX_INFEASIBLE,
        CVX_FAILED
    };

    typedef vector<Var> VarVector;
    typedef vector<AffExpr> AffExprVector;
    typedef vector<QuadExpr> QuadExprVector;

    /** @brief Convex optimization problem

    Gotchas:
    - after adding a variable, need to call update() before doing anything else with that variable

     */
    class Model
    {
    public:
        virtual Var addVar(const string &name) = 0;
        virtual Var addVar(const string &name, double lb, double ub);

        virtual Cnt addEqCnt(const AffExpr &, const string &name) = 0;     // expr == 0
        virtual Cnt addIneqCnt(const AffExpr &, const string &name) = 0;   // expr <= 0
        virtual Cnt addIneqCnt(const QuadExpr &, const string &name) = 0;  // expr <= 0

        virtual void removeVar(const Var &var);
        virtual void removeCnt(const Cnt &cnt);
        virtual void removeVars(const VarVector &vars) = 0;
        virtual void removeCnts(const vector<Cnt> &cnts) = 0;

        virtual void update() = 0;  // call after adding/deleting stuff
        virtual void setVarBounds(const Var &var, double lower, double upper);
        virtual void setVarBounds(const VarVector &vars, const vector<double> &lower, const vector<double> &upper) = 0;
        virtual double getVarValue(const Var &var) const;
        virtual vector<double> getVarValues(const VarVector &vars) const = 0;
        virtual CvxOptStatus optimize() = 0;

        virtual void setObjective(const AffExpr &) = 0;
        virtual void setObjective(const QuadExpr &) = 0;

        /** \brief Adds the given experssion to the currently defined objective function. */
        // virtual void addToObjective(const AffExpr&)=0;

        /** \brief Adds the given experssion to the currently defined objective function. */
        // virtual void addToObjective(const QuadExpr&)=0;

        virtual void writeToFile(const string &fname) = 0;

        virtual VarVector getVars() const = 0;

        virtual ~Model()
        {
        }
    };

    /**
     * \brief A variable representation in an optimization problem:
     * index: the number of variable this is. The optimization problem state is commonly
     *        stored as a vector of doubles, this is used as the index into that vector.
     * name: arbitrary human understandable name for the variable.
     * creator: used to determine that the same optimization instance made this variable.
     */
    struct VarRep
    {
        VarRep(int _index, const string &_name, void *_creator)
          : index(_index), name(_name), removed(false), creator(_creator)
        {
        }
        int index;
        string name;
        bool removed;
        void *creator;
    };

    // A variable in an optimization problem (actual info is in VarRep).
    struct Var
    {
        VarRep *var_rep;
        Var() : var_rep(NULL)
        {
        }
        Var(VarRep *var_rep) : var_rep(var_rep)
        {
        }
        Var(const Var &other) : var_rep(other.var_rep)
        {
        }
        double value(const double *x) const
        {
            return x[var_rep->index];
        }
        double value(const vector<double> &x) const
        {
            assert(var_rep->index < (int)x.size());
            return x[var_rep->index];
        }
    };

    struct CntRep
    {
        CntRep(int _index, void *_creator) : index(_index), removed(false), creator(_creator)
        {
        }
        int index;
        bool removed;
        void *creator;
        ConstraintType type;
        string expr;  // todo placeholder
    };

    // Constraint
    struct Cnt
    {
        CntRep *cnt_rep;
        Cnt() : cnt_rep(NULL)
        {
        }
        Cnt(CntRep *cnt_rep) : cnt_rep(cnt_rep)
        {
        }
        Cnt(const Cnt &other) : cnt_rep(other.cnt_rep)
        {
        }
    };

    /** \brief An affine expression of the form
     *  f(x) = constant + coeffs[0] * x[vars[0].var_rep->index] + ...
     */
    struct AffExpr
    {  // affine expression
        double constant;
        vector<double> coeffs;
        vector<Var> vars;
        AffExpr() : constant(0)
        {
        }
        explicit AffExpr(double a) : constant(a)
        {
        }
        explicit AffExpr(const Var &v) : constant(0), coeffs(1, 1), vars(1, v)
        {
        }
        AffExpr(const AffExpr &other) : constant(other.constant), coeffs(other.coeffs), vars(other.vars)
        {
        }
        size_t size() const
        {
            return coeffs.size();
        }
        double value(const double *x) const;
        double value(const vector<double> &x) const;
    };

    /** \brief A quadratic expression of the form
        f(x) = affexpr(x) + coeffs[0] * x[vars1[0].var_rep->index] * x[vars2[0].var_rep->index] + ...
    */
    struct QuadExpr
    {
        AffExpr affexpr;
        vector<double> coeffs;
        vector<Var> vars1;
        vector<Var> vars2;
        QuadExpr()
        {
        }
        explicit QuadExpr(double a) : affexpr(a)
        {
        }
        explicit QuadExpr(const Var &v) : affexpr(v)
        {
        }
        explicit QuadExpr(const AffExpr &aff) : affexpr(aff)
        {
        }
        size_t size() const
        {
            return coeffs.size();
        }
        double value(const double *x) const;
        double value(const vector<double> &x) const;
    };

    ostream &operator<<(ostream &, const Var &);
    ostream &operator<<(ostream &, const Cnt &);
    ostream &operator<<(ostream &, const AffExpr &);
    ostream &operator<<(ostream &, const QuadExpr &);

    ModelPtr createModel();
}
