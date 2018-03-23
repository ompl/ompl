#pragma once

#include "sco_fwd.h"
#include "solver_interface.h"

namespace sco
{
    ////// In-place operations ///////

    // multiplication
    inline void exprScale(AffExpr &v, double a)
    {
        v.constant *= a;
        for (size_t i = 0; i < v.coeffs.size(); ++i)
            v.coeffs[i] *= a;
    }
    inline void exprScale(QuadExpr &q, double a)
    {
        exprScale(q.affexpr, a);
        for (size_t i = 0; i < q.coeffs.size(); ++i)
            q.coeffs[i] *= a;
    }

    // addition
    inline void exprInc(AffExpr &a, double b)
    {
        a.constant += b;
    }
    inline void exprInc(AffExpr &a, const AffExpr &b)
    {
        a.constant += b.constant;
        a.coeffs.insert(a.coeffs.end(), b.coeffs.begin(), b.coeffs.end());
        a.vars.insert(a.vars.end(), b.vars.begin(), b.vars.end());
    }
    inline void exprInc(AffExpr &a, const Var &b)
    {
        exprInc(a, AffExpr(b));
    }
    inline void exprInc(QuadExpr &a, double b)
    {
        exprInc(a.affexpr, b);
    }
    inline void exprInc(QuadExpr &a, const Var &b)
    {
        exprInc(a.affexpr, AffExpr(b));
    }
    inline void exprInc(QuadExpr &a, const AffExpr &b)
    {
        exprInc(a.affexpr, b);
    }
    inline void exprInc(QuadExpr &a, const QuadExpr &b)
    {
        exprInc(a.affexpr, b.affexpr);
        a.coeffs.insert(a.coeffs.end(), b.coeffs.begin(), b.coeffs.end());
        a.vars1.insert(a.vars1.end(), b.vars1.begin(), b.vars1.end());
        a.vars2.insert(a.vars2.end(), b.vars2.begin(), b.vars2.end());
    }

    // subtraction
    inline void exprDec(AffExpr &a, double b)
    {
        a.constant -= b;
    }
    inline void exprDec(AffExpr &a, AffExpr b)
    {
        exprScale(b, -1);
        exprInc(a, b);
    }
    inline void exprDec(AffExpr &a, const Var &b)
    {
        exprDec(a, AffExpr(b));
    }
    inline void exprDec(QuadExpr &a, double b)
    {
        exprDec(a.affexpr, b);
    }
    inline void exprDec(QuadExpr &a, const Var &b)
    {
        exprDec(a.affexpr, b);
    }
    inline void exprDec(QuadExpr &a, const AffExpr &b)
    {
        exprDec(a.affexpr, b);
    }
    inline void exprDec(QuadExpr &a, QuadExpr b)
    {
        exprScale(b, -1);
        exprInc(a, b);
    }

    /////////////////////

    inline AffExpr exprMult(const Var &a, double b)
    {
        AffExpr c(a);
        exprScale(c, b);
        return c;
    }
    // multiplication
    inline AffExpr exprMult(AffExpr a, double b)
    {
        exprScale(a, b);
        return a;
    }
    inline QuadExpr exprMult(QuadExpr a, double b)
    {
        exprScale(a, b);
        return a;
    }

    inline AffExpr exprAdd(AffExpr a, double b)
    {
        exprInc(a, b);
        return a;
    }
    inline AffExpr exprAdd(AffExpr a, const Var &b)
    {
        exprInc(a, b);
        return a;
    }
    inline AffExpr exprAdd(AffExpr a, const AffExpr &b)
    {
        exprInc(a, b);
        return a;
    }
    inline QuadExpr exprAdd(QuadExpr a, double b)
    {
        exprInc(a, b);
        return a;
    }
    inline QuadExpr exprAdd(QuadExpr a, const Var &b)
    {
        exprInc(a, b);
        return a;
    }
    inline QuadExpr exprAdd(QuadExpr a, const AffExpr &b)
    {
        exprInc(a, b);
        return a;
    }
    inline QuadExpr exprAdd(QuadExpr a, const QuadExpr &b)
    {
        exprInc(a, b);
        return a;
    }

    inline AffExpr exprSub(AffExpr a, double b)
    {
        exprDec(a, b);
        return a;
    }
    inline AffExpr exprSub(AffExpr a, const Var &b)
    {
        exprDec(a, b);
        return a;
    }
    inline AffExpr exprSub(AffExpr a, const AffExpr &b)
    {
        exprDec(a, b);
        return a;
    }
    inline QuadExpr exprSub(QuadExpr a, double b)
    {
        exprDec(a, b);
        return a;
    }
    inline QuadExpr exprSub(QuadExpr a, const Var &b)
    {
        exprDec(a, b);
        return a;
    }
    inline QuadExpr exprSub(QuadExpr a, const AffExpr &b)
    {
        exprDec(a, b);
        return a;
    }
    inline QuadExpr exprSub(QuadExpr a, const QuadExpr &b)
    {
        exprDec(a, b);
        return a;
    }

    //////////////////////

    QuadExpr exprSquare(const Var &);
    QuadExpr exprSquare(const AffExpr &);

    AffExpr cleanupAff(const AffExpr &);
    QuadExpr cleanupQuad(const QuadExpr &);  // warning: might make it non-psd!
}
