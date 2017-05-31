#pragma once
#include "expr_ops.hpp"

namespace sco {

inline AffExpr operator+(const Var& x, double y) {
  return exprAdd(AffExpr(x), y);
}
inline AffExpr operator+(const AffExpr& x, double y) {
  return exprAdd(x, y);
}
inline QuadExpr operator+(const QuadExpr& x, double y) {
  return exprAdd(x, y);
}

inline AffExpr operator+(const Var& x, const Var& y) {
  return exprAdd(AffExpr(x), y);
}
inline AffExpr operator+(const AffExpr& x, const Var& y) {
  return exprAdd(x, y);
}
inline QuadExpr operator+(const QuadExpr& x, const Var& y) {
  return exprAdd(x, y);
}

inline AffExpr operator+(const Var& x, const AffExpr& y) {
  return exprAdd(AffExpr(x), y);
}
inline AffExpr operator+(const AffExpr& x, const AffExpr& y) {
  return exprAdd(x, y);
}
inline QuadExpr operator+(const QuadExpr& x, const AffExpr& y) {
  return exprAdd(x, y);
}

inline QuadExpr operator+(const Var& x, const QuadExpr& y) {
  return exprAdd(QuadExpr(x), y);
}
inline QuadExpr operator+(const AffExpr& x, const QuadExpr& y) {
  return exprAdd(QuadExpr(x), y);
}
inline QuadExpr operator+(const QuadExpr& x, const QuadExpr& y) {
  return exprAdd(x, y);
}




inline AffExpr operator-(const Var& x, double y) {
  return exprSub(AffExpr(x), y);
}
inline AffExpr operator-(const AffExpr& x, double y) {
  return exprSub(x, y);
}
inline QuadExpr operator-(const QuadExpr& x, double y) {
  return exprSub(x, y);
}

inline AffExpr operator-(const Var& x, const Var& y) {
  return exprSub(AffExpr(x), y);
}
inline AffExpr operator-(const AffExpr& x, const Var& y) {
  return exprSub(x, y);
}
inline QuadExpr operator-(const QuadExpr& x, const Var& y) {
  return exprSub(x, y);
}

inline AffExpr operator-(const Var& x, const AffExpr& y) {
  return exprSub(AffExpr(x), y);
}
inline AffExpr operator-(const AffExpr& x, const AffExpr& y) {
  return exprSub(x, y);
}
inline QuadExpr operator-(const QuadExpr& x, const AffExpr& y) {
  return exprSub(x, y);
}

inline QuadExpr operator-(const Var& x, const QuadExpr& y) {
  return exprSub(QuadExpr(x), y);
}
inline QuadExpr operator-(const AffExpr& x, const QuadExpr& y) {
  return exprSub(QuadExpr(x), y);
}
inline QuadExpr operator-(const QuadExpr& x, const QuadExpr& y) {
  return exprSub(x, y);
}

///////////////



inline AffExpr operator*(double a, const Var& b) {
  return exprMult(b, a);
}
inline AffExpr operator*(double a, const AffExpr& b) {
  return exprMult(b, a);
}
inline QuadExpr operator*(double a, const QuadExpr& b) {
  return exprMult(b, a);
}

inline AffExpr operator*(const Var& a, double b) {
  return exprMult(a, b);
}
inline AffExpr operator*(const AffExpr& a, double b) {
  return exprMult(a, b);
}
inline QuadExpr operator*(const QuadExpr& a, double b) {
  return exprMult(a, b);
}

inline AffExpr operator-(const Var& a) {
  return exprMult(a, -1);
}
inline AffExpr operator-(const AffExpr& a) {
  return exprMult(a, -1);
}

}
