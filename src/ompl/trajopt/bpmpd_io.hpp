#include <vector>
#include <string>
using std::string;
using std::vector;
#include <iostream>
#include <assert.h>
#include <stdlib.h>
#include <unistd.h>
using namespace std;

namespace bpmpd_io {

enum SerMode {
  DESER,
  SER
};

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"

template <typename T>
void ser(int fp, T& x, SerMode mode) {
  
  switch (mode) {
    case SER: {
      T xcopy=x;
      int n = write(fp, &xcopy, sizeof(T));
      assert (n == sizeof(T));
      break;
    }
    case DESER: {
      int n = read(fp, &x, sizeof(T));
      assert (n == sizeof(T));
      break;
    }
  }
}

template <typename T>
void ser(int fp,  vector<T>& x,  SerMode mode) {
  int size = x.size();
  ser(fp, size, mode);
  switch (mode) {
    case SER: {
      int n = write(fp, x.data(), sizeof(T)*size);
      assert (n == sizeof(T)*size);      
      break;
    }
    case DESER: {
      x.resize(size);
      int n = read(fp, x.data(), sizeof(T)*size);    
      assert (n == sizeof(T)*size);            
      break;
    }
  }
  
}

struct bpmpd_input
{
    int m, n, nz, qn, qnz;
    vector<int> acolcnt, acolidx;
    vector<double> acolnzs;
    vector<int> qcolcnt, qcolidx;
    vector<double> qcolnzs;
    vector<double> rhs, obj, lbound, ubound;

  bpmpd_input() {}
  bpmpd_input(int m, int n, int nz, int qn, int qnz, 
              const vector<int>& acolcnt, const vector<int>& acolidx, const vector<double>& acolnzs, 
              const vector<int>& qcolcnt, const vector<int>& qcolidx, const vector<double>& qcolnzs, 
              const vector<double>& rhs, const vector<double>& obj, const vector<double>& lbound, const vector<double>& ubound) :
    m(m), n(n), nz(nz), qn(qn), qnz(qnz), acolcnt(acolcnt), acolidx(acolidx), acolnzs(acolnzs), qcolcnt(qcolcnt), qcolidx(qcolidx), qcolnzs(qcolnzs),
    rhs(rhs), obj(obj), lbound(lbound), ubound(ubound) {}
};

const char EXIT_CHAR = 123;
const char CHECK_CHAR = 111;

void ser(int fp, bpmpd_input & bi, SerMode mode) {
  
  char scorrect='z', s=(mode==SER) ? scorrect : 0;
  ser(fp, s, mode);
  if (s == EXIT_CHAR) {
    exit(0);
  }
  
  
    ser(fp,bi.m,mode);
    ser(fp,bi.n,mode);
    ser(fp,bi.nz,mode);
    ser(fp,bi.qn,mode);
    ser(fp,bi.qnz,mode);
    ser(fp,bi.acolcnt,mode);
    ser(fp,bi.acolidx,mode);
    ser(fp,bi.acolnzs,mode);
    ser(fp,bi.qcolcnt,mode);
    ser(fp,bi.qcolidx,mode);
    ser(fp,bi.qcolnzs,mode);
    ser(fp,bi.rhs,mode);
    ser(fp,bi.obj,mode);
    ser(fp,bi.lbound,mode);
    ser(fp,bi.ubound,mode);      
}




struct bpmpd_output
{

  vector<double> primal, dual;
  vector<int> status;
  int code;    
  double opt;
  bpmpd_output() {}    
  bpmpd_output(const vector<double>& primal, const vector<double>& dual, const vector<int>& status, int code, double opt)  :
    primal(primal), dual(dual), status(status), code(code), opt(opt) {}
};

void ser(int fp, bpmpd_output & bo, SerMode mode)
{
  char scorrect=CHECK_CHAR, s=(mode==SER) ? scorrect : 0;
  ser(fp, s, mode);
  if (s == EXIT_CHAR) {
    exit(0);
  }
  ser(fp, bo.primal, mode);
  ser(fp, bo.dual, mode);
  ser(fp, bo.status, mode);
  ser(fp, bo.code, mode);
  ser(fp, bo.opt, mode);


}


#pragma GCC diagnostic pop


}