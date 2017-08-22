#include <unistd.h>
#include <iostream>
#include <errno.h>
#include <string.h>
#include "ompl/trajopt/bpmpd_io.h"
#include "ompl/trajopt/stl_to_string.h"
using namespace bpmpd_io;
using namespace std;

extern "C" {
  extern void bpmpd(int *, int *, int *, int *, int *, int *, int *,
         double *, int *, int *, double *, double *, double *, double *,
         double *, double *, double *, int *, double *, int *, double *, int *);
}

int main(int argc, char** argv) {
  string working_dir = BPMPD_WORKING_DIR;
  int err = chdir(working_dir.c_str());
  if (err != 0) {
    cerr << "error going to BPMPD working dir\n";
    cerr << strerror(err) << endl;
    abort();
  }
  // int counter=0;
  while (true) {
    bpmpd_input bi;
    ser(STDIN_FILENO, bi, DESER);

    int memsiz = 0;
    double BIG = 1e30;
    bpmpd_output bo;
    bo.primal.resize(bi.m+bi.n);
    bo.dual.resize(bi.m+bi.n);
    bo.status.resize(bi.m+bi.n);

#define DBG(expr) //cerr << #expr << ": " << CSTR(expr) << std::endl
      DBG(bi.m);
      DBG(bi.n);
      DBG(bi.nz);
      DBG(bi.qn);
      DBG(bi.qnz);
      DBG(bi.acolcnt);
      DBG(bi.acolidx);
      DBG(bi.acolnzs);
      DBG(bi.qcolcnt);
      DBG(bi.qcolidx);
      DBG(bi.qcolnzs);
      DBG(bi.rhs);
      DBG(bi.obj);
      DBG(bi.lbound);
      DBG(bi.ubound);

    // boost::posix_time::ptime start = boost::posix_time::microsec_clock::local_time( );
    bpmpd(&bi.m, &bi.n, &bi.nz, &bi.qn, &bi.qnz, bi.acolcnt.data(), bi.acolidx.data(), bi.acolnzs.data(), bi.qcolcnt.data(), bi.qcolidx.data(), bi.qcolnzs.data(),
        bi.rhs.data(), bi.obj.data(), bi.lbound.data(), bi.ubound.data(),
        bo.primal.data(), bo.dual.data(), bo.status.data(), &BIG, &bo.code, &bo.opt, &memsiz);
    // boost::posix_time::ptime end = boost::posix_time::microsec_clock::local_time( );
    // std::cout << "ELAPSED" << end-start << std::endl;

    ser(STDOUT_FILENO, bo, SER);
  }

}
