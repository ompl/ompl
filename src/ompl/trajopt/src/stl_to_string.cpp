#include "ompl/trajopt/stl_to_string.hpp"
#include <sstream>
#include <vector>
using namespace std;

namespace {
template<class T>
std::string Str_impl(const vector<T>& x) {
  stringstream ss;
  ss << "(";
  if (x.size() > 0) ss << x[0];
  for(size_t i = 1; i < x.size(); ++i)
    ss << ", " << x[i];
  ss << ")";
  return ss.str();
}

}


namespace util {

std::string Str(const vector<double>& x) {
  return Str_impl(x);
}
std::string Str(const vector<float>& x) {
  return Str_impl(x);
}
std::string Str(const vector<int>& x) {
  return Str_impl(x);
}




}
