#ifndef OMPL_BASE_GENERAL_
#define OMPL_BASE_GENERAL_

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdarg>
#include <cassert>
#include <climits>
#include <cmath>

#include <random_utils/random_utils.h>
#include <time_utils/time_utils.h>

#define ForwardClassDeclaration(A) class A; typedef A * A##_t; typedef const A * A##_const_t
#define ForwardStructDeclaration(A) struct A; typedef A * A##_t; typedef const A * A##_const_t

#endif
