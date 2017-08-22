#include <cstdlib>
#include <iostream>
#include <string>

#include "ompl/trajopt/logging.h"
using namespace std;

namespace util {

LogLevel gLogLevel;

int LoggingInit() {
  const char* VALID_THRESH_VALUES = "FATAL ERROR WARN INFO DEBUG TRACE";

  char* lvlc = getenv("TRAJOPT_LOG_THRESH");
  string lvlstr;
  if (lvlc == NULL) {
    printf("You can set logging level with TRAJOPT_LOG_THRESH. Valid values: %s. Defaulting to INFO\n", VALID_THRESH_VALUES);
    lvlstr = "INFO";
  }
  else lvlstr = string(lvlc);
  if (lvlstr == "FATAL") gLogLevel = LevelFatal;
  else if (lvlstr == "ERROR") gLogLevel =  LevelError;
  else if (lvlstr == "WARN") gLogLevel = LevelWarn;
  else if (lvlstr == "INFO") gLogLevel = LevelInfo;
  else if (lvlstr == "DEBUG") gLogLevel = LevelDebug;
  else if (lvlstr == "TRACE") gLogLevel = LevelTrace;
  else {
    printf("Invalid value for environment variable TRAJOPT_LOG_THRESH: %s\n", lvlstr.c_str());
    printf("Valid values: %s\n", VALID_THRESH_VALUES);
    abort();
  }
  return 1;
}
int this_is_a_hack_but_rhs_executes_on_library_load = LoggingInit();

}
