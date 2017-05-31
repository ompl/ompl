#pragma once
#include <cstdio>

namespace util {
  
enum LogLevel {
  LevelFatal = 0,
  LevelError = 1,
  LevelWarn = 2,
  LevelInfo = 3,
  LevelDebug = 4,
  LevelTrace = 5
};

extern LogLevel gLogLevel;
inline LogLevel GetLogLevel() {return gLogLevel;}

  
#define FATAL_PREFIX "\x1b[31m[FATAL] "
#define ERROR_PREFIX "\x1b[31m[ERROR] "
#define WARN_PREFIX "\x1b[33m[WARN] "
#define INFO_PREFIX "[INFO] "
#define DEBUG_PREFIX "\x1b[32m[DEBUG] "
#define TRACE_PREFIX "\x1b[34m[TRACE] "
#define LOG_SUFFIX "\x1b[0m\n"
  
#define LOG_FATAL(msg, ...) if (util::GetLogLevel() >= util::LevelFatal) {printf(FATAL_PREFIX); printf(msg, ##__VA_ARGS__); printf(LOG_SUFFIX);}
#define LOG_ERROR(msg, ...) if (util::GetLogLevel() >= util::LevelError) {printf(ERROR_PREFIX); printf(msg, ##__VA_ARGS__); printf(LOG_SUFFIX);}
#define LOG_WARN(msg, ...) if (util::GetLogLevel() >= util::LevelWarn) {printf(WARN_PREFIX); printf(msg, ##__VA_ARGS__); printf(LOG_SUFFIX);}
#define LOG_INFO(msg, ...) if (util::GetLogLevel() >= util::LevelInfo) {printf(INFO_PREFIX); printf(msg, ##__VA_ARGS__); printf(LOG_SUFFIX);}
#define LOG_DEBUG(msg, ...) if (util::GetLogLevel() >= util::LevelDebug) {printf(DEBUG_PREFIX); printf(msg, ##__VA_ARGS__); printf(LOG_SUFFIX);}
#define LOG_TRACE(msg, ...) if (util::GetLogLevel() >= util::LevelTrace) {printf(TRACE_PREFIX); printf(msg, ##__VA_ARGS__); printf(LOG_SUFFIX);}

}
