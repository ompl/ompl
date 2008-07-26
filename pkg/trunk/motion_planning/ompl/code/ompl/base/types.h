#ifndef OMPL_TYPES_
#define OMPL_TYPES_

// this is just for interoperability with visual studio, where the standard 
// integer types are not defined.

#ifndef _MSC_VER
  #include <stdint.h>
#else
  typedef          __int64  int64_t;
  typedef unsigned __int64 uint64_t;
  typedef          __int32  int32_t;
  typedef unsigned __int32 uint32_t;
  typedef          __int16  int16_t;
  typedef unsigned __int16 uint16_t;
  typedef          __int8    int8_t;
  typedef unsigned __int8   uint8_t;
#endif


#endif
