#pragma once

/* Vendored build configuration. Upstream generates this from config.h.in via
 * CMake; all three options default to OFF, so every symbol stays undefined:
 *
 *   QPMAD_ENABLE_TRACING    debug tracing to std::cout
 *   QPMAD_USE_HOUSEHOLDER   Householder instead of Givens factorization updates
 *   QPMAD_PEDANTIC_LICENSE  defines EIGEN_MPL2_ONLY
 *
 * Define any of them on the compile line to enable it.
 */
