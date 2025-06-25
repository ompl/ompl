# VAMP SIMD Requirements and Build Options

## Overview

VAMP (Vector-Accelerated Motion Planning) uses SIMD (Single Instruction, Multiple Data) instructions to accelerate collision detection and motion validation. This document outlines the SIMD requirements and build options for creating portable and optimized binaries.

## OMPL Build Options

When building OMPL with VAMP support, you can control the SIMD optimization level using these CMake options:

### Basic VAMP Integration
```bash
# Enable VAMP support (required for SIMD options to take effect)
cmake -DOMPL_BUILD_VAMP=ON ..
```

### SIMD Optimization Control
```bash
# Native build - best performance for end users (default)
cmake -DOMPL_BUILD_VAMP=ON ..

# Portable build - wider compatibility for package maintainers
cmake -DOMPL_BUILD_VAMP=ON -DVAMP_PORTABLE_BUILD=ON ..
```

**Note**: The `VAMP_PORTABLE_BUILD` option only affects builds when `OMPL_BUILD_VAMP=ON`. If VAMP is not enabled, this option has no effect.

## SIMD Architecture Support

### x86_64 Architecture
- **SIMD Type**: AVX2 (`__m256` and `__m256i`)
- **Vector Width**: 8 floats or 8 integers per operation
- **Minimum Requirements**: AVX2 + BMI2 (Intel Haswell 2013+, AMD Excavator 2015+)
- **Alignment**: 32-byte aligned memory access
- **Additional Instructions**: BMI2 for bit manipulation operations used in collision detection

### ARM64/aarch64 Architecture  
- **SIMD Type**: NEON (`float32x4_t` and `int32x4_t`)
- **Vector Width**: 4 floats or 4 integers per operation
- **Minimum Requirement**: ARMv8-A (all ARM64 processors)
- **Alignment**: 16-byte aligned memory access

## Build Modes

VAMP supports two build modes to balance performance and portability:

### 1. Native Build (Default: `VAMP_PORTABLE_BUILD=OFF`)
**Best performance, optimized for build machine**

**x86_64**: Uses `-march=native -mavx2`
**ARM64**: Uses `-mcpu=native -mtune=native`

**Recommended for**: End users building from source, development environments, single-machine deployments.

### 2. Portable Build (`VAMP_PORTABLE_BUILD=ON`)
**Wider compatibility, slightly slower performance**

**x86_64**: Uses `-march=x86-64 -mavx2 -mfma -mbmi2`
**ARM64**: Uses `-march=armv8-a`

**Recommended for**: Package maintainers (Debian, Ubuntu, vcpkg), distribution builds, multi-platform deployments.

**Note**: The portable build requires BMI2 support (Intel Haswell 2013+, AMD Excavator 2015+) for x86_64 systems due to VAMP's collision detection algorithms.

## CPU Compatibility

### x86_64 Compatibility
| CPU Generation | Release Year | AVX2 Support | BMI2 Support | Compatible |
|----------------|--------------|--------------|--------------|------------|
| Intel Haswell  | 2013         | ✅ Yes       | ✅ Yes       | ✅ Yes     |
| Intel Broadwell| 2014         | ✅ Yes       | ✅ Yes       | ✅ Yes     |
| Intel Skylake  | 2015         | ✅ Yes       | ✅ Yes       | ✅ Yes     |
| Intel Kaby Lake| 2016         | ✅ Yes       | ✅ Yes       | ✅ Yes     |
| Intel Coffee Lake| 2017      | ✅ Yes       | ✅ Yes       | ✅ Yes     |
| Intel Ice Lake | 2019         | ✅ Yes       | ✅ Yes       | ✅ Yes     |
| Intel Tiger Lake| 2020        | ✅ Yes       | ✅ Yes       | ✅ Yes     |
| Intel Alder Lake| 2021        | ✅ Yes       | ✅ Yes       | ✅ Yes     |
| AMD Excavator  | 2015         | ✅ Yes       | ✅ Yes       | ✅ Yes     |
| AMD Zen        | 2017         | ✅ Yes       | ✅ Yes       | ✅ Yes     |
| AMD Zen+       | 2018         | ✅ Yes       | ✅ Yes       | ✅ Yes     |
| AMD Zen 2      | 2019         | ✅ Yes       | ✅ Yes       | ✅ Yes     |
| AMD Zen 3      | 2020         | ✅ Yes       | ✅ Yes       | ✅ Yes     |
| AMD Zen 4      | 2022         | ✅ Yes       | ✅ Yes       | ✅ Yes     |

**Not Compatible**: Intel Sandy Bridge, Ivy Bridge, AMD Bulldozer, Piledriver (pre-2013) - these lack AVX2 and/or BMI2 support

### ARM64 Compatibility
All ARM64 processors (ARMv8-A and later) support NEON SIMD instructions:
- Apple M1/M2/M3 series
- Qualcomm Snapdragon 8xx series  
- Samsung Exynos 9xxx series
- NVIDIA Tegra X1/X2
- Amazon Graviton series
- All ARM64 server processors

## Performance Impact

### Native vs Portable Builds

**Estimated Performance Differences:**
- **x86_64**: 10-30% slower with portable builds
- **ARM64**: 5-15% slower with portable builds

**Factors affecting performance:**
- CPU-specific optimizations (cache sizes, instruction latencies)
- Compiler auto-vectorization improvements


## Runtime Detection

VAMP automatically detects the SIMD capabilities at compile time and uses the appropriate implementation. No runtime SIMD detection is needed since the code is compiled for the target architecture.

## Troubleshooting

### Verification Commands

```bash
# Check CPU capabilities
cat /proc/cpuinfo | grep -E "(flags|model name)"

# Check build flags
grep -r "VAMP_ARCH" build/CMakeCache.txt

# Test SIMD functionality
./build/demos/demo_VampDemo  # Should run without errors
```

### Build Option Verification

```bash
# Verify VAMP build options in CMake cache
grep -E "(OMPL_BUILD_VAMP|VAMP_PORTABLE_BUILD)" build/CMakeCache.txt

# Check which SIMD flags were applied
grep -E "(VAMP.*portable|VAMP.*native)" build/CMakeCache.txt

# Check for BMI2 support in CPU flags
cat /proc/cpuinfo | grep -o "bmi2" | head -1
```

### Common Issues

1. **BMI2 instruction errors in portable build**: This indicates your CPU doesn't support BMI2. VAMP requires both AVX2 and BMI2 support:
   ```bash
   # Check if your CPU supports BMI2
   cat /proc/cpuinfo | grep -o "bmi2" | head -1
   
   # If no output, your CPU is not compatible with VAMP
   ```

2. **VAMP_PORTABLE_BUILD not taking effect**: Make sure you're using the `-D` flag:
   ```bash
   # Correct
   cmake -DOMPL_BUILD_VAMP=ON -DVAMP_PORTABLE_BUILD=ON ..
   
   # Incorrect (missing -D)
   cmake -DOMPL_BUILD_VAMP=ON VAMP_PORTABLE_BUILD=ON ..
   ```

3. **VAMP not building**: Ensure the VAMP submodule is initialized:
   ```bash
   git submodule update --init --recursive
   ```

4. **Python not found**: VAMP requires Python 3.8+ with development headers:
   ```bash
   # Ubuntu/Debian
   sudo apt-get install python3-dev python3-pip
   
   # macOS
   brew install python3
   ```

## References

- [Intel AVX2 Documentation](https://software.intel.com/sites/landingpage/IntrinsicsGuide/#techs=AVX2)
- [ARM NEON Documentation](https://developer.arm.com/architectures/instruction-sets/simd-isas/neon)
- [GCC x86 Options](https://gcc.gnu.org/onlinedocs/gcc/x86-Options.html)
- [GCC ARM Options](https://gcc.gnu.org/onlinedocs/gcc/ARM-Options.html) 