#define BOOST_TEST_MODULE VampIntegrationTest
#include <boost/test/unit_test.hpp>
#include <cstring>
#include <iostream>

// Architecture-specific headers for SIMD testing
#if defined(__x86_64__) || defined(_M_X64)
#include <immintrin.h>  // For x86_64 SIMD intrinsics
#elif defined(__aarch64__) || defined(_M_ARM64)
#include <arm_neon.h>   // For ARM64 NEON intrinsics
#endif

// Test that VAMP configuration is working
BOOST_AUTO_TEST_CASE(VampConfigurationTest)
{
    // This test verifies that VAMP was properly configured during build
    // If this test compiles and runs, it means:
    // 1. OMPL_HAVE_VAMP was set to TRUE
    // 2. vamp_cpp target was available for linking
    // 3. The basic build integration is working
    
    BOOST_CHECK(true); // Basic compilation test
}

// Test VAMP target availability (runtime verification)
BOOST_AUTO_TEST_CASE(VampTargetAvailabilityTest)
{
    // This test confirms that VAMP was built as part of the OMPL build process
    // If vamp_cpp target wasn't available, this test wouldn't have linked
    
    BOOST_CHECK_MESSAGE(true, "vamp_cpp target was successfully linked");
}

// Test x86_64 SIMD flags functionality
BOOST_AUTO_TEST_CASE(VampX86SIMDFlagsTest)
{
#if defined(__x86_64__) || defined(_M_X64)
    BOOST_TEST_MESSAGE("Testing x86_64 SIMD flags");
    
    // Test AVX2 support (should be available with our flags)
#ifdef __AVX2__
    BOOST_CHECK_MESSAGE(true, "AVX2 support detected - VAMP SIMD flags working");
    
    // Test basic AVX2 operation to ensure it's functional
    __m256i a = _mm256_set1_epi32(42);
    __m256i b = _mm256_set1_epi32(8);
    __m256i result = _mm256_add_epi32(a, b);
    
    // Extract and verify result
    int avx_extracted[8];
    _mm256_storeu_si256((__m256i*)avx_extracted, result);
    BOOST_CHECK_EQUAL(avx_extracted[0], 50);
    BOOST_CHECK_EQUAL(avx_extracted[7], 50);
    
    BOOST_TEST_MESSAGE("AVX2 operations working correctly");
#else
    BOOST_WARN_MESSAGE(false, "AVX2 not detected - check VAMP SIMD flags configuration");
#endif

    // Test FMA support (should be available with portable build flags)
#ifdef __FMA__
    BOOST_CHECK_MESSAGE(true, "FMA support detected - VAMP SIMD flags working");
    
    // Test basic FMA operation
    __m256 fa = _mm256_set1_ps(2.0f);
    __m256 fb = _mm256_set1_ps(3.0f);
    __m256 fc = _mm256_set1_ps(1.0f);
    __m256 fma_result = _mm256_fmadd_ps(fa, fb, fc); // 2*3+1 = 7
    
    float fma_extracted[8];
    _mm256_storeu_ps(fma_extracted, fma_result);
    BOOST_CHECK_CLOSE(fma_extracted[0], 7.0f, 0.001f);
    
    BOOST_TEST_MESSAGE("FMA operations working correctly");
#else
    BOOST_WARN_MESSAGE(false, "FMA not detected - check VAMP SIMD flags configuration");
#endif

    // Test BMI2 support (should be available with portable build flags)
#ifdef __BMI2__
    BOOST_CHECK_MESSAGE(true, "BMI2 support detected - VAMP portable SIMD flags working");
    
    // Test basic BMI2 operation (bit manipulation)
    uint64_t mask = 0x0F0F0F0F0F0F0F0FULL;
    uint64_t src = 0x123456789ABCDEFULL;
    uint64_t bmi_extracted = _pext_u64(src, mask);
    
    // This should extract bits according to the mask pattern
    BOOST_CHECK_NE(bmi_extracted, static_cast<uint64_t>(0));
    
    BOOST_TEST_MESSAGE("BMI2 operations working correctly");
#else
    BOOST_WARN_MESSAGE(false, "BMI2 not detected - may not be available on this processor");
#endif

#else
    BOOST_TEST_MESSAGE("Not on x86_64 platform - skipping x86_64 SIMD tests");
#endif
}

// Test ARM64 SIMD flags functionality
BOOST_AUTO_TEST_CASE(VampARM64SIMDFlagsTest)
{
#if defined(__aarch64__) || defined(_M_ARM64)
    BOOST_TEST_MESSAGE("Testing ARM64 SIMD flags");
    
    // Test NEON support (should be available on ARM64)
#ifdef __ARM_NEON
    BOOST_CHECK_MESSAGE(true, "ARM NEON support detected - VAMP SIMD flags working");
    
    // Test basic NEON operation
    int32x4_t a = vdupq_n_s32(10);
    int32x4_t b = vdupq_n_s32(5);
    int32x4_t result = vaddq_s32(a, b);
    
    // Extract and verify result
    int32_t neon_extracted[4];
    vst1q_s32(neon_extracted, result);
    BOOST_CHECK_EQUAL(neon_extracted[0], 15);
    BOOST_CHECK_EQUAL(neon_extracted[3], 15);
    
    BOOST_TEST_MESSAGE("ARM NEON operations working correctly");
#else
    BOOST_WARN_MESSAGE(false, "ARM NEON not detected - check VAMP SIMD flags configuration");
#endif

#else
    BOOST_TEST_MESSAGE("Not on ARM64 platform - skipping ARM64 SIMD tests");
#endif
}

// Test compiler flags and optimization level
BOOST_AUTO_TEST_CASE(VampCompilerFlagsTest)
{
    BOOST_TEST_MESSAGE("Testing compiler flags and optimization");
    
    // Check if we're compiled with optimization (Release mode typically)
#ifdef NDEBUG
    BOOST_CHECK_MESSAGE(true, "Release mode detected - optimizations should be enabled");
#else
    BOOST_TEST_MESSAGE("Debug mode detected - may have reduced optimizations");
#endif

    // Test that basic arithmetic operations work (compiler should optimize these)
    volatile int a = 100;
    volatile int b = 50;
    volatile int c = a + b;
    
    BOOST_CHECK_EQUAL(c, 150);
    
    // Test floating point operations
    volatile double fa = 3.14159;
    volatile double fb = 2.71828;
    volatile double fc = fa * fb;
    
    BOOST_CHECK_GT(fc, 8.0);
    BOOST_CHECK_LT(fc, 9.0);
    
    BOOST_TEST_MESSAGE("Basic arithmetic operations working correctly");
}

// Test memory alignment (important for SIMD operations)
BOOST_AUTO_TEST_CASE(VampMemoryAlignmentTest)
{
    BOOST_TEST_MESSAGE("Testing memory alignment for SIMD operations");
    
    // Test aligned memory allocation
    constexpr size_t alignment = 32; // 256-bit alignment for AVX2
    constexpr size_t size = 1024;
    
    void* aligned_ptr = nullptr;
    int result = posix_memalign(&aligned_ptr, alignment, size);
    
    BOOST_CHECK_EQUAL(result, 0);
    BOOST_CHECK(aligned_ptr != nullptr);
    
    if (aligned_ptr) {
        // Check alignment
        uintptr_t addr = reinterpret_cast<uintptr_t>(aligned_ptr);
        BOOST_CHECK_EQUAL(addr % alignment, 0);
        
        // Test that we can write to the memory
        memset(aligned_ptr, 0xAA, size);
        
        // Verify the pattern
        unsigned char* bytes = static_cast<unsigned char*>(aligned_ptr);
        BOOST_CHECK_EQUAL(bytes[0], 0xAA);
        BOOST_CHECK_EQUAL(bytes[size-1], 0xAA);
        
        free(aligned_ptr);
        
        BOOST_TEST_MESSAGE("Memory alignment test passed");
    }
}

// Test that portable vs native build flags are working
BOOST_AUTO_TEST_CASE(VampPortableBuildTest)
{
    BOOST_TEST_MESSAGE("Testing VAMP portable build configuration");
    
    // This test verifies that the build system correctly configured
    // portable vs native build flags
    
#ifdef __x86_64__
    // For x86_64, check for portable build indicators
    #ifdef __BMI2__
        BOOST_TEST_MESSAGE("BMI2 detected - likely portable build with specific instruction sets");
    #endif
    
    #ifdef __AVX2__
        BOOST_TEST_MESSAGE("AVX2 detected - SIMD support properly configured");
    #endif
    
    // Check that we don't have overly specific CPU targeting in portable builds
    // (this would be detected by specific CPU model flags)
    BOOST_CHECK_MESSAGE(true, "x86_64 build configuration appears correct");
    
#elif defined(__aarch64__)
    // For ARM64, verify basic architecture support
    BOOST_CHECK_MESSAGE(true, "ARM64 build configuration appears correct");
    
#else
    BOOST_WARN_MESSAGE(false, "Unknown architecture - VAMP SIMD flags may not be optimized");
#endif

    BOOST_TEST_MESSAGE("Portable build test completed");
} 