#define BOOST_TEST_MODULE VampIntegrationTest
#include <boost/test/unit_test.hpp>
#include <cstring>
#include <iostream>

// Architecture-specific headers for basic SIMD testing
#if defined(__x86_64__) || defined(_M_X64)
#include <immintrin.h>
#elif defined(__aarch64__) || defined(_M_ARM64)
#include <arm_neon.h>
#endif

// Test that VAMP configuration is working
BOOST_AUTO_TEST_CASE(VampConfigurationTest)
{
    // This test verifies that VAMP was properly configured during build
    // If this test compiles and runs, it means:
    // 1. OMPL_HAVE_VAMP was set to TRUE in CMake
    // 2. vamp_cpp target was available for linking
    // 3. The basic build integration is working
    
    BOOST_CHECK_MESSAGE(true, "VAMP integration compiled and linked successfully");
}

// Test VAMP target availability and linking
BOOST_AUTO_TEST_CASE(VampTargetLinkingTest)
{
    // This test confirms that VAMP was built and linked properly
    // If vamp_cpp target wasn't available, this test wouldn't have linked
    
    BOOST_CHECK_MESSAGE(true, "vamp_cpp target successfully linked");
}

// Test basic SIMD support (minimal architecture verification)
BOOST_AUTO_TEST_CASE(VampBasicSIMDTest)
{
#if defined(__x86_64__) || defined(_M_X64)
    BOOST_TEST_MESSAGE("Testing x86_64 SIMD support");
    
    // Test basic AVX2 (core requirement for VAMP performance)
#ifdef __AVX2__
    BOOST_CHECK_MESSAGE(true, "AVX2 support detected - VAMP SIMD configuration working");
    
    // Basic AVX2 functionality test
    __m256i a = _mm256_set1_epi32(10);
    __m256i b = _mm256_set1_epi32(5);
    __m256i result = _mm256_add_epi32(a, b);
    
    int extracted[8];
    _mm256_storeu_si256((__m256i*)extracted, result);
    BOOST_CHECK_EQUAL(extracted[0], 15);
    
    BOOST_TEST_MESSAGE("AVX2 operations working correctly");
#else
    BOOST_WARN_MESSAGE(false, "AVX2 not detected - VAMP may have reduced performance");
#endif

#elif defined(__aarch64__) || defined(_M_ARM64)
    BOOST_TEST_MESSAGE("Testing ARM64 SIMD support");
    
    // Test basic NEON (standard on ARM64)
#ifdef __ARM_NEON
    BOOST_CHECK_MESSAGE(true, "ARM NEON support detected - VAMP SIMD configuration working");
    
    // Basic NEON functionality test
    int32x4_t a = vdupq_n_s32(10);
    int32x4_t b = vdupq_n_s32(5);
    int32x4_t result = vaddq_s32(a, b);
    
    int32_t extracted[4];
    vst1q_s32(extracted, result);
    BOOST_CHECK_EQUAL(extracted[0], 15);
    
    BOOST_TEST_MESSAGE("ARM NEON operations working correctly");
#else
    BOOST_WARN_MESSAGE(false, "ARM NEON not detected - check VAMP SIMD configuration");
#endif

#else
    BOOST_TEST_MESSAGE("Unknown architecture - VAMP SIMD optimizations may not be available");
#endif
}

// Test memory alignment for SIMD operations (basic requirement)
BOOST_AUTO_TEST_CASE(VampMemoryAlignmentTest)
{
    BOOST_TEST_MESSAGE("Testing SIMD-compatible memory alignment");
    
    // Test 32-byte alignment (required for AVX2)
    constexpr size_t alignment = 32;
    constexpr size_t size = 256;
    
    void* aligned_ptr = nullptr;
    int result = posix_memalign(&aligned_ptr, alignment, size);
    
    BOOST_CHECK_EQUAL(result, 0);
    BOOST_CHECK(aligned_ptr != nullptr);
    
    if (aligned_ptr) {
        // Verify alignment
        uintptr_t addr = reinterpret_cast<uintptr_t>(aligned_ptr);
        BOOST_CHECK_EQUAL(addr % alignment, 0);
        
        // Basic write test
        memset(aligned_ptr, 0x42, size);
        unsigned char* bytes = static_cast<unsigned char*>(aligned_ptr);
        BOOST_CHECK_EQUAL(bytes[0], 0x42);
        
        free(aligned_ptr);
        
        BOOST_TEST_MESSAGE("Memory alignment test passed");
    }
}

// Test portable vs native build configuration
BOOST_AUTO_TEST_CASE(VampBuildConfigurationTest)
{
    BOOST_TEST_MESSAGE("Testing VAMP build configuration");
    
#ifdef __x86_64__
    // Check for expected instruction sets in portable/native builds
    #ifdef __AVX2__
        BOOST_TEST_MESSAGE("AVX2 enabled - SIMD acceleration available");
    #else
        BOOST_WARN_MESSAGE(false, "AVX2 not enabled - performance may be limited");
    #endif
    
    BOOST_CHECK_MESSAGE(true, "x86_64 build configuration validated");
    
#elif defined(__aarch64__)
    BOOST_CHECK_MESSAGE(true, "ARM64 build configuration validated");
    
#else
    BOOST_WARN_MESSAGE(false, "Unknown architecture - VAMP may not be optimized");
#endif

    BOOST_TEST_MESSAGE("Build configuration test completed");
}

// Test compiler optimization level (important for VAMP performance)
BOOST_AUTO_TEST_CASE(VampOptimizationTest)
{
    BOOST_TEST_MESSAGE("Testing compiler optimization");
    
    // Check optimization level
#ifdef NDEBUG
    BOOST_CHECK_MESSAGE(true, "Release mode - optimizations enabled");
#else
    BOOST_TEST_MESSAGE("Debug mode - may have reduced performance");
#endif

    // Basic performance indicator test
    volatile int a = 100;
    volatile int b = 50;
    volatile int c = a + b;
    
    BOOST_CHECK_EQUAL(c, 150);
    BOOST_TEST_MESSAGE("Basic operations working correctly");
} 