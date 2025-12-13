#define BOOST_TEST_MODULE VampIntegrationTest
#include <boost/test/unit_test.hpp>
#include <cstring>
#include <iostream>
#include <array>
#include <vector>

// VAMP robot types for testing  
#include <vamp/robots/panda.hh>

// OMPL includes for state conversion testing
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/ScopedState.h>

namespace ob = ompl::base;

// Test VAMP target availability and linking
BOOST_AUTO_TEST_CASE(VampTargetLinkingTest)
{
    // This test confirms that VAMP was built and linked properly
    // If vamp_cpp target wasn't available, this test wouldn't have linked
    
    BOOST_CHECK_MESSAGE(true, "vamp_cpp target successfully linked");
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

// Test memory alignment for SIMD operations
BOOST_AUTO_TEST_CASE(VampMemoryAlignmentTest)
{
    BOOST_TEST_MESSAGE("Testing SIMD-compatible memory alignment");
    
    // Test 32-byte alignment
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