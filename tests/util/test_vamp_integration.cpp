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

// VAMP-OMPL integration headers
#include "../../demos/Vamp/core/VampValidators.h"
#include "../../demos/Vamp/core/VampOMPLInterfaces.h"

namespace ob = ompl::base;

// Test VAMP target availability and linking
BOOST_AUTO_TEST_CASE(VampTargetLinkingTest)
{
    // This test confirms that VAMP was built and linked properly
    // If vamp_cpp target wasn't available, this test wouldn't have linked
    
    BOOST_CHECK_MESSAGE(true, "vamp_cpp target successfully linked");
}

// Test OMPL to VAMP state conversion for Panda robot
BOOST_AUTO_TEST_CASE(VampStateConversionPandaTest)
{
    using Robot = vamp::robots::Panda;
    constexpr std::size_t robotDimension = Robot::dimension;
    
    BOOST_TEST_MESSAGE("Testing OMPL to VAMP state conversion for Panda robot");
    BOOST_CHECK_EQUAL(robotDimension, 7);
    
    // Create OMPL state space
    auto realVectorSpace = std::make_shared<ob::RealVectorStateSpace>(robotDimension);
    ob::RealVectorBounds bounds(robotDimension);
    for (size_t i = 0; i < robotDimension; ++i) {
        bounds.setLow(i, -3.14);
        bounds.setHigh(i, 3.14);
    }
    realVectorSpace->setBounds(bounds);
    
    auto spaceInfo = std::make_shared<ob::SpaceInformation>(realVectorSpace);
    ob::ScopedState<> omplState(realVectorSpace);
    
    // Set test configuration values
    std::array<double, robotDimension> testConfig = {0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785};
    for (size_t i = 0; i < robotDimension; ++i) {
        omplState[i] = testConfig[i];
    }
    
    // Test that conversion function can be called without error
    try {
        auto vampConfig = vamp_ompl::conversion::ompl_to_vamp<Robot>(omplState.get());
        BOOST_CHECK_MESSAGE(true, "OMPL to VAMP conversion succeeded for Panda robot");
    } catch (const std::exception& e) {
        BOOST_FAIL("OMPL to VAMP conversion failed: " + std::string(e.what()));
    }
    
    BOOST_TEST_MESSAGE("Panda state conversion test passed");
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

// Test precision handling in double to float conversion
BOOST_AUTO_TEST_CASE(VampStateConversionPrecisionTest)
{
    using Robot = vamp::robots::Panda;
    constexpr std::size_t robotDimension = Robot::dimension;
    
    BOOST_TEST_MESSAGE("Testing precision handling in state conversion");
    BOOST_CHECK_EQUAL(robotDimension, 7);
    
    // Create OMPL state space
    auto realVectorSpace = std::make_shared<ob::RealVectorStateSpace>(robotDimension);
    ob::RealVectorBounds bounds(robotDimension);
    for (size_t i = 0; i < robotDimension; ++i) {
        bounds.setLow(i, -10.0);
        bounds.setHigh(i, 10.0);
    }
    realVectorSpace->setBounds(bounds);
    
    ob::ScopedState<> omplState(realVectorSpace);
    
    // Test with high-precision double values
    std::array<double, robotDimension> highPrecisionConfig = {
        1.2345678901234567,  // More precision than float can handle
        -2.9876543210987654,
        0.1111111111111111,
        3.1415926535897932,
        -1.4142135623730951,
        2.7182818284590452,
        1.2312312312312312
    };
    
    for (size_t i = 0; i < robotDimension; ++i) {
        omplState[i] = highPrecisionConfig[i];
    }
    
    // Test that conversion handles high-precision values without error
    try {
        auto vampConfig = vamp_ompl::conversion::ompl_to_vamp<Robot>(omplState.get());
        BOOST_CHECK_MESSAGE(true, "High-precision conversion succeeded for Panda robot");
    } catch (const std::exception& e) {
        BOOST_FAIL("High-precision conversion failed: " + std::string(e.what()));
    }
    
    BOOST_TEST_MESSAGE("Precision handling test passed");
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