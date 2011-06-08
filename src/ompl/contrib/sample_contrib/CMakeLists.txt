# The default should be "ON" for contributions. Since this sample
# contribution doesn't do anything useful it's disabled by default.
option(OMPL_CONTRIB_SAMPLECONTRIB "Enable SampleContrib" OFF)

if(OMPL_CONTRIB_SAMPLECONTRIB)
    file(GLOB_RECURSE OMPL_SAMPLECONTRIB_SOURCE_CODE src/*.cpp)
    set(OMPL_SOURCE_CODE ${OMPL_SOURCE_CODE} ${OMPL_SAMPLECONTRIB_SOURCE_CODE} PARENT_SCOPE)

    if(OMPL_BUILD_TESTS)
        add_ompl_test(test_samplecontrib tests/test_sample_contrib.cpp)
    endif()
endif()
