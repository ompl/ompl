add_executable(regression_test RegressionTest.cpp)
target_link_libraries(regression_test
    ompl
    ${Boost_DATE_TIME_LIBRARY})
