find_package(Boost 1.58 QUIET COMPONENTS unit_test_framework)

# ${ARGV} is the number of arguments passed to the macro
# ${ARGV0}, ${ARGV1}, ... have actual values of the arguments passed
macro(add_ccd_test test_name)
  add_executable(${ARGV})
  target_link_libraries(${test_name}
    ccd
    ${Boost_PROGRAM_OPTIONS_LIBRARY}
    ${Boost_SERIALIZATION_LIBRARY}
    ${Boost_FILESYSTEM_LIBRARY}
    ${Boost_SYSTEM_LIBRARY}
    ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
  add_test(NAME ${test_name} COMMAND $<TARGET_FILE:${test_name}>)
endmacro(add_ccd_test)

# check if boost unit test library exists
if(Boost_UNIT_TEST_FRAMEWORK_LIBRARY)
    option(CCD_BUILD_TESTS "Build OMPL tests" ON)
else()
    option(CCD_BUILD_TESTS "Build OMPL tests" OFF)
endif()

# add_feature_info(OMPL_BUILD_TESTS "${OMPL_BUILD_TESTS}" "Whether to build the unit tests.")

if (CCD_BUILD_TESTS)
    # file(TO_NATIVE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/resources" TEST_RESOURCES_DIR)
    # string(REPLACE "\\" "\\\\" TEST_RESOURCES_DIR "${TEST_RESOURCES_DIR}")
    # add_definitions(-DTEST_RESOURCES_DIR=\"${TEST_RESOURCES_DIR}\")

    # Test code
    # add_ccd_test(test_robot_model robot_model/robot_model.cpp)
    # add_ccd_test(test_robot_state robot_state/robot_state.cpp)
    # add_ccd_test(test_math util/math.cpp)
    # add_ccd_test(test_calculator base/calculator.cpp)
    add_ccd_test(test_ccd base/ccd.cpp)
    
endif()


