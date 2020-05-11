# Add an unit tests executable that makes use of Google Test / Mock.
#
# add_unit_test(
#   BASENAME <basename>
#   SOURCES <source1> [<source2> ...]
#   TYPE <type>
#   [LIBS <lib-to-link1> [<lib-to-link2> ...]]
# )
#
# The executable target will be called 'ut_<basename>'.
#
# This function ensures that the tests
#  - links to the necessary GTest/GMock libraries
#  - links to the "tests-utils" library and all other given LIBS
#  - is built with all necessary include directories set up
#  - is compiled/linked with scripts flags if needed
#  - is called when the 'tests' target is run
#  - is installed into the proper directory
#
#
function(add_unit_test)
    set(_single_value_args BASENAME TYPE)
    set(_multi_value_args LIBS SOURCES)
    cmake_parse_arguments(_XT
        "${_options}" "${_single_value_args}" "${_multi_value_args}" ${ARGN}
    )

    if (_XT_UNPARSED_ARGUMENTS)
        message(FATAL_ERROR "Unrecognized params given ('${_XT_UNPARSED_ARGUMENTS}')!")
    endif()

    if (NOT _XT_SOURCES OR NOT _XT_BASENAME)
        message(FATAL_ERROR "Required param(s) missing")
    endif()

    list(APPEND _XT_LIBS ${GTEST_LIBS} )

    if (NOT _XT_TYPE)
        set(testname "ut_${_XT_BASENAME}")
    else()
        set(testname "${_XT_TYPE}_${_XT_BASENAME}")
    endif()

    set(TEST_MAIN Tests::TestMain)

    add_executable(${testname} ${_XT_SOURCES})

    target_link_libraries(${testname}
        PUBLIC ${_XT_LIBS}
        PUBLIC ${TEST_MAIN}
    )

    target_include_directories(${testname}
        SYSTEM PUBLIC ${GTEST_INCLUDE_DIRS})

    if (NOT _UT_WORKINGDIR)
        add_test(NAME ${testname} COMMAND ${testname})
    else()
        add_test(NAME ${testname} COMMAND ${testname} WORKING_DIRECTORY ${_XT_WORKINGDIR})
    endif()

    install(
        TARGETS ${testname}
        RUNTIME
        DESTINATION ${CMAKE_BINARY_DIR}}
    )

endfunction()
