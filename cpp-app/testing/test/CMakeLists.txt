# dummy targets

add_custom_target(ut)

add_custom_target(ut-run
    COMMAND ${CMAKE_CTEST_COMMAND}
    DEPENDS ut
)

# ut utilities

include_directories(
    ${GTEST_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/src
)

add_library(gmain STATIC main.cpp)

function(ut_test name)
    add_executable(${name} ${ARGN})
    target_link_libraries(${name} gtest gmain)
    add_test(NAME ${name} COMMAND ${name})
    add_dependencies(ut ${name})
endfunction()

# ut executables

ut_test(test_foo foo/test_foo.cpp)
target_link_libraries(test_foo libapp)

ut_test(test_bar bar/test_bar.cpp)
target_link_libraries(test_bar libapp)
