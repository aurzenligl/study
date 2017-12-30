# dummy target to collect ut executables
add_custom_target(ut)

# dummy target to run ut
add_custom_target(ut-run
    COMMAND ${CMAKE_CTEST_COMMAND}
    DEPENDS ut
)

include_directories(
    ${GTEST_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/src
)

add_library(gmain STATIC main.cpp)

add_executable(test_foo foo/test_foo.cpp)
target_link_libraries(test_foo gtest gmain libapp)
add_test(NAME test_foo COMMAND test_foo)
add_dependencies(ut test_foo)

add_executable(test_bar bar/test_bar.cpp)
target_link_libraries(test_bar gtest gmain libapp)
add_test(NAME test_bar COMMAND test_bar)
add_dependencies(ut test_bar)
