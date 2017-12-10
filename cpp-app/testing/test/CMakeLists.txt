# targets

include_directories(
    ${GTEST_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/src
)

add_library(gmain STATIC main.cpp)

add_executable(test_foo
    foo/test_foo.cpp
)
target_link_libraries(test_foo gtest gmain libapp)

add_executable(test_bar
    bar/test_bar.cpp
)
target_link_libraries(test_bar gtest gmain libapp)
