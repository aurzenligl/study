# targets

include_directories(
    ${GTEST_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/src
)

add_executable(test0
    main.cpp
    test_foo.cpp
)

target_link_libraries(test0 gtest libapp)
