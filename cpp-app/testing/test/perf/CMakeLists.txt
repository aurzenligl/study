# perf proxies

add_custom_target(perf-build)

add_custom_target(perf
    COMMAND ${CMAKE_CTEST_COMMAND} -L "perf" --verbose
    DEPENDS perf-build
)

# perf utilities

include_directories(
    ${CMAKE_SOURCE_DIR}/src
)

function(perf_test name)
    add_executable(${name} ${ARGN})
    target_link_libraries(${name} benchmark::benchmark)
    add_test(NAME ${name} COMMAND ${name})
    set_tests_properties(${name} PROPERTIES LABELS perf)
    add_dependencies(perf-build ${name})
endfunction()

# perf executables

perf_test(perf_foo foo/perf_foo.cpp)
target_link_libraries(perf_foo libapp)

perf_test(perf_bar bar/perf_bar.cpp)
target_link_libraries(perf_bar libapp)
