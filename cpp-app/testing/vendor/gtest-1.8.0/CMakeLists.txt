find_package(Threads)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

add_library(gtest STATIC gtest-all.cc)
target_link_libraries(gtest ${CMAKE_THREAD_LIBS_INIT})
