# targets

set(sources
    foo.cpp
)

add_library(libapp STATIC ${sources})
add_executable(app main.cpp)
target_link_libraries(app libapp)
