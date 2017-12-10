# targets

set(sources
    app.cpp
)

add_library(libapp STATIC ${sources})
set_target_properties(libapp PROPERTIES PREFIX "")

add_executable(app main.cpp)
target_link_libraries(app libapp)
