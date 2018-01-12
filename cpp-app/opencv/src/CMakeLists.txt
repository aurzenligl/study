add_executable(cppapp
    main.cpp
    process.cpp
    algo.cpp
)
target_link_libraries(cppapp
    ${OpenCV_LIBS}
)
