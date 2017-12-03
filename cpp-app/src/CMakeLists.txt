add_executable(cppapp
    main.cpp
    opts.cpp
)
target_link_libraries(cppapp
    ${Boost_LIBRARIES}
    ${OpenCV_LIBS}
)
