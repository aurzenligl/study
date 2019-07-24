Problem reproduction for:
https://github.com/ros/catkin/pull/1017

Run [build.sh](build.sh) and see how `foo_msgs/Foo.h` header is included from `ws0` rather
than from `ws1` as expected.