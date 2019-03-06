Todo
---------------
- post process .cpp.o.time files
- modularize timing all targets

Usage
---------------

Building:
```
git clone git@github.com:aurzenligl/study.git
cd study/cpp-compile-time/2
mkdir build
cd build
cmake ..
make  # generates compilation time statistics
```

Generated files:
```
$ find -name '*time'
./CMakeFiles/measured.dir/src/std/variant.cpp.o.time
./CMakeFiles/measured.dir/src/std/memory.cpp.o.time
./CMakeFiles/measured.dir/src/boost/shared_ptr.hpp.cpp.o.time
./CMakeFiles/measured.dir/src/boost/variant.hpp.cpp.o.time
$ find -name '*fstats'
./CMakeFiles/measured.dir/src/std/memory.cpp.o.fstats
./CMakeFiles/measured.dir/src/std/variant.cpp.o.fstats
./CMakeFiles/measured.dir/src/boost/variant.hpp.cpp.o.fstats
./CMakeFiles/measured.dir/src/boost/shared_ptr.hpp.cpp.o.fstats
```

Post processing:
```
$ ../print-includes.sh | grep size
std/variant -> /usr/include/x86_64-linux-gnu/bits/wordsize.h
std/memory -> /usr/include/x86_64-linux-gnu/bits/typesizes.h
std/memory -> /usr/include/x86_64-linux-gnu/bits/wordsize.h
boost/variant.hpp -> /usr/include/boost/mpl/O1_size.hpp
boost/variant.hpp -> /usr/include/boost/mpl/O1_size_fwd.hpp
boost/variant.hpp -> /usr/include/boost/mpl/aux_/O1_size_impl.hpp
boost/variant.hpp -> /usr/include/boost/mpl/aux_/has_size.hpp
boost/variant.hpp -> /usr/include/boost/mpl/aux_/size_impl.hpp
boost/variant.hpp -> /usr/include/boost/mpl/list/aux_/O1_size.hpp
boost/variant.hpp -> /usr/include/boost/mpl/list/aux_/size.hpp
boost/variant.hpp -> /usr/include/boost/mpl/size.hpp
boost/variant.hpp -> /usr/include/boost/mpl/size_fwd.hpp
boost/variant.hpp -> /usr/include/boost/mpl/size_t.hpp
boost/variant.hpp -> /usr/include/boost/mpl/size_t_fwd.hpp
boost/variant.hpp -> /usr/include/boost/mpl/sizeof.hpp
boost/variant.hpp -> /usr/include/boost/preprocessor/array/size.hpp
boost/variant.hpp -> /usr/include/boost/preprocessor/seq/size.hpp
boost/variant.hpp -> /usr/include/boost/preprocessor/variadic/size.hpp
boost/variant.hpp -> /usr/include/x86_64-linux-gnu/bits/typesizes.h
boost/variant.hpp -> /usr/include/x86_64-linux-gnu/bits/wordsize.h
boost/shared_ptr.hpp -> /usr/include/x86_64-linux-gnu/bits/typesizes.h
boost/shared_ptr.hpp -> /usr/include/x86_64-linux-gnu/bits/wordsize.h
$ ../print-time-table.sh
0.815919  0.474471  ../src/boost/variant.hpp.cpp
0.558471  0.397222  ../src/boost/shared_ptr.hpp.cpp
0.336594  0.161207  ../src/std/memory.cpp
0.077228  0.07977   ../src/std/variant.cpp
```
