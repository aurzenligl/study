Todo
---------------
- check if time and bc exist (cmake time)
- allow any include to point to public module header

Usage
---------------

Building:
```
git clone git@github.com:aurzenligl/study.git
cd study/cpp-compile-time/2
mkdir build
cd build
cmake -GNinja -DSTATS=1 ..
ninja  # generates compilation time statistics
```

Generated files:
```
build $ ll ./CMakeFiles/measured.dir/src/foo
total 512
drwxrwxr-x 2 aurzenligl aurzenligl   4096 Mar  7 01:56 ./
drwxrwxr-x 5 aurzenligl aurzenligl   4096 Mar  7 01:56 ../
lrwxrwxrwx 1 aurzenligl aurzenligl     30 Mar  7 01:56 bar.cpp -> ../../../../../src/foo/bar.cpp
-rw-rw-r-- 1 aurzenligl aurzenligl   1706 Mar  7 01:56 bar.cpp.cmd
-rw-rw-r-- 1 aurzenligl aurzenligl  74368 Mar  7 01:56 bar.cpp.o
-rw-rw-r-- 1 aurzenligl aurzenligl      5 Mar  7 01:56 bar.cpp.o.time
-rw-rw-r-- 1 aurzenligl aurzenligl     30 Mar  7 01:56 bar.header.cpp
-rw-rw-r-- 1 aurzenligl aurzenligl   1096 Mar  7 01:56 bar.header.cpp.o
-rw-rw-r-- 1 aurzenligl aurzenligl    126 Mar  7 01:56 bar.header.cpp.o.d
-rw-rw-r-- 1 aurzenligl aurzenligl      5 Mar  7 01:56 bar.header.cpp.o.time
-rw-rw-r-- 1 aurzenligl aurzenligl     64 Mar  7 01:56 bar.includes.cpp
-rw-rw-r-- 1 aurzenligl aurzenligl   1104 Mar  7 01:56 bar.includes.cpp.o
-rw-rw-r-- 1 aurzenligl aurzenligl    156 Mar  7 01:56 bar.includes.cpp.o.d
-rw-rw-r-- 1 aurzenligl aurzenligl      5 Mar  7 01:56 bar.includes.cpp.o.time
lrwxrwxrwx 1 aurzenligl aurzenligl     30 Mar  7 01:56 foo.cpp -> ../../../../../src/foo/foo.cpp
-rw-rw-r-- 1 aurzenligl aurzenligl   1706 Mar  7 01:56 foo.cpp.cmd
-rw-rw-r-- 1 aurzenligl aurzenligl 126800 Mar  7 01:56 foo.cpp.o
-rw-rw-r-- 1 aurzenligl aurzenligl      5 Mar  7 01:56 foo.cpp.o.time
-rw-rw-r-- 1 aurzenligl aurzenligl     30 Mar  7 01:56 foo.header.cpp
-rw-rw-r-- 1 aurzenligl aurzenligl   3712 Mar  7 01:56 foo.header.cpp.o
-rw-rw-r-- 1 aurzenligl aurzenligl    302 Mar  7 01:56 foo.header.cpp.o.d
-rw-rw-r-- 1 aurzenligl aurzenligl      5 Mar  7 01:56 foo.header.cpp.o.time
-rw-rw-r-- 1 aurzenligl aurzenligl    103 Mar  7 01:56 foo.includes.cpp
-rw-rw-r-- 1 aurzenligl aurzenligl  43256 Mar  7 01:56 foo.includes.cpp.o
-rw-rw-r-- 1 aurzenligl aurzenligl   2288 Mar  7 01:56 foo.includes.cpp.o.d
-rw-rw-r-- 1 aurzenligl aurzenligl      5 Mar  7 01:56 foo.includes.cpp.o.time
```

Post processing. Columns:
1. cpp compilation time
2. includes compilation time
3. cpp's header compilation time
4. [2] to [1] ratio in percents
5. [3] to [1] ratio in percents
6. cpp path relative to cwd
```
aurzenligl@dell7510 ~/projects/aeolus/share/study/cpp-compile-time/2/build $ ../print-times.sh 
6.86  6.71  1.96  97   28   ../src/foo/foo.cpp
0.71  0.72  0.79  101  111  ../src/boost/variant.hpp.cpp
0.39  0.43  0.49  110  125  ../src/boost/shared_ptr.hpp.cpp
0.16  0.16  0.00  100  0    ../src/std/memory.cpp
0.15  0.11  0.09  73   60   ../src/foo/bar.cpp
```
