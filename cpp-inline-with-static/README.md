Inline function with static variable can have separate instances of the variable in different shared-objects.

If we compile a.out with visibility default, we'll have one instance of static variable in program.
```
$ ./compile.sh && ./a.out
123
123
```

If we compile a.out with visibility hidden, we'll have two, one in shlib, one inlined into a.out.
```
$ HIDDEN=1 ./compile.sh && ./a.out
123
0
```
