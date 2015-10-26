#!/bin/bash

gcc -shared hello.c hello_fn.c -fPIC -o hello.so
