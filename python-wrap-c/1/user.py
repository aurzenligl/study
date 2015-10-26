#!/usr/bin/env python

import hello
hello.hello('alabama')

from threading import Thread

t = [Thread(target=hello.waste_time) for _ in range(8)]
map(Thread.start, t)
map(Thread.join, t)
