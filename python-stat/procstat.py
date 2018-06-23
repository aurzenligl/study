#!/usr/bin/env python

import os
import sys

desc = [
    # file    no   name                     tag         description
    ('stat',   1, 'comm',                  'id!',      'executable filename'),  # interesting!
    ('stat',   0, 'pid',                   'id',       'process id'),
    ('stat',   3, 'ppid',                  'id',       'parent process id'),
    ('stat',   4, 'pgrp',                  'id',       'process group id'),
    ('stat',   5, 'session',               'id',       'process session id'),
    ('stat',   6, 'tty_nr',                'id',       'controlling terminal'),
    ('stat',   7, 'tpgid',                 'id',       'foreground terminal process group id'),

    ('stat',   9, 'minflt',                'counter!', 'minor page faults'),  # interesting!
    ('stat',  11, 'majflt',                'counter!', 'major page faults'),  # interesting!
    ('stat',  13, 'utime',                 'counter!', 'user-mode cpu ticks used'),  # interesting!
    ('stat',  14, 'stime',                 'counter!', 'kernel-mode cpu ticks used'),  # insteresting!
    ('stat',  41, 'delayacct_blkio_ticks', 'counter!', 'block IO delays in ticks'),  # interesting!
    ('stat',  10, 'cminflt',               'counter',  'waited-for-children minor page faults'),
    ('stat',  12, 'cmajflt',               'counter',  'waited-for-children major page faults'),
    ('stat',  15, 'cutime',                'counter',  'waited-for-children user-mode cpu ticks used'),
    ('stat',  16, 'cstime',                'counter',  'waited-for-children kernel-mode cpu ticks used'),
    ('stat',  42, 'guest_time',            'counter',  'virtual cpu for guest OS ticks used'),
    ('stat',  43, 'cguest_time',           'counter',  'guest time of children in OS ticks'),

    ('stat',  19, 'num_threads',           'status!',  'number of threads'),  # interesting!
    ('stat',   2, 'state',                 'status',   'process state'),
    ('stat',   8, 'flags',                 'status',   'process flags'),
    ('stat',  38, 'processor',             'status',   'cpu number last running on'),
    ('stat',  21, 'starttime',             'status',   'ticks after boot when process started'),
    ('stat',  37, 'exit_signal',           'status',   'signal to be sent to parent on death'),
    ('stat',  51, 'exit_code',             'status',   'exit status as returned by waitpid'),

    ('statm',  1, 'resident',              'memory',   'resident set size'),  # interesting!
    ('statm',  2, 'shared',                'memory',   'shared resident pages'),  # interesting!

    ('stat',  17, 'priority',              'setting',  'raw nice value or RT priority if negative'),
    ('stat',  18, 'nice',                  'setting',  'nice value'),
    ('stat',  39, 'rt_priority',           'setting',  'RT priority'),
    ('stat',  40, 'policy',                'setting',  'scheduling policy'),

    # unhelpful ones

    ('stat',  23, 'rss',                   'memory',  'resident set size in pages'),
    ('stat',  22, 'vsize',                 'memory',   'virtual memory size in bytes'),
    ('statm',  0, 'size',                  'memory',   'total program size in pages'),
    ('statm',  3, 'text',                  'memory',   'text size'),
    ('statm',  5, 'data',                  'memory',   'data and stack size'),

    ('stat',  28, 'kstkesp',               'address',  'current stack pointer'),
    ('stat',  29, 'kstkeip',               'address',  'current instruction pointer'),
    ('stat',  34, 'wchan',                 'address',  'address in kernel where process sleeps'),
    ('stat',  27, 'startstack',            'address',  'start of stack memory region'),
    ('stat',  25, 'startcode',             'address',  'start of executable memory region'),
    ('stat',  26, 'endcode',               'address',  'end of executable memory region'),
    ('stat',  44, 'start_data',            'address',  'start of data memory region'),
    ('stat',  45, 'end_data',              'address',  'end of data memory region'),
    ('stat',  46, 'start_brk',             'address',  'start of brk expansion memory region'),
    ('stat',  47, 'arg_start',             'address',  'start of argv cmdline args memory region'),
    ('stat',  48, 'arg_end',               'address',  'end of argv cmdline args memory region'),
    ('stat',  49, 'env_start',             'address',  'start of program environment memory region'),
    ('stat',  50, 'env_end',               'address',  'end of program environment memory region'),

    ('stat',  20, 'itrealvalue',           'obsolete', 'hard coded to 0'),
    ('stat',  24, 'rsslim',                'obsolete', 'size of ram in bytes'),
    ('stat',  30, 'signal',                'obsolete', 'bitmap of pending signals as dec'),
    ('stat',  31, 'blocked',               'obsolete', 'bitmap of blocked signals as dec'),
    ('stat',  32, 'sigignore',             'obsolete', 'bitmap of ignored signals as dec'),
    ('stat',  33, 'sigcatch',              'obsolete', 'bitmap of caught signals as dec'),
    ('stat',  35, 'nswap',                 'obsolete', 'number of swapped pages'),
    ('stat',  36, 'cnswap',                'obsolete', 'cumulative nswap with children'),
    ('statm',  4, 'lib',                   'obsolete', 'library size'),
    ('statm',  6, 'dt',                    'obsolete', 'dirty pages'),
]

def read(name):
    return open(sys.argv[1] + '/' + name).read().split()

def separator():
    class C:
        tag = None
    def f(tag):
        if C.tag is not None and C.tag != tag:
            print '-' * 120
        C.tag = tag
    return f

data = {
    'stat': read('stat'),
    'statm': read('statm')
}
sep = separator()
for fname, idx, name, tag, desc in desc:
    sep(tag)
    print '%-5s %3s %-22s %-10s %-48s %s' % (fname, idx, name, tag, desc, data[fname][idx])
