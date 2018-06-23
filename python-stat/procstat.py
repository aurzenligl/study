#!/usr/bin/env python

import os
import sys

desc = [
    # file     scope  no   name                     tag         description
    ('stat',  'thr',   1, 'comm',                  'id!',      'executable filename'),  # interesting!
    ('stat',  'thr',   0, 'pid',                   'id!',      'process id'),  # interesting!

    ('stat',  'thr',   9, 'minflt',                'counter!', 'minor page faults'),  # interesting!
    ('stat',  'thr',  11, 'majflt',                'counter!', 'major page faults'),  # interesting!
    ('stat',  'thr',  13, 'utime',                 'counter!', 'user-mode cpu ticks used'),  # interesting!
    ('stat',  'thr',  14, 'stime',                 'counter!', 'kernel-mode cpu ticks used'),  # insteresting!
    ('stat',  'thr',  41, 'delayacct_blkio_ticks', 'counter!', 'block IO delays in ticks'),  # interesting!

    ('statm', 'proc',  1, 'resident',              'memory!',  'resident set size'),  # interesting!
    ('statm', 'proc',  2, 'shared',                'memory!',  'shared resident pages'),  # interesting!

    ('stat',  'proc',  3, 'ppid',                  'id',       'parent process id'),
    ('stat',  'proc',  4, 'pgrp',                  'id',       'process group id'),
    ('stat',  'proc',  5, 'session',               'id',       'process session id'),
    ('stat',  'proc',  6, 'tty_nr',                'id',       'controlling terminal'),
    ('stat',  'proc',  7, 'tpgid',                 'id',       'foreground terminal process group id'),

    ('stat',  'proc', 10, 'cminflt',               'counter',  'waited-for-children minor page faults'),
    ('stat',  'proc', 12, 'cmajflt',               'counter',  'waited-for-children major page faults'),
    ('stat',  'proc', 15, 'cutime',                'counter',  'waited-for-children user-mode cpu ticks used'),
    ('stat',  'proc', 16, 'cstime',                'counter',  'waited-for-children kernel-mode cpu ticks used'),
    ('stat',  'proc', 42, 'guest_time',            'counter',  'virtual cpu for guest OS ticks used'),
    ('stat',  'proc', 43, 'cguest_time',           'counter',  'guest time of children in OS ticks'),

    ('stat',  'thr',  2,  'state',                 'status',   'process state'),
    ('stat',  'thr',  8,  'flags',                 'status',   'process flags'),
    ('stat',  'thr',  38, 'processor',             'status',   'cpu number last running on'),
    ('stat',  'thr',  21, 'starttime',             'status',   'ticks after boot when process started'),
    ('stat',  'proc', 19, 'num_threads',           'status',   'number of threads'),  # interesting!
    ('stat',  'proc', 37, 'exit_signal',           'status',   'signal to be sent to parent on death'),
    ('stat',  'proc', 51, 'exit_code',             'status',   'exit status as returned by waitpid'),

    ('stat',  'proc', 22, 'vsize',                 'memory',   'virtual memory size in bytes'),
    ('stat',  'proc', 23, 'rss',                   'memory',   'resident set size in pages'),
    ('statm', 'proc',  0, 'size',                  'memory',   'total program size in pages'),
    ('statm', 'proc',  3, 'text',                  'memory',   'text size'),
    ('statm', 'proc',  5, 'data',                  'memory',   'data and stack size'),

    ('stat',  'proc', 17, 'priority',              'setting',  'raw nice value or RT priority if negative'),
    ('stat',  'proc', 18, 'nice',                  'setting',  'nice value'),
    ('stat',  'proc', 39, 'rt_priority',           'setting',  'RT priority'),
    ('stat',  'proc', 40, 'policy',                'setting',  'scheduling policy'),

    # unhelpful ones

    # ('stat',  28, 'kstkesp',               'address',  'current stack pointer'),
    # ('stat',  29, 'kstkeip',               'address',  'current instruction pointer'),
    # ('stat',  34, 'wchan',                 'address',  'address in kernel where process sleeps'),
    # ('stat',  27, 'startstack',            'address',  'start of stack memory region'),
    # ('stat',  25, 'startcode',             'address',  'start of executable memory region'),
    # ('stat',  26, 'endcode',               'address',  'end of executable memory region'),
    # ('stat',  44, 'start_data',            'address',  'start of data memory region'),
    # ('stat',  45, 'end_data',              'address',  'end of data memory region'),
    # ('stat',  46, 'start_brk',             'address',  'start of brk expansion memory region'),
    # ('stat',  47, 'arg_start',             'address',  'start of argv cmdline args memory region'),
    # ('stat',  48, 'arg_end',               'address',  'end of argv cmdline args memory region'),
    # ('stat',  49, 'env_start',             'address',  'start of program environment memory region'),
    # ('stat',  50, 'env_end',               'address',  'end of program environment memory region'),

    # ('stat',  20, 'itrealvalue',           'obsolete', 'hard coded to 0'),
    # ('stat',  24, 'rsslim',                'obsolete', 'size of ram in bytes'),
    # ('stat',  30, 'signal',                'obsolete', 'bitmap of pending signals as dec'),
    # ('stat',  31, 'blocked',               'obsolete', 'bitmap of blocked signals as dec'),
    # ('stat',  32, 'sigignore',             'obsolete', 'bitmap of ignored signals as dec'),
    # ('stat',  33, 'sigcatch',              'obsolete', 'bitmap of caught signals as dec'),
    # ('stat',  35, 'nswap',                 'obsolete', 'number of swapped pages'),
    # ('stat',  36, 'cnswap',                'obsolete', 'cumulative nswap with children'),
    # ('statm',  4, 'lib',                   'obsolete', 'library size'),
    # ('statm',  6, 'dt',                    'obsolete', 'dirty pages'),
]

def read(name):
    def split(x):
        if '(' in x:
            y = x.split('(')[1].split(')')[0]
            v = ''.join(x.split(y)).split()
            v[1] = y
            return v
        else:
            return x.split()
    return [split(open(n + '/' + name).read()) for n in sys.argv[1:]]

def separator():
    class C:
        tag = None
    def f(tag):
        if C.tag is not None and C.tag != tag:
            print '-' * 236
        C.tag = tag
    return f

data = {
    'stat': read('stat'),
    'statm': read('statm')
}
sep = separator()
for fname, scope, idx, name, tag, desc in desc:
    sep(tag)
    sys.stdout.write('%-5s %-4s %3s %-22s %-10s %-48s' % (fname, scope, idx, name, tag, desc))
    for stat, statm in zip(data['stat'], data['statm']):
        sys.stdout.write(' %16s' % locals()[fname][idx])
    sys.stdout.write('\n')
