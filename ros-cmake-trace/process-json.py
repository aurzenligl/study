#!/usr/bin/env python3

import json
import sys
import functools

version_parsed = False
epoch_parsed = False
epoch = 0
last_record = None
generated = {}

def version_parse():
    global version_parsed
    if not version_parsed:
        version_parsed = True
        return True
    return False

def epoch_parse(rec):
    global epoch_parsed
    global epoch
    if not epoch_parsed:
        epoch_parsed = True
        epoch = rec['time']
        return True
    return False

def update_time(rec):
    global epoch
    rec['time'] = rec['time'] - epoch
    rec['elapsed'] = rec['time'] - last_record['time'] if last_record else 0

def update_generated(rec):
    global generated
    file = rec['file']
    if file not in generated:
        with open(file) as f:
            lines = f.readlines()
            gen = lines[0].strip() == '# generated from catkin/cmake/template/pkgConfig.cmake.in'
            generated[file] = (gen, len(lines))
    rec['generated'] = generated[file][0]
    rec['file_len'] = generated[file][1]

def to_location(rec):
    return '%s:%d' % (rec['file'], rec['line'])

def to_time(rec):
    return '%.3f' % (rec['elapsed'] * 1000)

def to_expr(rec):
    expr = '%s(%s)' % (rec['cmd'], ', '.join(rec['args']))
    expr = expr.replace('\n', ' ')
    return expr[:80]

@functools.lru_cache(128)
def to_template(file_len):
    global generated
    for file, val in generated.items():
        if val[0] and val[1] == file_len:
            return '%s(%s)' % (file, file_len)
    raise Exception('template not found')

def print_each_record(recs):
    for rec in recs:
        print('%8s %3s %-80s %s' % (to_time(rec), rec['frame'], to_location(rec), to_expr(rec)))

def print_each_file(recs):
    by_file = []
    for rec in recs:
        if by_file and rec['file'] == by_file[-1]['file']:
            by_file[-1]['elapsed'] += rec['elapsed']
        else:
            by_file.append(rec)
    for rec in by_file:
        print('%8s %3s %-80s' % (to_time(rec), rec['frame'], rec['file']))

def print_each_expr(recs):
    by_line = {}
    for rec in recs:
        if rec['generated']:
            key = (rec['file_len'], rec['line'])

            by_line.setdefault(key, {'elapsed': 0, 'count': 0, 'file': to_template(rec['file_len']), 'line': rec['line']})
            by_line[key]['count'] += 1
            by_line[key]['elapsed'] += rec['elapsed']

            #if key == (223, 190):
            #    print(to_time(rec), rec['file'], to_expr(rec))

    for key, rec in by_line.items():
        print('%10s %8s %s' % (to_time(rec), rec['count'], to_location(rec)))

records = []
with open(sys.argv[1]) as f:
    for r in f.read().splitlines():
        if version_parse():
            continue
        rec = json.loads(r)
        epoch_parse(rec)
        update_time(rec)
        update_generated(rec)
        records.append(rec)
        last_record = rec

print_each_expr(records)
