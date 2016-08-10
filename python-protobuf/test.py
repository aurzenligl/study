#!/usr/bin/env python

import collections
from alu_pb2 import ALU

def fill_by_dict(obj, data):
    def deenumize(obj, key, val):
        et = obj.DESCRIPTOR.fields_by_name[key].enum_type
        return et and et.values_by_name[val].number or val

    def fill_by_list(obj, key, val_list):
        for val in val_list:
            if isinstance(val, collections.Mapping):
                fill_by_dict(obj.d.add(), val)
            else:
                getattr(obj, key).append(deenumize(obj, key, val))

    for key, val in data.items():
        if isinstance(val, collections.Mapping):
            fill_by_dict(getattr(obj, key), val)
        elif isinstance(val, collections.Iterable) and not isinstance(val, basestring):
            fill_by_list(obj, key, val)
        else:
            setattr(obj, key, deenumize(obj, key, val))
    return obj

class Mo(object):
    def __init__(self, obj):
        self.obj = obj

    @staticmethod
    def from_dict(data):
        return fill_by_dict(ALU(), data)

    def __str__(self):
        return str(self.obj)

mo = Mo.from_dict({
    'a': 1,
    'b': 'the_value',
    'c': {
        'x': [1, 2, 3],
        'y': ['1', '22', '333'],
    },
    'd': [
        { },
        {'v': 2},
    ],
    'e': 'Fenum2',
    'f': ['Oenum1', 'Oenum2']
})

print(mo)

