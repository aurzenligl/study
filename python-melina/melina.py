#!/usr/bin/env python

import os
import sys
import shlex
import argparse
import enum
import re
import io
import lxml.etree as ET
import decimal

'''
Mo
    name
    children
    fields (Struct, Enum, Scalar)
Field
    name
    cardinality
    type
Struct
    name
    fields (...)
Enum
    name
    enumerators (...)
Int
Float
String
'''

def _indent(text, value):
    return '\n'.join(x and (' ' * value + x) or '' for x in text.split('\n'))

def _sanitize(obj, classes, maybe = False):
    if maybe:
        if obj == None:
            return
    assert isinstance(obj, classes), 'Object %s is not an instance of expected classes: %s' % (repr(obj), classes)
    return obj

def _sanitize_list(lst, classes, maybe = False):
    if maybe:
        if lst == None:
            return
    for obj in lst:
        assert isinstance(obj, classes), 'List element is an instance of unexpected class'
    return lst

_re_identifier = re.compile('^[a-zA-Z_][a-zA-Z0-9_]*$')
def _sanitize_identifier(name):
    assert _re_identifier.match(name), 'String does not represent a valid identifier'
    return name

_re_enumerator_identifier = re.compile('^[a-zA-Z0-9_]+$')
def _sanitize_enumerator_identifier(name):
    assert _re_enumerator_identifier.match(name), 'String does not represent a valid identifier'
    return name

def _add_to_1st_line(text, doc):
    lines = text.splitlines()
    lines[0] += '  // ' + doc
    out = ''.join((line + '\n' for line in lines))
    return out

class TranslationUnit(object):
    def __init__(self, mos):
        self.mos = _sanitize_list(mos, Mo)

    def __repr__(self):
        return '<TranslationUnit with %s mos>' % len(self.mos)

    def __str__(self):
        return ''.join((str(mo) for mo in self.mos))

class Mo(object):
    default_flags = [False, True, True, True]

    def __init__(self, name, fields, children, doc, flags):
        self.name = _sanitize_identifier(name)
        self.fields = _sanitize_list(fields, Field)
        self.children = _sanitize_list(children, MoChild)
        self.doc = _sanitize(doc, str, maybe=True)
        self.flags = _sanitize_list(flags, bool, maybe=True)

        if flags is not None:
            assert len(flags) == 4

    def __repr__(self):
        return '<Mo %s>' % self.name

    def __str__(self):
        text = 'mo'
        if self.flags:
            text += '(%s)' % ''.join((char for flag, char in zip(self.flags, 'hcud') if flag))
        text += (
            ' %s' % self.name + ':' + ''.join((' ' + str(x) for x in self.children)) + '\n' +
            _indent(''.join((str(field) for field in self.fields)), 4)
        )
        if self.doc:
            return _add_to_1st_line(text, self.doc)
        return text

class MoChild(object):
    def __init__(self, name, max_count):
        self.name = _sanitize_identifier(name)
        self.max_count = _sanitize(max_count, (int, long), maybe=True)

        if max_count is not None:
            assert max_count >= 1

    def __repr__(self):
        return '<MoChild %s>' % self.name

    def __str__(self):
        if self.max_count is not None:
            return '%s(%s)' % (self.name, self.max_count)
        else:
            return self.name

class Field(object):
    def __init__(self, name, type_, cardinality, doc):
        self.name = _sanitize_identifier(name)
        self.type = _sanitize(type_, (Struct, Enum, Scalar))
        self.cardinality = _sanitize(cardinality, Cardinality)
        self.doc = _sanitize(doc, str, maybe=True)

    def __repr__(self):
        return '<Field %s>' % self.name

    def __str__(self):
        if isinstance(self.type, Scalar):
            opts = self.type.options
            if opts:
                opts = ' ' + opts
            text = '%s %s %s%s\n' % (self.cardinality, self.type, self.name, opts)
        else:
            text = '%s %s' % (self.cardinality, self.type)
        if self.doc:
            return _add_to_1st_line(text, self.doc)
        return text

class Cardinality(object):
    def __init__(self, kind, max_count = None):
        self.kind = _sanitize(kind, CardinalityKind)
        self.max_count = _sanitize(max_count, (int, long), maybe=True)

        if max_count is not None:
            assert kind == CardinalityKind.REPEATED
            assert max_count > 1

    def __repr__(self):
        return '<Cardinality %s>' % self.kind.name.lower()

    def __str__(self):
        if self.kind == CardinalityKind.REPEATED and self.max_count:
            return 'repeated(%s)' % self.max_count
        else:
            return self.kind.name.lower()

class CardinalityKind(enum.Enum):
    REQUIRED = 0
    OPTIONAL = 1
    REPEATED = 2

class Struct(object):
    def __init__(self, name, fields):
        self.name = _sanitize_identifier(name)
        self.fields = _sanitize_list(fields, Field)

    def __repr__(self):
        return '<Struct %s>' % self.name

    def __str__(self):
        return (
            'struct %s\n' % self.name +
            _indent(''.join((str(field) for field in self.fields)), 4)
        )

class Enum(object):
    def __init__(self, name, enumerators, default = None):
        self.name = _sanitize_identifier(name)
        self.enumerators = _sanitize_list(enumerators, Enumerator)
        self.default = _sanitize(default, (int, long), maybe=True)

        if default is not None:
            assert default in ((x.value for x in enumerators))

    def __repr__(self):
        return '<Enum %s>' % self.name

    def __str__(self):
        return (
            'enum %s%s\n' % (self.name, ' [default = %s]' % self.default if self.default is not None else '') +
            _indent(''.join((str(enumer) for enumer in self.enumerators)), 4)
        )

class Enumerator(object):
    def __init__(self, name, value):
        self.name = _sanitize_enumerator_identifier(name)
        self.value = _sanitize(value, (int, long))

    def __repr__(self):
        return '<Enumerator %s>' % self.name

    def __str__(self):
        return '%s = %s\n' % (self.name, self.value)

class Scalar(object):

    def __repr__(self):
        return '<%s>' % self.__class__.__name__

    def __str__(self):
        return '%s' % self.__class__.__name__.lower()

class Bool(Scalar):
    def __init__(self, default = None):
        self.default = _sanitize(default, bool, maybe=True)

    @property
    def defaultstr(self):
        if self.default is not None:
            return 'true' if self.default else 'false'
        else:
            return ''

    @property
    def options(self):
        if self.default is not None:
            return '[default = %s]' % self.defaultstr
        else:
            return ''

class Int(Scalar):

    def __init__(self, minval, maxval, step, default = None, units = None):
        self.minval = _sanitize(minval, (int, long, decimal.Decimal), maybe=True)
        self.maxval = _sanitize(maxval, (int, long, decimal.Decimal), maybe=True)
        self.step = _sanitize(step, (int, long, decimal.Decimal), maybe=True)
        self.default = _sanitize(default, (int, long, decimal.Decimal), maybe=True)
        self.units = _sanitize(units, str, maybe=True)

        if minval is not None or maxval is not None:
            if step is None:
                assert isinstance(minval, (int, long))
                assert isinstance(maxval, (int, long))
            else:
                assert minval is not None
                assert maxval is not None
                assert step != 0
            assert minval <= maxval

        if default is not None:
            if step is None:
                assert isinstance(default, (int, long))

    @staticmethod
    def dectostr(dec):
        decstr = str(dec)
        if 'E' in decstr:
            exp = int(str(dec).split('E')[1])
            if exp < 0:
                return '%%.%sf' % abs(exp) % dec
            else:
                return '%f' % dec
        else:
            return decstr

    @property
    def minvalstr(self):
        return self.dectostr(self.minval)

    @property
    def maxvalstr(self):
        return self.dectostr(self.maxval)

    @property
    def stepstr(self):
        return self.dectostr(self.step)

    @property
    def defaultstr(self):
        if self.default is not None:
            return self.dectostr(self.default)
        else:
            return ''

    @property
    def options(self):
        opts = []
        if self.default is not None:
            opts.append('default = %s' % self.defaultstr)
        if self.units is not None:
            opts.append('units = "%s"' % self.units)
        if opts:
            return '[%s]' % ', '.join(opts)
        else:
            return ''

    def __str__(self):
        if self.step is not None:
            return 'int(%s, %s, %s)' % (self.minvalstr, self.maxvalstr, self.stepstr)
        elif self.minval is not None:
            return 'int(%s..%s)' % (self.minvalstr, self.maxvalstr)
        else:
            return 'int'

class String(Scalar):
    def __init__(self, minlen, maxlen, default = None):
        self.minlen = _sanitize(minlen, (int, long), maybe=True)
        self.maxlen = _sanitize(maxlen, (int, long), maybe=True)
        self.default = _sanitize(default, str, maybe=True)

        if minlen is not None or maxlen is not None:
            assert 0 <= minlen
            assert 0 <= maxlen
            assert minlen <= maxlen
            if default is not None:
                assert minlen <= len(default) <= maxlen
        else:
            assert minlen is None
            assert maxlen is None

    @property
    def options(self):
        if self.default is not None:
            return '[default = "%s"]' % self.default
        else:
            return ''

    def __str__(self):
        out = 'string'
        if self.minlen is not None:
            out += '(%s..%s)' % (self.minlen, self.maxlen)
        return out

def _line(text, pos):
    return text.count('\n', 0, pos) + 1

def _col(text, pos):
    return pos - text.rfind('\n', 0, pos)

class MetaSpan(object):
    __slots__ = ('input', 'start', 'end')

    def __init__(self, input, start, end=None):
        self.input = input
        self.start = start
        self.end = end

    @property
    def start_linecol(self):
        return (_line(self.input, self.start), _col(self.input, self.start))

    @property
    def start_repr(self):
        return '%s:%s' % self.start_linecol

    @property
    def start_line(self):
        return self.input.splitlines()[self.start_linecol[0] - 1]

    @property
    def end_linecol(self):
        return (_line(self.input, self.end), _col(self.input, self.end))

    @property
    def end_repr(self):
        return '%s:%s' % self.end_linecol

    def __repr__(self):
        return '%s:%s-%s:%s' % (self.start_linecol + self.end_linecol)

class MetaTokenKind(enum.Enum):
    KEYW = 0      # mo, struct, enum, repeated, optional, int, float, string
    NAME = 1      # [_a-zA-Z][_a-zA-Z0-9]*
    STRING = 2    # ".*"
    NUMBER = 3    # [0-9]*
    FLOAT = 4     # [0-9]*\.[0-9]*
    NUMNAME = 5   # [_a-zA-Z0-9]+
    LCB = 6       # {
    RCB = 7       # }
    LSB = 8       # [
    RSB = 9       # ]
    LP = 10       # (
    RP = 11       # )
    SEMI = 12     # ;
    COMMA = 13    # ,
    ASSIGN = 14   # =
    ARROW = 15    # ->
    TWODOT = 16   # ..
    COMMENT = 17  # '//\n', '/**/' <ignored, stored>
    END = 18

class MetaToken(object):
    __slots__ = ('kind', 'value', 'span', 'string')

    '''TODO remove repr_vals in favor of value'''
    _repr_vals = {
        MetaTokenKind.LCB: '{',
        MetaTokenKind.RCB: '}',
        MetaTokenKind.LSB: '[',
        MetaTokenKind.RSB: ']',
        MetaTokenKind.LP: '(',
        MetaTokenKind.RP: ')',
        MetaTokenKind.SEMI: ';',
        MetaTokenKind.COMMA: ',',
        MetaTokenKind.ASSIGN: '=',
        MetaTokenKind.ARROW: '->',
        MetaTokenKind.TWODOT: '..',
    }

    _repr_direct = (MetaTokenKind.KEYW, MetaTokenKind.NAME, MetaTokenKind.NUMBER, MetaTokenKind.NUMNAME, MetaTokenKind.COMMENT)

    def __init__(self, kind, value = None, span = None, string = None):
        self.kind = kind
        self.value = value
        self.span = span
        self.string = (string is not None) and string or value

    @property
    def pair(self):
        return (self.kind, self.value)

    def __repr__(self):
        if self.span:
            repr = "<MetaToken %s %s" % (self.span, self.kind.name)
        else:
            repr = "<MetaToken %s" % self.kind.name

        if self.kind in self._repr_direct:
            return repr + " %s>" % self.value
        reprval = self._repr_vals.get(self.kind)
        if reprval:
            return repr + " %s>" % reprval
        return repr + ">"

class MetaParserError(Exception):
    def __init__(self, message, filename, span):
        Exception.__init__(self, message)
        self.filename = filename
        self.span = span

    @property
    def origin(self):
        return '%s:%s' % (self.filename or 'stdin', self.span.start_repr)

    @property
    def prettymsg(self):
        return '%s: error: %s\n%s\n%s\n' % (
            self.origin,
            self.message,
            self.span.start_line,
            ' ' * (self.span.start_linecol[1] - 1) + '^'
        )

class MetaTokenizer(object):
    def __init__(self, input, filename=None):
        self.input = input
        self.filename = filename
        self.pos = 0
        self.token = MetaToken(MetaTokenKind.END)
        self.comments = []

    @classmethod
    def from_file(cls, filename):
        return MetaTokenizer(open(filename).read())

    def get(self):
        self.token = token = self._get()
        return token

    @property
    def cur(self):
        return self.token

    _sre = re.compile('|'.join((
        r'(?P<name>[a-zA-Z_][a-zA-Z0-9_]*)',
        r'(?P<string>"[^"]*")',
        r'(?P<number>-?[0-9]+(?![a-zA-Z0-9_]|\.[^\.]))',
        r'(?P<float>-?[0-9]*\.(?!\.)[0-9]*(?![a-zA-Z0-9_]))',
        r'(?P<numname>-?[0-9]+[a-zA-Z_][a-zA-Z0-9_]*)',
        r'(?P<operator>(->)|(\.\.)|[{}\[\]();,=])',
        r'(?P<comment>(//.*\n|/\*(\*(?!/)|[^*])*\*/))',
        r'(?P<space>\s+)',
    )))

    _keywords = ('mo', 'struct', 'enum', 'repeated', 'optional', 'bool', 'int', 'float', 'string')

    _operators = {
        '->': MetaTokenKind.ARROW,
        '..': MetaTokenKind.TWODOT,
        '{': MetaTokenKind.LCB,
        '}': MetaTokenKind.RCB,
        '[': MetaTokenKind.LSB,
        ']': MetaTokenKind.RSB,
        '(': MetaTokenKind.LP,
        ')': MetaTokenKind.RP,
        ';': MetaTokenKind.SEMI,
        ',': MetaTokenKind.COMMA,
        '=': MetaTokenKind.ASSIGN,
    }

    def _get(self):
        while True:
            start = self.pos
            pack = self._read_raw()
            if pack:
                span = MetaSpan(self.input, start, self.pos - 1)
                string, group = pack
                if group == 'name':
                    if string in self._keywords:
                        return MetaToken(MetaTokenKind.KEYW, string, span=span)
                    else:
                        return MetaToken(MetaTokenKind.NAME, string, span=span)
                elif group == 'string':
                    return MetaToken(MetaTokenKind.STRING, string[1:-1], span=span)
                elif group == 'numname':
                    return MetaToken(MetaTokenKind.NUMNAME, string, span=span)
                elif group == 'number':
                    return MetaToken(MetaTokenKind.NUMBER, int(string), span=span, string=string)
                elif group == 'float':
                    return MetaToken(MetaTokenKind.FLOAT, decimal.Decimal(string), span=span, string=string)
                elif group == 'operator':
                    return MetaToken(self._operators[string], span=span)
                elif group == 'comment':
                    return MetaToken(MetaTokenKind.COMMENT, string, span=span)
                elif group == 'space':
                    continue
                assert False, "o-oh, we shouldn't end up here"

            if self.pos == len(self.input):
                return MetaToken(MetaTokenKind.END)
            else:
                ch = self.input[self.pos]
                span = MetaSpan(self.input, self.pos, self.pos)
                raise MetaParserError('unexpected character: "%s", ord=%s' % (ch, ord(ch)), self.filename, span)

    def _read_raw(self):
        match = self._sre.match(self.input, self.pos)
        if match:
            self.pos = match.end()
            return match.group(), match.lastgroup

def _isupperdoc(tok):
    newline = tok.span.input.rfind('\n', 0, tok.span.start)
    linestart = (newline != -1) and (newline + 1) or 0
    line = tok.span.input[linestart:tok.span.start]
    return not line or line.isspace()

def _isrightdoc(doctok, symtok):
    newline = doctok.span.input.rfind('\n', 0, doctok.span.end)
    linestart = (newline != -1) and (newline + 1) or 0
    return linestart <= symtok.span.end

def _docstring(tok):
    if tok.value[:3] == '/**':
        lines = [line.strip() for line in tok.value[2:-2].splitlines()]
        lines = [line for line in lines if line]
        if all((line[0] == '*' for line in lines)):
            lines = [line[1:].strip() for line in lines]
            lines = [line for line in lines if line]
            return ' '.join(lines)
    elif tok.value[:3] == '///':
        return tok.value[3:].strip()

def _mergedoc(ldoc, rdoc):
    if ldoc is not None or rdoc is not None:
        return (ldoc or '') + (rdoc or '')

class MetaParser(object):
    '''
    specification:
        mo_list end
        end

    mo_list:
        mo mo_list
        mo

    mo:
        mo_head { field_list } ;

    mo_head:
        MO name -> mo_children
        MO name

    mo_children:
        name , mo_children
        name

    field_list:
        field field_list
        field

    field:
        field_qualifier field_definition
        field_definition

    field_qualifier:
        repeated
        optional

    field_definition:
        struct
        enum
        scalar

    struct:
        STRUCT name { field_list } ;

    enum:
        ENUM name { enumerator_list } ;

    enumerator_list:
        enumerator , enumerator_list
        enumerator ,
        enumerator

    enumerator:
        name = number
        name

    scalar:
        scalar_type name ;

    scalar_type:
        INT
        STRING
    '''

    def __init__(self, input, filename=None):
        self.ts = MetaTokenizer(input, filename)
        self.filename = filename
        self.cached_comment = None

    @classmethod
    def from_file(cls, filename):
        return cls(open(filename).read(), filename)

    @property
    def cur(self):
        return self.ts.cur

    def get(self):
        self.cached_comment = []
        while True:
            tok = self.ts.get()
            if tok.kind == MetaTokenKind.COMMENT:
                self.cached_comment.append(tok)
                continue
            return tok

    def parse(self):
        mos = []
        while True:
            if self.get().kind == MetaTokenKind.END:
                break
            mos.append(self.mo())

        return TranslationUnit(mos)

    def mo(self):
        doc = None
        if self.cached_comment and _isupperdoc(self.cached_comment[-1]):
            doc = _docstring(self.cached_comment[-1])

        if self.cur.pair != (MetaTokenKind.KEYW, 'mo'):
            raise MetaParserError('expected keyword "mo"', self.filename, self.cur.span)

        flags = None
        if self.get().kind == MetaTokenKind.LP:
            if self.get().kind != MetaTokenKind.NAME or not all((x in 'hcud' for x in self.cur.value)):
                raise MetaParserError('expected hidden("h"), create("c"), update("u"), delete("d") flags', self.filename, self.cur.span)
            flags = [char in self.cur.value for char in 'hcud']
            if self.get().kind != MetaTokenKind.RP:
                raise MetaParserError('expected closing paren after flags specification', self.filename, self.cur.span)
            self.get()

        if self.cur.kind != MetaTokenKind.NAME:
            raise MetaParserError('expected mo name', self.filename, self.cur.span)

        name = self.cur.value
        children = []
        fields = []

        if self.get().kind == MetaTokenKind.ARROW:
            children = self.mo_child_list()

        if self.cur.kind != MetaTokenKind.LCB:
            raise MetaParserError('expected mo definition', self.filename, self.cur.span)

        self.get()
        while True:
            if self.cur.kind == MetaTokenKind.RCB:
                break
            fields.append(self.field())

        prev = self.cur
        if self.get().kind != MetaTokenKind.SEMI:
            raise MetaParserError('expected semicolon after mo definition', self.filename, prev.span)

        return Mo(name, fields, children, doc, flags)

    def mo_child_list(self):
        def child(self):
            if self.get().kind != MetaTokenKind.NAME:
                raise MetaParserError('expected mo child name', self.filename, self.cur.span)
            name = self.cur.value
            max_count = None
            if self.get().kind == MetaTokenKind.LP:
                if self.get().kind != MetaTokenKind.NUMBER:
                    raise MetaParserError('expected mo child max count', self.filename, self.cur.span)
                max_count = self.cur.value
                if max_count < 1:
                    raise MetaParserError('expected positive integer', self.filename, self.cur.span)
                if self.get().kind != MetaTokenKind.RP:
                    raise MetaParserError('expected closing paren after max count specification', self.filename, self.cur.span)
                self.get()
            return MoChild(name, max_count)

        children = [child(self)]
        while True:
            if self.cur.kind != MetaTokenKind.COMMA:
                break
            children.append(child(self))

        return children

    def field(self):
        # doc
        doc = None
        if self.cached_comment and _isupperdoc(self.cached_comment[-1]):
            doc = _docstring(self.cached_comment[-1])

        # cardinality
        cardinality = Cardinality(CardinalityKind.REQUIRED)
        if self.cur.pair == (MetaTokenKind.KEYW, 'repeated'):
            max_count = None
            if self.get().kind == MetaTokenKind.LP:
                if self.get().kind != MetaTokenKind.NUMBER:
                    raise MetaParserError('expected max count in parens', self.filename, self.cur.span)
                max_count = self.cur.value
                if max_count <= 1:
                    raise MetaParserError('expected max count larger than 1', self.filename, self.cur.span)
                prev = self.cur
                if self.get().kind != MetaTokenKind.RP:
                    raise MetaParserError('expected closing paren after max count', self.filename, prev.span)
                self.get()
            cardinality = Cardinality(CardinalityKind.REPEATED, max_count)
        elif self.cur.pair == (MetaTokenKind.KEYW, 'optional'):
            cardinality = Cardinality(CardinalityKind.OPTIONAL)
            self.get()

        # type
        if not (self.cur.kind == MetaTokenKind.KEYW and self.cur.value in ('struct', 'enum', 'bool', 'int', 'string')):
            raise MetaParserError('expected field definition', self.filename, self.cur.span)
        if self.cur.pair == (MetaTokenKind.KEYW, 'struct'):
            type_, name = self.struct()
        elif self.cur.pair == (MetaTokenKind.KEYW, 'enum'):
            type_, name = self.enum()
        else:
            type_, name = self.scalar()

        # doc
        prev = self.cur
        self.get()
        if self.cached_comment and _isrightdoc(self.cached_comment[0], prev):
            doc = _mergedoc(doc, _docstring(self.cached_comment[0]))

        return Field(name, type_, cardinality, doc)

    def struct(self):
        assert self.cur.pair == (MetaTokenKind.KEYW, 'struct')

        if self.get().kind != MetaTokenKind.NAME:
            raise MetaParserError('expected struct name', self.filename, self.cur.span)

        name = self.cur.value
        typename = name[0].upper() + name[1:]
        fields = []

        if self.get().kind != MetaTokenKind.LCB:
            raise MetaParserError('expected struct definition', self.filename, self.cur.span)

        self.get()
        while True:
            if self.cur.kind == MetaTokenKind.RCB:
                break
            fields.append(self.field())

        prev = self.cur
        if self.get().kind != MetaTokenKind.SEMI:
            raise MetaParserError('expected semicolon after struct definition', self.filename, prev.span)

        return Struct(typename, fields), name

    def enum(self):
        assert self.cur.pair == (MetaTokenKind.KEYW, 'enum')

        if self.get().kind != MetaTokenKind.NAME:
            raise MetaParserError('expected enum name', self.filename, self.cur.span)

        name = self.cur.value
        typename = name[0].upper() + name[1:]

        default = None
        if self.get().kind == MetaTokenKind.LSB:
            if self.get().kind != MetaTokenKind.NAME:
                raise MetaParserError('expected option name', self.filename, self.cur.span)
            if self.cur.value != 'default':
                raise MetaParserError('unexpected option name', self.filename, self.cur.span)
            if self.get().kind != MetaTokenKind.ASSIGN:
                raise MetaParserError('expected assignment operator', self.filename, self.cur.span)
            if self.get().kind != MetaTokenKind.NUMBER:
                raise MetaParserError('expected integer default value', self.filename, self.cur.span)
            default = self.cur.value
            deftok = self.cur
            if self.get().kind != MetaTokenKind.RSB:
                raise MetaParserError('expected closing square bracket after options', self.filename, self.cur.span)
            self.get()

        if self.cur.kind != MetaTokenKind.LCB:
            raise MetaParserError('expected enum definition', self.filename, self.cur.span)

        enumerators = self.enumerator_list()

        if self.cur.kind != MetaTokenKind.RCB:
            raise MetaParserError('expected brace closing enum definition', self.filename, self.cur.span)

        prev = self.cur
        if self.get().kind != MetaTokenKind.SEMI:
            raise MetaParserError('expected semicolon after enum definition', self.filename, prev.span)

        if default is not None:
            if default not in ((x.value for x in enumerators)):
                raise MetaParserError('expected default value corresponding to enumerator', self.filename, deftok.span)

        return Enum(typename, enumerators, default), name

    def enumerator_list(self):
        enumerators = []
        value = 0

        while True:
            if self.get().kind not in (MetaTokenKind.NAME, MetaTokenKind.NUMBER, MetaTokenKind.NUMNAME):
                break

            name = self.cur.string

            if self.get().kind == MetaTokenKind.ASSIGN:
                if self.get().kind != MetaTokenKind.NUMBER:
                    raise MetaParserError('expected enumerator value', self.filename, self.cur.span)
                value = self.cur.value
                self.get()

            enumerators.append(Enumerator(name, value))
            value += 1

            if self.cur.kind != MetaTokenKind.COMMA:
                break

        return enumerators

    def scalar(self):
        if self.cur.pair == (MetaTokenKind.KEYW, 'bool'):
            def sanitize(self, value):
                if value == 'true':
                    return True
                elif value == 'false':
                    return False
                else:
                    raise MetaParserError('expected %s' % opt_description, self.filename, self.cur.span)

            type_type = Bool
            type_args = ()
            type_possible_opts = {'default': ((MetaTokenKind.NAME,), 'boolean value', sanitize)}
            self.get()
        elif self.cur.pair == (MetaTokenKind.KEYW, 'int'):
            minval, maxval, step = None, None, None
            if self.get().kind == MetaTokenKind.LP:
                if self.get().kind not in (MetaTokenKind.NUMBER, MetaTokenKind.FLOAT):
                    raise MetaParserError('expected minimum value', self.filename, self.cur.span)
                minvaltok = self.cur
                minval = self.cur.value

                if self.get().kind not in (MetaTokenKind.TWODOT, MetaTokenKind.COMMA):
                    raise MetaParserError('expected double dot or comma', self.filename, self.cur.span)
                if self.cur.kind == MetaTokenKind.TWODOT:
                    if minvaltok.kind != MetaTokenKind.NUMBER:
                        raise MetaParserError('expected integer minimum value', self.filename, minvaltok)
                    if self.get().kind != MetaTokenKind.NUMBER:
                        raise MetaParserError('expected integer maximum value', self.filename, self.cur.span)
                    maxval = self.cur.value
                else:
                    if self.get().kind not in (MetaTokenKind.NUMBER, MetaTokenKind.FLOAT):
                        raise MetaParserError('expected maximum value', self.filename, self.cur.span)
                    maxval = self.cur.value
                    if self.get().kind != MetaTokenKind.COMMA:
                        raise MetaParserError('expected comma', self.filename, self.cur.span)
                    if self.get().kind not in (MetaTokenKind.NUMBER, MetaTokenKind.FLOAT):
                        raise MetaParserError('expected step', self.filename, self.cur.span)
                    step = self.cur.value

                if self.get().kind != MetaTokenKind.RP:
                    raise MetaParserError('expected closing paren after int specification', self.filename, self.cur.span)
                self.get()
            type_type = Int
            type_args = (minval, maxval, step)
            type_possible_opts = {
                'units': ((MetaTokenKind.STRING,), 'unit name', None),
                'default': ((MetaTokenKind.NUMBER,), 'integer default value', None)
                    if step is None else ((MetaTokenKind.NUMBER, MetaTokenKind.FLOAT), 'float default value', None)
            }
        elif self.cur.pair == (MetaTokenKind.KEYW, 'string'):
            minlen, maxlen = None, None
            if self.get().kind == MetaTokenKind.LP:
                if self.get().kind != MetaTokenKind.NUMBER:
                    raise MetaParserError('expected minimum string length', self.filename, self.cur.span)
                minlen = self.cur.value
                if self.get().kind != MetaTokenKind.TWODOT:
                    raise MetaParserError('expected double dot operator', self.filename, self.cur.span)
                if self.get().kind != MetaTokenKind.NUMBER:
                    raise MetaParserError('expected maximum string length', self.filename, self.cur.span)
                maxlen = self.cur.value
                if self.get().kind != MetaTokenKind.RP:
                    raise MetaParserError('expected closing paren after string specification', self.filename, self.cur.span)
                self.get()
            type_type = String
            type_args = (minlen, maxlen)
            type_possible_opts = {'default': ((MetaTokenKind.STRING,), 'default value', None)}
        else:
            assert False, "o-oh, we shouldn't end up here"

        if self.cur.kind != MetaTokenKind.NAME:
            raise MetaParserError('expected scalar name', self.filename, self.cur.span)
        name = self.cur.value

        type_opts = {}
        presemitok = self.cur
        if self.get().kind == MetaTokenKind.LSB:
            while True:
                if self.get().kind != MetaTokenKind.NAME:
                    break
                opt_name = self.cur.value
                opt = type_possible_opts.get(opt_name)
                if opt is None:
                    raise MetaParserError('unexpected option name', self.filename, self.cur.span)
                if self.get().kind != MetaTokenKind.ASSIGN:
                    raise MetaParserError('expected assignment operator', self.filename, self.cur.span)
                opt_kind, opt_description, opt_sanitizer = opt
                if self.get().kind not in opt_kind:
                    raise MetaParserError('expected %s' % opt_description, self.filename, self.cur.span)
                value = opt_sanitizer(self, self.cur.value) if opt_sanitizer else self.cur.value
                type_opts[opt_name] = value
                if self.get().kind != MetaTokenKind.COMMA:
                    break
            if self.cur.kind != MetaTokenKind.RSB:
                raise MetaParserError('expected closing square bracket after options', self.filename, self.cur.span)
            presemitok = self.cur
            self.get()

        type_ = type_type(*type_args, **type_opts)

        if self.cur.kind != MetaTokenKind.SEMI:
            raise MetaParserError('expected semicolon closing field definition', self.filename, presemitok.span)

        return type_, name

class MetaGenerator(object):
    def __init__(self, tu):
        self.tu = tu

    def to_file(self, filename):
        open(filename, 'w').write(self.to_string())

    def to_string(self):
        return '\n'.join(self.mo(mo) for mo in self.tu.mos)

    def mo(self, mo):
        out = ''
        if mo.doc:
            out += '/**\n * %s\n */\n' % mo.doc
        out += 'mo'
        if mo.flags:
            out += '(%s)' % ''.join((char for flag, char in zip(mo.flags, 'hcud') if flag))
        out += ' %s' % mo.name
        if mo.children:
            out += ' -> ' + ', '.join((str(child) for child in mo.children))
        out += '\n{\n' + _indent(self.fields(mo.fields), 4) + '};\n'
        return out

    def fields(self, fields):
        out = ''
        last_type = None
        for field in fields:
            if last_type:
                if not (isinstance(last_type, Scalar) and isinstance(field.type, Scalar)):
                    out += '\n'
            out += self.field(field)
            last_type = field.type
        return out

    def field(self, field):
        out = ''
        if field.doc and isinstance(field.type, (Struct, Enum)):
            out += '/**\n * %s\n */\n' % field.doc
        if field.cardinality.kind != CardinalityKind.REQUIRED:
            out += field.cardinality.kind.name.lower()
            if field.cardinality.kind == CardinalityKind.REPEATED and field.cardinality.max_count:
                out += '(%s)' % field.cardinality.max_count
            out += ' '
        if isinstance(field.type, Struct):
            out += self.struct(field.type, field.name)
        elif isinstance(field.type, Enum):
            out += self.enum(field.type, field.name)
        else:
            out += self.scalar(field.type, field.name)
            if field.doc:
                out += '  /// %s' % field.doc
        out += '\n'
        return out

    def struct(self, struct_, name):
        out = 'struct %s\n{\n' % name
        out += _indent(self.fields(struct_.fields), 4) + '};'
        return out

    def enum(self, enum_, name):
        out = 'enum %s' % name
        if enum_.default is not None:
            out += ' [default = %s]' % enum_.default
        out += '\n{\n'
        out += _indent(',\n'.join(('%s = %s' % (er.name, er.value) for er in enum_.enumerators)), 4)
        if enum_.enumerators:
            out += '\n'
        out += '};'
        return out

    def scalar(self, type_, name):
        opts = type_.options
        if opts:
            opts = ' ' + opts
        return '%s %s%s;' % (type_, name, opts)

'''TODO: use instead of "true"/"false"'''
def _bool(text):
    if text == 'true':
        return True
    elif text == 'false':
        return False

def _int(text):
    try:
        return int(text)
    except (ValueError, TypeError):
        return

def _positive_int(text):
    x = _int(text)
    if x is not None and x > 0:
        return x

def _nonnegative_int(text):
    x = _int(text)
    if x is not None and x >= 0:
        return x

def _decimal(text):
    try:
        return decimal.Decimal(text)
    except decimal.InvalidOperation:
        return

def _nonzero_decimal(text):
    x = _decimal(text)
    if x is not None and x != 0:
        return x

class XmlConst():
    default_repeated_max_occurs = 999999

class XmlParserError(Exception):
    def __init__(self, message, filename, position, input_):
        Exception.__init__(self, message)
        self.filename = filename
        self.input = input_
        if isinstance(position, tuple):
            self.lineno, self.colno = position
        else:
            self.lineno, self.colno = position, None

    @property
    def line(self):
        return self.input.splitlines()[self.lineno - 1]

    @property
    def origin(self):
        line = '%s:%s' % (self.filename, self.lineno)
        if self.colno:
            line += ':%s' % (self.colno)
        return line

    @property
    def prettymsg(self):
        msg = '%s: error: %s\n%s\n' % (
            self.origin,
            self.message,
            self.line
        )
        if self.colno:
            msg += ' ' * (self.colno - 1) + '^\n'
        return msg

class XmlParser(object):
    def __init__(self, input_, filename=None):
        '''TODO handle xml parser errors in driver'''
        '''TODO [langfeature] pdmeta version=int.int,
           header domain='str' product='str' release='str' version='str' revision='str' '''
        self.filename = filename
        self.input = input_
        try:
            self.et = ET.fromstring(input_)
        except ET.XMLSyntaxError as e:
            raise XmlParserError(e.message.split(', line ')[0], self.filename, e.position, self.input)

    @classmethod
    def from_file(cls, filename=None):
        return cls(open(filename).read(), filename)

    def parse(self):
        mos = [self.mo(mo) for mo in self.et.findall('.//managedObject')]
        return TranslationUnit(mos)

    def error(self, msg, elem):
        raise XmlParserError(msg, self.filename, elem.sourceline, self.input)

    def get(self, tag, attr, sanitizer = None, typename = 'string'):
        value = tag.get(attr)
        if value is None:
            self.error('expected "%s" attribute in "%s" tag' % (attr, tag.tag), tag)
        if sanitizer:
            value = sanitizer(value)
            if value is None:
                self.error('expected %s in "%s" attribute' % (typename, attr), tag)
        return value

    def get_maybe(self, tag, attr, sanitizer = None, typename = 'string'):
        value = tag.get(attr)
        if value is None:
            return
        if sanitizer:
            value = sanitizer(value)
            if value is None:
                self.error('expected %s in "%s" attribute' % (typename, attr), tag)
        return value

    def mo(self, mo):
        name = self.get(mo, 'class')
        doc = mo.get('fullName')
        flags = [
            self.get_maybe(mo, 'hidden', _bool, 'hidden boolean flag'),
            self.get_maybe(mo, 'create', _bool, 'hidden create flag'),
            self.get_maybe(mo, 'update', _bool, 'hidden update flag'),
            self.get_maybe(mo, 'delete', _bool, 'hidden delete flag'),
        ]
        if not all((fl is not None for fl in flags)):
            flags = None
        if flags == Mo.default_flags:
            flags = None
        children = self.mo_child_list(mo)
        fields = [self.field(field) for field in mo.findall('p')]
        return Mo(name, fields, children, doc, flags)

    def mo_child_list(self, mo):
        children = []
        for child in mo.findall('childManagedObject'):
            name = self.get(child, 'class')
            max_count = self.get_maybe(child, 'maxOccurs', _nonnegative_int, 'non-negative integer')
            children.append(MoChild(name, max_count))
        return children

    def field(self, field):
        name = self.get(field, 'name')
        typename = name[0].upper() + name[1:]
        doc = field.get('fullName')

        max_occurs = field.get('maxOccurs')
        if max_occurs is not None:
            max_occurs = _positive_int(max_occurs)
            if max_occurs is None:
                self.error('expected positive integer in "maxOccurs"', field)

        if max_occurs == 1 or max_occurs == None:
            creation = field.find('creation')
            if creation is not None:
                prio = self.get(creation, 'priority')
                if prio == 'optional':
                    cardinality = Cardinality(CardinalityKind.OPTIONAL)
                elif prio == 'mandatory':
                    cardinality = Cardinality(CardinalityKind.REQUIRED)
                else:
                    self.error('expected "optional" or "mandatory" cardinality', creation)
            else:
                cardinality = Cardinality(CardinalityKind.REQUIRED)
        else:
            if max_occurs == XmlConst.default_repeated_max_occurs:
                max_occurs = None
            cardinality = Cardinality(CardinalityKind.REPEATED, max_occurs)

        complex_ = field.find('complexType')
        simple = field.find('simpleType')
        if complex_ is not None:
            type_ = self.struct(complex_, typename)
        elif simple is not None:
            if simple.find('enumeration') is not None:
                type_ = self.enum(simple, typename)
            else:
                type_ = self.scalar(simple)
        else:
            self.error('expected "simpleType" or "complexType" tag under "p" tag', field)

        return Field(name, type_, cardinality, doc)

    def struct(self, complex_, name):
        fields = [self.field(field) for field in complex_.findall('p')]
        return Struct(name, fields)

    def enum(self, simple, name):
        enumerators = self.enumerator_list(simple)

        default = None
        deftag = simple.find('default')
        if deftag is not None:
            default = self.get(deftag, 'value', _int, 'int')
            if default not in ((x.value for x in enumerators)):
                self.error('expected default value corresponding to enumerator', deftag)

        return Enum(name, enumerators, default)

    def enumerator_list(self, simple):
        enumerators = []
        for enumer in simple.findall('enumeration'):
            name = self.get(enumer, 'text')
            name = name.replace('-', '_')
            value = _int(self.get(enumer, 'value'))
            if value is None:
                self.error('expected integer enumerator value', enumer)
            enumerators.append(Enumerator(name, value))
        return enumerators

    def scalar(self, simple):
        base = self.get(simple, 'base')

        if base == 'boolean':
            def get_default(self, simple):
                tag = simple.find('default')
                if tag is None:
                    return
                value = self.get(tag, 'value')
                if value == 'true':
                    return True
                elif value == 'false':
                    return False
                else:
                    self.error('expected "true" or "false" in "value" attribute', tag)

            return Bool(get_default(self, simple))

        elif base in ('integer', 'decimal'):
            def get_minmaxstep(self, editing):
                def mergestep(minval, maxval, step, divisor):
                    if minval is None or maxval is None:
                        return
                    if step is not None:
                        return step
                    if divisor is not None:
                        return 1 / decimal.Decimal(divisor)

                if editing is None:
                    return (None, None, None)
                divisor = self.get_maybe(editing, 'divisor', _positive_int, 'positive int')
                range = editing.find('range')
                if range is None:
                    return (None, None, None)
                rawstep = self.get_maybe(range, 'step', _nonzero_decimal, 'non-zero decimal')
                minmax_typeargs = (divisor is None and rawstep is None) and (_int, 'int') or (_decimal, 'float')
                minval = self.get(range, 'minIncl', *minmax_typeargs)
                maxval = self.get(range, 'maxIncl', *minmax_typeargs)
                if minval > maxval:
                    self.error('expected "minIncl" less than "maxIncl"', range)
                step = mergestep(minval, maxval, rawstep, divisor)
                return minval, maxval, step

            def get_units(self, editing):
                if editing is None:
                    return
                return editing.get('units')

            def get_default(self, simple, editing, step):
                deftag1 = simple.find('default')
                deftag2 = editing.find('default') if editing is not None else None
                if deftag1 is None and deftag2 is None:
                    return
                if deftag1 is not None and deftag2 is not None:
                    self.error('expected single "default" tag, found two', simple)
                tag = deftag1 if deftag1 is not None else deftag2
                def_typeargs = (_int, 'int') if step is None else (_decimal, 'float')
                value = self.get(tag, 'value', *def_typeargs)
                return value

            editing = simple.find('editing')
            minval, maxval, step = get_minmaxstep(self, editing)
            default = get_default(self, simple, editing, step)
            units = get_units(self, editing)
            return Int(minval, maxval, step, default, units)

        elif base == 'string':
            def get_minmax(self, simple):
                def get_value(self, tag):
                    return self.get(tag, 'value', _nonnegative_int, 'non-negative integer')

                mintag = simple.find('minLength')
                maxtag = simple.find('maxLength')
                if maxtag is None:
                    return (None, None)
                min_ = (mintag is not None) and get_value(self, mintag) or 0
                max_ = get_value(self, maxtag)
                return (min_, max_)

            def get_default(self, simple):
                tag = simple.find('default')
                if tag is None:
                    return
                return self.get(tag, 'value')

            minlen, maxlen = get_minmax(self, simple)
            default = get_default(self, simple)
            return String(minlen, maxlen, default)

        else:
            self.error('expected "boolean", "integer" or "string" in "base" attribute', simple)

class XmlGenerator(object):
    def __init__(self, tu):
        self.tu = tu

    def to_file(self, filename):
        open(filename, 'w').write(self.to_string())

    def to_string(self):
        root = ET.Element('pdmeta')
        for mo in self.tu.mos:
            self.mo(root, mo)
        hdr = '''<?xml version="1.0" encoding="utf-8"?>\n'''  # lxml would put single quotes...
        body = ET.tostring(root, pretty_print=True)
        return hdr + body

    def mo(self, parent, mo):
        moelem = ET.SubElement(parent, 'managedObject', {'class': mo.name})
        if mo.doc:
            moelem.set('fullName', mo.doc)
        flags = Mo.default_flags if mo.flags is None else mo.flags
        moelem.set('hidden', ('false', 'true')[flags[0]])
        moelem.set('create', ('false', 'true')[flags[1]])
        moelem.set('update', ('false', 'true')[flags[2]])
        moelem.set('delete', ('false', 'true')[flags[3]])
        for child in mo.children:
            celem = ET.SubElement(moelem, 'childManagedObject', {'class': child.name})
            if child.max_count is not None:
                celem.set('maxOccurs', str(child.max_count))
        self.fields(moelem, mo.fields)

    def fields(self, parent, fields):
        for field in fields:
            self.field(parent, field)

    def field(self, parent, field):
        if field.cardinality.kind == CardinalityKind.REPEATED:
            if field.cardinality.max_count:
                max_occurs = field.cardinality.max_count
            else:
                max_occurs = XmlConst.default_repeated_max_occurs
        else:
            max_occurs = 1

        pelem = ET.SubElement(parent, 'p', name=field.name)
        if field.doc:
            pelem.set('fullName', field.doc)
        pelem.set('maxOccurs', str(max_occurs))

        if field.cardinality.kind == CardinalityKind.OPTIONAL:
            ET.SubElement(pelem, 'creation', {'priority': 'optional'})

        if isinstance(field.type, Struct):
            self.struct(pelem, field.type)
        elif isinstance(field.type, Enum):
            self.enum(pelem, field.type)
        else:
            self.scalar(pelem, field.type)

    def struct(self, parent, struct_):
        celem = ET.SubElement(parent, 'complexType')
        self.fields(celem, struct_.fields)

    def enum(self, parent, enum_):
        selem = ET.SubElement(parent, 'simpleType', base='integer')
        for enumer in enum_.enumerators:
            eelem = ET.SubElement(selem, 'enumeration', value=str(enumer.value))
            eelem.set('text', enumer.name)
        if enum_.default is not None:
            ET.SubElement(selem, 'default', value=str(enum_.default))

    def scalar(self, parent, type_):
        if isinstance(type_, Bool):
            selem = ET.SubElement(parent, 'simpleType', base='boolean')
            if type_.default is not None:
                ET.SubElement(selem, 'default', value=type_.defaultstr)
        elif isinstance(type_, Int):
            selem = ET.SubElement(parent, 'simpleType', base='integer')
            if type_.minval is not None or type_.units is not None:
                eelem = ET.SubElement(selem, 'editing')
                if type_.units is not None:
                    eelem.set('units', type_.units)
                if type_.minval is not None:
                    relem = ET.SubElement(eelem, 'range')
                    relem.set('minIncl', type_.minvalstr)
                    relem.set('maxIncl', type_.maxvalstr)
                    if type_.step is not None:
                        relem.set('step', type_.stepstr)
            if type_.default is not None:
                ET.SubElement(selem, 'default', value=type_.defaultstr)
        elif isinstance(type_, String):
            selem = ET.SubElement(parent, 'simpleType', base='string')
            if type_.minlen is not None:
                ET.SubElement(selem, 'minLength', value=str(type_.minlen))
                ET.SubElement(selem, 'maxLength', value=str(type_.maxlen))
                if type_.default is not None:
                    ET.SubElement(selem, 'default', value=type_.default)
        else:
            assert False, "o-oh, we shouldn't end up here"

EXIT_OK = 0
EXIT_FAILURE = 1

class DriverError(Exception):
    pass

def driver_parseopts(args):
    def readable_dir(name):
        if not os.path.isdir(name):
            raise argparse.ArgumentTypeError("%s directory not found" % name)
        return name

    def readable_file(name):
        if not os.path.isfile(name):
            raise argparse.ArgumentTypeError("%s file not found" % name)
        return name

    class ArgumentParser(argparse.ArgumentParser):
        def error(self, msg):
            raise DriverError(msg)

    parser = ArgumentParser('melina')
    parser.add_argument('input',
                        type = readable_file,
                        nargs = '?',
                        help = ('Input in melina language.'))
    parser.add_argument('--meta-out',
                        metavar = 'OUT_DIR',
                        type = readable_dir,
                        help = 'Generate meta output files in given directory.')
    parser.add_argument('--xml-out',
                        metavar = 'OUT_DIR',
                        type = readable_dir,
                        help = 'Generate xml output files in given directory.')

    stdoutgroup = parser.add_mutually_exclusive_group()
    stdoutgroup.add_argument('--meta-stdout',
                             action = 'store_true',
                             help = 'Output as meta to stdout.')
    stdoutgroup.add_argument('--xml-stdout',
                             action = 'store_true',
                             help = 'Output as xml to stdout.')

    inputgroup = parser.add_mutually_exclusive_group()
    inputgroup.add_argument('--meta',
                            action = 'store_true',
                            help = 'Parse input files as meta.')
    inputgroup.add_argument('--xml',
                            action = 'store_true',
                            help = 'Parse input files as xml.')

    opts = parser.parse_args(args=args)

    if not opts.input:
        raise DriverError('missing input')

    return opts

def driver(args=None):
    if args is None:
        args = sys.argv[1:]
    else:
        if not isinstance(args, str):
            raise Exception("not a string argument list: %s" % args)
        args = shlex.split(args)

    def get_parser_cls(filepath, opts):
        if opts.meta:
            return MetaParser
        if opts.xml:
            return XmlParser

        ext = os.path.splitext(filepath)[1]
        if ext == '.meta':
            return MetaParser
        elif ext == '.xml':
            return XmlParser

        raise DriverError("Input type was not given and cannot be deduced from extension: %s"
                          % os.path.basename(filepath))

    def make_output_filepath(input_filepath, output_dir, wanted_extension):
        stem = os.path.splitext(os.path.basename(opts.input))[0]
        basename = stem + wanted_extension
        output_filepath = os.path.join(output_dir, basename)
        return output_filepath

    try:
        opts = driver_parseopts(args=args)
        parser = get_parser_cls(opts.input, opts)
        tu = parser.from_file(opts.input).parse()
        if opts.meta_out:
            MetaGenerator(tu).to_file(make_output_filepath(opts.input, opts.meta_out, '.meta'))
        if opts.xml_out:
            XmlGenerator(tu).to_file(make_output_filepath(opts.input, opts.xml_out, '.xml'))
        if opts.meta_stdout:
            sys.stdout.write(MetaGenerator(tu).to_string())
        if opts.xml_stdout:
            sys.stdout.write(XmlGenerator(tu).to_string())
        if not (opts.meta_out or opts.xml_out or opts.meta_stdout or opts.xml_stdout):
            sys.stderr.write('Your input is beautiful! No output selected though.\n')
        return EXIT_OK
    except (DriverError, MetaParserError, XmlParserError) as e:
        pretty = getattr(e, 'prettymsg', None)
        if pretty:
            sys.stderr.write(pretty)
        else:
            sys.stderr.write('melina: error: %s\n' % e.message)
        return EXIT_FAILURE

def main(args=None):
    return driver(args=args)

if __name__ == '__main__':
    sys.exit(main())
