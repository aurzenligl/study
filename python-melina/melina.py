#!/usr/bin/env python

import os
import sys
import shlex
import argparse
import enum
import re
import io
import lxml.etree as ET

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

class TranslationUnit(object):
    def __init__(self, mos):
        self.mos = mos

    def __repr__(self):
        return '<TranslationUnit with %s mos>' % len(self.mos)

    def __str__(self):
        return ''.join((str(mo) for mo in self.mos))

class Mo(object):
    def __init__(self, name, fields, children):
        self.name = name
        self.fields = fields
        self.children = children

    def __repr__(self):
        return '<Mo %s>' % self.name

    def __str__(self):
        return (
            'mo %s' % self.name + ':' + ''.join((' ' + x for x in self.children)) + '\n' +
            _indent(''.join((str(field) for field in self.fields)), 4)
        )

class Field(object):
    def __init__(self, name, type_, cardinality):
        self.name = name
        self.type = type_
        self.cardinality = cardinality

    def __repr__(self):
        return '<Field %s>' % self.name

    def __str__(self):
        if isinstance(self.type, Scalar):
            return '%s %s %s\n' % (self.cardinality, self.type, self.name)
        else:
            return '%s %s' % (self.cardinality, self.type)

class Struct(object):
    def __init__(self, name, fields):
        self.name = name
        self.fields = fields

    def __repr__(self):
        return '<Struct %s>' % self.name

    def __str__(self):
        return (
            'struct %s\n' % self.name +
            _indent(''.join((str(field) for field in self.fields)), 4)
        )

class Enum(object):
    def __init__(self, name, enumerators):
        self.name = name
        self.enumerators = enumerators

    def __repr__(self):
        return '<Enum %s>' % self.name

    def __str__(self):
        return (
            'enum %s\n' % self.name +
            _indent(''.join((str(enumer) for enumer in self.enumerators)), 4)
        )

class Enumerator(object):
    def __init__(self, name, value):
        self.name = name
        self.value = value

    def __repr__(self):
        return '<Enumerator %s>' % self.name

    def __str__(self):
        return '%s = %s\n' % (self.name, self.value)

class Scalar(object):

    def __repr__(self):
        return '<%s>' % self.__class__.__name__

    def __str__(self):
        return '%s' % self.__class__.__name__.lower()

class Int(Scalar):
    pass

class String(Scalar):
    pass

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
    KEYW = 0    # mo, struct, enum, repeated, optional, int, float, string
    NAME = 1    # [_a-zA-Z][_a-zA-Z0-9]*
    NUMBER = 2  # [0-9]*
    LCB = 3     # {
    RCB = 4     # }
    SEMI = 5    # ;
    COMMA = 6   # ,
    ASSIGN = 7  # =
    ARROW = 8   # ->
    COMMENT = 9 # '//\n', '/**/' <ignored, stored>
    END = 10

class MetaToken(object):
    __slots__ = ('kind', 'value', 'span')

    _repr_vals = {
        MetaTokenKind.LCB: '{',
        MetaTokenKind.RCB: '}',
        MetaTokenKind.SEMI: ';',
        MetaTokenKind.COMMA: ',',
        MetaTokenKind.ASSIGN: '=',
        MetaTokenKind.ARROW: '->',
    }

    _repr_direct = (MetaTokenKind.KEYW, MetaTokenKind.NAME, MetaTokenKind.NUMBER, MetaTokenKind.COMMENT)

    def __init__(self, kind, value = None, span = None):
        self.kind = kind
        self.value = value
        self.span = span

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
        r'(?P<number>[0-9]+)',
        r'(?P<operator>(->)|[{};,=])',
        r'(?P<comment>(//.*\n|/\*(\*(?!/)|[^*])*\*/))',
        r'(?P<space>\s+)',
    )))

    _keywords = ('mo', 'struct', 'enum', 'repeated', 'optional', 'int', 'float', 'string')

    _operators = {
        '->': MetaTokenKind.ARROW,
        '{': MetaTokenKind.LCB,
        '}': MetaTokenKind.RCB,
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
                elif group == 'number':
                    return MetaToken(MetaTokenKind.NUMBER, int(string), span=span)
                elif group == 'operator':
                    return MetaToken(self._operators[string], span=span)
                elif group == 'comment':
                    self.comments.append(MetaToken(MetaTokenKind.COMMENT, string, span=span))
                    continue
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

    @classmethod
    def from_file(cls, filename):
        return cls(open(filename).read(), filename)

    def parse(self):
        ts = self.ts

        mos = []
        while True:
            if ts.get().kind == MetaTokenKind.END:
                break
            mos.append(self.mo())

        return TranslationUnit(mos)

    def mo(self):
        ts = self.ts
        if ts.cur.pair != (MetaTokenKind.KEYW, 'mo'):
            raise MetaParserError('expected keyword "mo"', self.filename, ts.cur.span)

        if ts.get().kind != MetaTokenKind.NAME:
            raise MetaParserError('expected mo name', self.filename, ts.cur.span)

        name = ts.cur.value
        children = []
        fields = []

        if ts.get().kind == MetaTokenKind.ARROW:
            children = self.mo_child_list()

        if ts.cur.kind != MetaTokenKind.LCB:
            raise MetaParserError('expected mo definition', self.filename, ts.cur.span)

        while True:
            if ts.get().kind == MetaTokenKind.RCB:
                break
            fields.append(self.field())

        prev = ts.cur
        if ts.get().kind != MetaTokenKind.SEMI:
            raise MetaParserError('expected semicolon after mo definition', self.filename, prev.span)

        return Mo(name, fields, children)

    def mo_child_list(self):
        ts = self.ts

        if ts.get().kind != MetaTokenKind.NAME:
            raise MetaParserError('expected mo child name', self.filename, ts.cur.span)
        children = [ts.cur.value]

        while True:
            if ts.get().kind != MetaTokenKind.COMMA:
                break

            if ts.get().kind != MetaTokenKind.NAME:
                raise MetaParserError('expected mo child name', self.filename, ts.cur.span)
            children.append(ts.cur.value)

        return children

    def field(self):
        ts = self.ts

        cardinality = 'required'
        if ts.cur.pair == (MetaTokenKind.KEYW, 'repeated'):
            cardinality = 'repeated'
            ts.get()
        elif ts.cur.pair == (MetaTokenKind.KEYW, 'optional'):
            cardinality = 'optional'
            ts.get()

        if not (ts.cur.kind == MetaTokenKind.KEYW and ts.cur.value in ('struct', 'enum', 'int', 'string')):
            raise MetaParserError('expected field definition', self.filename, ts.cur.span)

        if ts.cur.pair == (MetaTokenKind.KEYW, 'struct'):
            type_ = self.struct()
            name = type_.name
        elif ts.cur.pair == (MetaTokenKind.KEYW, 'enum'):
            type_ = self.enum()
            name = type_.name
        else:
            type_, name = self.scalar()

        return Field(name, type_, cardinality)

    def struct(self):
        ts = self.ts

        assert ts.cur.pair == (MetaTokenKind.KEYW, 'struct')

        if ts.get().kind != MetaTokenKind.NAME:
            raise MetaParserError('expected struct name', self.filename, ts.cur.span)

        name = ts.cur.value
        fields = []

        if ts.get().kind != MetaTokenKind.LCB:
            raise MetaParserError('expected struct definition', self.filename, ts.cur.span)

        while True:
            if ts.get().kind == MetaTokenKind.RCB:
                break
            fields.append(self.field())

        prev = ts.cur
        if ts.get().kind != MetaTokenKind.SEMI:
            raise MetaParserError('expected semicolon after struct definition', self.filename, prev.span)

        return Struct(name, fields)

    def enum(self):
        ts = self.ts

        assert ts.cur.pair == (MetaTokenKind.KEYW, 'enum')

        if ts.get().kind != MetaTokenKind.NAME:
            raise MetaParserError('expected enum name', self.filename, ts.cur.span)

        name = ts.cur.value

        if ts.get().kind != MetaTokenKind.LCB:
            raise MetaParserError('expected enum definition', self.filename, ts.cur.span)

        enumerators = self.enumerator_list()

        if ts.cur.kind != MetaTokenKind.RCB:
            raise MetaParserError('expected brace closing enum definition', self.filename, ts.cur.span)

        prev = ts.cur
        if ts.get().kind != MetaTokenKind.SEMI:
            raise MetaParserError('expected semicolon after enum definition', self.filename, prev.span)

        return Enum(name, enumerators)

    def enumerator_list(self):
        ts = self.ts

        enumerators = []
        value = 0

        while True:
            if ts.get().kind != MetaTokenKind.NAME:
                break

            name = ts.cur.value

            if ts.get().kind == MetaTokenKind.ASSIGN:
                if ts.get().kind != MetaTokenKind.NUMBER:
                    raise MetaParserError('expected enumerator value', self.filename, ts.cur.span)
                value = ts.cur.value
                ts.get()

            enumerators.append(Enumerator(name, value))
            value += 1

            if ts.cur.kind != MetaTokenKind.COMMA:
                break

        return enumerators

    def scalar(self):
        ts = self.ts

        if ts.cur.pair == (MetaTokenKind.KEYW, 'int'):
            type_ = Int()
        elif ts.cur.pair == (MetaTokenKind.KEYW, 'string'):
            type_ = String()
        else:
            assert False, "o-oh, we shouldn't end up here"

        if ts.get().kind != MetaTokenKind.NAME:
            raise MetaParserError('expected scalar name', self.filename, ts.cur.span)

        name = ts.cur.value

        prev = ts.cur
        if ts.get().kind != MetaTokenKind.SEMI:
            raise MetaParserError('expected semicolon closing field definition', self.filename, prev.span)

        return type_, name

class MetaGenerator(object):
    def __init__(self, tu):
        self.tu = tu

    def to_file(self, filename):
        open(filename, 'w').write(self.to_string())

    def to_string(self):
        return '\n'.join(self.mo(mo) for mo in self.tu.mos)

    def mo(self, mo):
        out = 'mo %s' % mo.name
        if mo.children:
            out += ' -> ' + ', '.join(mo.children)
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
        if field.cardinality != 'required':
            out += field.cardinality + ' '
        if isinstance(field.type, Struct):
            out += self.struct(field.type)
        elif isinstance(field.type, Enum):
            out += self.enum(field.type)
        else:
            out += self.scalar(field.type, field.name)
        return out

    def struct(self, struct_):
        out = 'struct %s\n{\n' % struct_.name
        out += _indent(self.fields(struct_.fields), 4) + '};\n'
        return out

    def enum(self, enum_):
        out = 'enum %s\n{\n' % enum_.name
        out += _indent(',\n'.join(('%s = %s' % (er.name, er.value) for er in enum_.enumerators)), 4)
        if enum_.enumerators:
            out += '\n'
        out += '};\n'
        return out

    def scalar(self, type_, name):
        return '%s %s;\n' % (type_, name)

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

def _float(text):
    try:
        return float(text)
    except (ValueError, TypeError):
        return

class XmlParserError(Exception):
    pass

class XmlParser(object):
    def __init__(self, input_):
        '''TODO handle xml parser errors in driver'''
        '''TODO [langfeature] pdmeta version=int.int,
           header domain='str' product='str' release='str' version='str' revision='str' '''
        self.et = ET.fromstring(input_)

    @classmethod
    def from_file(cls, filename):
        return cls(open(filename).read())

    def parse(self):
        mos = [self.mo(mo) for mo in self.et.findall('.//managedObject')]
        return TranslationUnit(mos)

    def mo(self, mo):
        '''TODO [langfeature] add fullName comment to mo'''
        '''TODO [langfeature] add hidden/create/update/delete flags to mo'''

        name = mo.attrib.get('class')
        if not name:
            raise XmlParserError('mo definition has no name')
        children = self.mo_child_list(mo)
        fields = [self.field(field) for field in mo.findall('p')]
        return Mo(name, fields, children)

    def mo_child_list(self, mo):
        '''TODO [langfeature] add maxOccurs="1" to mo children'''

        children = []
        for child in mo.findall('childManagedObject'):
            name = child.attrib.get('class')
            '''TODO [langfeature] maxOccurs in children'''
            if not name:
                raise XmlParserError('mo child definition has no name')
            children.append(name)
        return children

    def field(self, field):
        '''TODO [langfeature] add fullName comment to field'''

        '''TODO [langfeature] field name has to begin with small letter (xml)'''
        '''TODO [langfeature] enum/struct name has to begin with capital letter (meta)'''
        name = field.attrib.get('name')
        if not name:
            raise XmlParserError('field declaration has no name')

        '''TODO [langfeature] add max_occurs value to repeated cardinality'''
        max_occurs = _positive_int(field.attrib.get('maxOccurs'))
        if not max_occurs:
            '''TODO [langfeature] field does not need to have maxOccurs attrib, in which case it's considered required'''
            raise XmlParserError('field declaration has no cardinality ("maxOccurs" attribute)')

        if max_occurs == 1:
            creation = field.find('creation')
            if creation is not None:
                prio = creation.attrib.get('priority')
                if prio != 'optional':
                    raise XmlParserError('priority tag present, but no optional attribute found')
                cardinality = 'optional'
            else:
                cardinality = 'required'
        else:
            cardinality = 'repeated'

        complex_ = field.find('complexType')
        simple = field.find('simpleType')
        if complex_ is not None:
            type_ = self.struct(complex_, name)
        elif simple is not None:
            if simple.find('enumeration') is not None:
                type_ = self.enum(simple, name)
            else:
                type_ = self.scalar(simple)
        else:
            raise XmlParserError('field definition not found')

        return Field(name, type_, cardinality)

    def struct(self, complex_, name):
        fields = [self.field(field) for field in complex_.findall('p')]
        return Struct(name, fields)

    def enum(self, simple, name):
        enumerators = self.enumerator_list(simple)
        '''TODO [langfeature] add default enum value'''
        return Enum(name, enumerators)

    def enumerator_list(self, simple):
        enumerators = []
        for enumer in simple.findall('enumeration'):
            name = enumer.attrib.get('text')
            if name is None:
                raise XmlParserError('enumerator name not found')
            value = _int(enumer.attrib.get('value'))
            if value is None:
                raise XmlParserError('enumerator value not found')
            enumerators.append(Enumerator(name, value))
        return enumerators

    def scalar(self, simple):
        '''TODO [langfeature] add decimal scalar base (fixed-point values)'''
        '''TODO [langfeature] add boolean scalar base'''

        base = simple.attrib.get('base')
        if not base:
            raise XmlParserError('scalar field declaration has no base')

        if base == 'integer':
            editing = simple.find('editing')
            if editing is not None:
                '''TODO [langfeature] add integer range/step'''
                range = editing.find('range')
                if range is not None:
                    min_val = _float(range.attrib.get('minIncl'))
                    if min_val is None:
                        raise XmlParserError('scalar range min val is not a float')
                    max_val = _float(range.attrib.get('maxIncl'))
                    if max_val is None:
                        raise XmlParserError('scalar range max val is not a float')

                units = editing.attrib.get('units')
                '''TODO [langfeature] add integer units'''

                default = editing.find('default')
                '''TODO [langfeature] add default integer value'''

                return Int()
            else:
                return Int()
        elif base == 'string':
            min_ = simple.find('minLength')
            max_ = simple.find('maxLength')
            if min_ is not None and max_ is not None:
                min_val = _nonnegative_int(min_.attrib.get('value'))
                if min_val is None:
                    raise XmlParserError('min val is not a positive integer')
                max_val = _nonnegative_int(max_.attrib.get('value'))
                if max_val is None:
                    raise XmlParserError('max val is not a positive integer')
                '''TODO [langfeature] add min/max string lengths'''

                return String()
            else:
                return String()
        else:
            raise XmlParserError('scalar field base unknown: %s' % base)

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
        for child in mo.children:
            ET.SubElement(moelem, 'childManagedObject', {'class': child})
        self.fields(moelem, mo.fields)

    def fields(self, parent, fields):
        for field in fields:
            self.field(parent, field)

    def field(self, parent, field):
        if field.cardinality != 'repeated':
            max_occurs = '1'
        else:
            max_occurs = '999999'
        pelem = ET.SubElement(parent, 'p', name=field.name)
        pelem.set('maxOccurs', max_occurs)

        if field.cardinality == 'optional':
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

    def scalar(self, parent, type_):
        if isinstance(type_, Int):
            selem = ET.SubElement(parent, 'simpleType', base='integer')
        elif isinstance(type_, String):
            selem = ET.SubElement(parent, 'simpleType', base='string')
            ET.SubElement(selem, 'minLength', value='0')
            ET.SubElement(selem, 'maxLength', value='2147483647')
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

    group = parser.add_mutually_exclusive_group()
    group.add_argument('--meta',
                       action = 'store_true',
                       help = 'Parse input files as xml.')
    group.add_argument('--xml',
                       action = 'store_true',
                       help = 'Parse input files as meta.')

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

    '''TODO add --meta-stdout, --xml-stdout options'''

    try:
        opts = driver_parseopts(args=args)
        parser = get_parser_cls(opts.input, opts)
        tu = parser.from_file(opts.input).parse()
        if opts.meta_out:
            MetaGenerator(tu).to_file(make_output_filepath(opts.input, opts.meta_out, '.meta'))
        if opts.xml_out:
            XmlGenerator(tu).to_file(make_output_filepath(opts.input, opts.xml_out, '.xml'))
        if not opts.meta_out and not opts.xml_out:
            sys.stderr.write('Your input is beautiful! No output selected though.\n')
        return EXIT_OK
    except (DriverError, MetaParserError, XmlParserError) as e:
        '''TODO output offending line and token in case of parsing errors'''
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
