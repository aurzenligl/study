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

def _sanitize(obj, classes):
    assert isinstance(obj, classes), 'Object %s is not an instance of expected classes: %s' % (repr(obj), classes)
    return obj

def _sanitize_list(lst, classes):
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
    def __init__(self, name, fields, children):
        self.name = _sanitize_identifier(name)
        self.fields = _sanitize_list(fields, Field)
        self.children = _sanitize_list(children, MoChild)

    def __repr__(self):
        return '<Mo %s>' % self.name

    def __str__(self):
        return (
            'mo %s' % self.name + ':' + ''.join((' ' + x.name for x in self.children)) + '\n' +
            _indent(''.join((str(field) for field in self.fields)), 4)
        )

class MoChild(object):
    def __init__(self, name):
        self.name = _sanitize_identifier(name)

    def __repr__(self):
        return '<MoChild %s>' % self.name

    def __str__(self):
        return self.name

class Field(object):
    def __init__(self, name, type_, cardinality, doc):
        self.name = _sanitize_identifier(name)
        self.type = _sanitize(type_, (Struct, Enum, Scalar))
        self.cardinality = _sanitize(cardinality, Cardinality)
        self.doc = _sanitize(doc, (type(None), str))

    def __repr__(self):
        return '<Field %s>' % self.name

    def __str__(self):
        if isinstance(self.type, Scalar):
            text = '%s %s %s\n' % (self.cardinality, self.type, self.name)
        else:
            text = '%s %s' % (self.cardinality, self.type)
        if self.doc:
            return _add_to_1st_line(text, self.doc)
        return text

class Cardinality(object):
    def __init__(self, kind):
        self.kind = _sanitize(kind, CardinalityKind)

    def __repr__(self):
        return '<Cardinality %s>' % self.kind.name.lower()

    def __str__(self):
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
    def __init__(self, name, enumerators):
        self.name = _sanitize_identifier(name)
        self.enumerators = _sanitize_list(enumerators, Enumerator)

    def __repr__(self):
        return '<Enum %s>' % self.name

    def __str__(self):
        return (
            'enum %s\n' % self.name +
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
    pass

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
    KEYW = 0      # mo, struct, enum, repeated, optional, int, float, string
    NAME = 1      # [_a-zA-Z][_a-zA-Z0-9]*
    NUMBER = 2    # [0-9]*
    NUMNAME = 3   # [_a-zA-Z0-9]+
    LCB = 4       # {
    RCB = 5       # }
    SEMI = 6      # ;
    COMMA = 7     # ,
    ASSIGN = 8    # =
    ARROW = 9     # ->
    COMMENT = 10  # '//\n', '/**/' <ignored, stored>
    END = 11

class MetaToken(object):
    __slots__ = ('kind', 'value', 'span', 'string')

    _repr_vals = {
        MetaTokenKind.LCB: '{',
        MetaTokenKind.RCB: '}',
        MetaTokenKind.SEMI: ';',
        MetaTokenKind.COMMA: ',',
        MetaTokenKind.ASSIGN: '=',
        MetaTokenKind.ARROW: '->',
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
        r'(?P<number>[0-9]+(?![a-zA-Z0-9_]))',
        r'(?P<numname>[0-9]+[a-zA-Z_][a-zA-Z0-9_]*)',
        r'(?P<operator>(->)|[{};,=])',
        r'(?P<comment>(//.*\n|/\*(\*(?!/)|[^*])*\*/))',
        r'(?P<space>\s+)',
    )))

    _keywords = ('mo', 'struct', 'enum', 'repeated', 'optional', 'bool', 'int', 'float', 'string')

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
                elif group == 'numname':
                    return MetaToken(MetaTokenKind.NUMNAME, string, span=span)
                elif group == 'number':
                    return MetaToken(MetaTokenKind.NUMBER, int(string), span=span, string=string)
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
    pos = tok.span.start_linecol[1] - 1
    leftwards = tok.span.start_line[:pos]
    return leftwards.isspace()

def _isrightdoc(tok, symtok):
    return tok.span.end_linecol[0] == symtok.span.end_linecol[0]

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
        self.prev = self.ts.cur

    @classmethod
    def from_file(cls, filename):
        return cls(open(filename).read(), filename)

    @property
    def cur(self):
        return self.ts.cur

    def get(self):
        while True:
            self.prev = self.cur
            tok = self.ts.get()
            if tok.kind != MetaTokenKind.COMMENT:
                return tok

    def parse(self):
        mos = []
        while True:
            if self.get().kind == MetaTokenKind.END:
                break
            mos.append(self.mo())

        return TranslationUnit(mos)

    def mo(self):
        if self.cur.pair != (MetaTokenKind.KEYW, 'mo'):
            raise MetaParserError('expected keyword "mo"', self.filename, self.cur.span)

        if self.get().kind != MetaTokenKind.NAME:
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

        return Mo(name, fields, children)

    def mo_child_list(self):
        if self.get().kind != MetaTokenKind.NAME:
            raise MetaParserError('expected mo child name', self.filename, self.cur.span)
        children = [MoChild(self.cur.value)]

        while True:
            if self.get().kind != MetaTokenKind.COMMA:
                break

            if self.get().kind != MetaTokenKind.NAME:
                raise MetaParserError('expected mo child name', self.filename, self.cur.span)
            children.append(MoChild(self.cur.value))

        return children

    def field(self):
        doc = None
        if self.prev.kind == MetaTokenKind.COMMENT:
            if _isupperdoc(self.prev):
                doc = _docstring(self.prev)

        cardinality = Cardinality(CardinalityKind.REQUIRED)
        if self.cur.pair == (MetaTokenKind.KEYW, 'repeated'):
            cardinality = Cardinality(CardinalityKind.REPEATED)
            self.get()
        elif self.cur.pair == (MetaTokenKind.KEYW, 'optional'):
            cardinality = Cardinality(CardinalityKind.OPTIONAL)
            self.get()

        if not (self.cur.kind == MetaTokenKind.KEYW and self.cur.value in ('struct', 'enum', 'bool', 'int', 'string')):
            raise MetaParserError('expected field definition', self.filename, self.cur.span)

        if self.cur.pair == (MetaTokenKind.KEYW, 'struct'):
            type_ = self.struct()
            name = type_.name
        elif self.cur.pair == (MetaTokenKind.KEYW, 'enum'):
            type_ = self.enum()
            name = type_.name
        else:
            type_, name = self.scalar()

        prev = self.cur
        self.get()
        if self.prev.kind == MetaTokenKind.COMMENT:
            if _isrightdoc(self.prev, prev):
                doc = _mergedoc(doc, _docstring(self.prev))

        return Field(name, type_, cardinality, doc)

    def struct(self):
        assert self.cur.pair == (MetaTokenKind.KEYW, 'struct')

        if self.get().kind != MetaTokenKind.NAME:
            raise MetaParserError('expected struct name', self.filename, self.cur.span)

        name = self.cur.value
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

        return Struct(name, fields)

    def enum(self):
        assert self.cur.pair == (MetaTokenKind.KEYW, 'enum')

        if self.get().kind != MetaTokenKind.NAME:
            raise MetaParserError('expected enum name', self.filename, self.cur.span)

        name = self.cur.value

        if self.get().kind != MetaTokenKind.LCB:
            raise MetaParserError('expected enum definition', self.filename, self.cur.span)

        enumerators = self.enumerator_list()

        if self.cur.kind != MetaTokenKind.RCB:
            raise MetaParserError('expected brace closing enum definition', self.filename, self.cur.span)

        prev = self.cur
        if self.get().kind != MetaTokenKind.SEMI:
            raise MetaParserError('expected semicolon after enum definition', self.filename, prev.span)

        return Enum(name, enumerators)

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
            type_ = Bool()
        elif self.cur.pair == (MetaTokenKind.KEYW, 'int'):
            type_ = Int()
        elif self.cur.pair == (MetaTokenKind.KEYW, 'string'):
            type_ = String()
        else:
            assert False, "o-oh, we shouldn't end up here"

        if self.get().kind != MetaTokenKind.NAME:
            raise MetaParserError('expected scalar name', self.filename, self.cur.span)

        name = self.cur.value

        prev = self.cur
        if self.get().kind != MetaTokenKind.SEMI:
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
            out += ' -> ' + ', '.join((child.name for child in mo.children))
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
        if field.cardinality.kind != CardinalityKind.REQUIRED:
            out += field.cardinality.kind.name.lower() + ' '
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

    def ensured_getattr(self, elem, name):
        value = elem.get(name)
        if not value:
            self.error('expected "%s" attribute in "%s" tag' % (name, elem.tag), elem)
        return value

    def mo(self, mo):
        '''TODO [langfeature] add fullName comment to mo'''
        '''TODO [langfeature] add hidden/create/update/delete flags to mo'''

        name = self.ensured_getattr(mo, 'class')
        children = self.mo_child_list(mo)
        fields = [self.field(field) for field in mo.findall('p')]
        return Mo(name, fields, children)

    def mo_child_list(self, mo):
        '''TODO [langfeature] add maxOccurs="1" to mo children'''

        children = []
        for child in mo.findall('childManagedObject'):
            '''TODO [langfeature] maxOccurs in children'''
            name = self.ensured_getattr(child, 'class')
            children.append(MoChild(name))
        return children

    def field(self, field):
        '''TODO [langfeature] add fullName comment to field'''

        '''TODO [langfeature] field name has to begin with small letter (xml)'''
        '''TODO [langfeature] enum/struct name has to begin with capital letter (meta)'''
        name = self.ensured_getattr(field, 'name')

        '''TODO [langfeature] add max_occurs value to repeated cardinality'''
        max_occurs = field.get('maxOccurs')
        if max_occurs is not None:
            max_occurs = _positive_int(max_occurs)
            if max_occurs is None:
                '''TODO [langfeature] field does not need to have maxOccurs attrib, in which case it's considered required'''
                self.error('expected positive integer in "maxOccurs"', field)

        if max_occurs == 1 or max_occurs == None:
            creation = field.find('creation')
            if creation is not None:
                prio = self.ensured_getattr(creation, 'priority')
                if prio == 'optional':
                    cardinality = Cardinality(CardinalityKind.OPTIONAL)
                elif prio == 'mandatory':
                    cardinality = Cardinality(CardinalityKind.REQUIRED)
                else:
                    self.error('expected "optional" or "mandatory" cardinality', creation)
            else:
                cardinality = Cardinality(CardinalityKind.REQUIRED)
        else:
            cardinality = Cardinality(CardinalityKind.REPEATED)

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
            self.error('expected "simpleType" or "complexType" tag under "p" tag', field)

        return Field(name, type_, cardinality, None)

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
            name = self.ensured_getattr(enumer, 'text')
            name = name.replace('-', '_')
            value = _int(self.ensured_getattr(enumer, 'value'))
            if value is None:
                self.error('expected integer enumerator value', enumer)
            enumerators.append(Enumerator(name, value))
        return enumerators

    def scalar(self, simple):
        '''TODO [langfeature] add decimal scalar base (fixed-point values)'''

        base = self.ensured_getattr(simple, 'base')

        if base == 'boolean':
            return Bool()
        elif base in ('integer', 'decimal'):
            editing = simple.find('editing')
            if editing is not None:
                '''TODO [langfeature] add integer range/step'''
                '''TODO [langfeature] is 'integer' and 'decimal' the same?'''
                '''TODO [langfeature] add how does step and divisor differ?'''
                '''TODO [langfeature] does internalValue always follow divisor/step?'''
                range = editing.find('range')
                if range is not None:
                    min_val = _float(self.ensured_getattr(range, 'minIncl'))
                    if min_val is None:
                        self.error('expected float in "minIncl"', range)
                    max_val = _float(self.ensured_getattr(range, 'maxIncl'))
                    if max_val is None:
                        self.error('expected float in "maxIncl"', range)

                units = editing.get('units')
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
                min_val = _nonnegative_int(self.ensured_getattr(min_, 'value'))
                if min_val is None:
                    self.error('expected non-negative integer in "value" attribute', min_)
                max_val = _nonnegative_int(self.ensured_getattr(max_, 'value'))
                if max_val is None:
                    self.error('expected non-negative integer in "value" attribute', max_)
                '''TODO [langfeature] add min/max string lengths'''

                return String()
            else:
                return String()
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
        for child in mo.children:
            ET.SubElement(moelem, 'childManagedObject', {'class': child.name})
        self.fields(moelem, mo.fields)

    def fields(self, parent, fields):
        for field in fields:
            self.field(parent, field)

    def field(self, parent, field):
        if field.cardinality.kind != CardinalityKind.REPEATED:
            max_occurs = '1'
        else:
            max_occurs = '999999'
        pelem = ET.SubElement(parent, 'p', name=field.name)
        pelem.set('maxOccurs', max_occurs)

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

    def scalar(self, parent, type_):
        if isinstance(type_, Bool):
            ET.SubElement(parent, 'simpleType', base='boolean')
        elif isinstance(type_, Int):
            ET.SubElement(parent, 'simpleType', base='integer')
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
