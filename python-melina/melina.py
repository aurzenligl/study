#!/usr/bin/env python

import os
import sys
import argparse
import enum
import re

def _line(text, pos):
    return text.count('\n', 0, pos) + 1

def _col(text, pos):
    return pos - text.rfind('\n', 0, pos)

class Span(object):
    __slots__ = ('input', 'start', 'end')

    def __init__(self, input, start, end):
        self.input = input
        self.start = start
        self.end = end

    @property
    def start_linecol(self):
        return (_line(self.input, self.start), _col(self.input, self.start))

    @property
    def end_linecol(self):
        return (_line(self.input, self.end), _col(self.input, self.end))

    def __repr__(self):
        return '%s:%s-%s:%s' % (self.start_linecol + self.end_linecol)

class TokenKind(enum.Enum):
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

class Token(object):
    '''TODO __eq__ for tuple (kind, value)'''

    __slots__ = ('kind', 'value', 'span')

    _repr_vals = {
        TokenKind.LCB: '{',
        TokenKind.RCB: '}',
        TokenKind.SEMI: ';',
        TokenKind.COMMA: ',',
        TokenKind.ASSIGN: '=',
        TokenKind.ARROW: '->',
    }

    _repr_direct = (TokenKind.KEYW, TokenKind.NAME, TokenKind.NUMBER, TokenKind.COMMENT)

    def __init__(self, kind, value = None, span = None):
        self.kind = kind
        self.value = value
        self.span = span

    @property
    def pair(self):
        return (self.kind, self.value)

    def __repr__(self):
        if self.span:
            repr = "<Token %s %s" % (self.span, self.kind.name)
        else:
            repr = "<Token %s" % self.kind.name

        if self.kind in self._repr_direct:
            return repr + " %s>" % self.value
        reprval = self._repr_vals.get(self.kind)
        if reprval:
            return repr + " %s>" % reprval
        return repr + ">"

class TokenizerError(Exception):
    pass

class Tokenizer(object):
    def __init__(self, input):
        self.input = input
        self.pos = 0
        self.token = Token(TokenKind.END)
        self.comments = []

    @classmethod
    def from_file(cls, filename):
        return Tokenizer(open(filename).read())

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
        '->': TokenKind.ARROW,
        '{': TokenKind.LCB,
        '}': TokenKind.RCB,
        ';': TokenKind.SEMI,
        ',': TokenKind.COMMA,
        '=': TokenKind.ASSIGN,
    }

    def _get(self):
        while True:
            start = self.pos
            pack = self._read_raw()
            if pack:
                span = Span(self.input, start, self.pos - 1)
                string, group = pack
                if group == 'name':
                    if string in self._keywords:
                        return Token(TokenKind.KEYW, string, span=span)
                    else:
                        return Token(TokenKind.NAME, string, span=span)
                elif group == 'number':
                    return Token(TokenKind.NUMBER, int(string), span=span)
                elif group == 'operator':
                    return Token(self._operators[string], span=span)
                elif group == 'comment':
                    self.comments.append(Token(TokenKind.COMMENT, string, span=span))
                    continue
                elif group == 'space':
                    continue
                assert False, "o-oh, we shouldn't end up here"

            if self.pos == len(self.input):
                return Token(TokenKind.END)
            else:
                ch = self.input[self.pos]
                raise TokenizerError("unexpected character: '%s', ord=%s" % (ch, ord(ch)))

    def _read_raw(self):
        match = self._sre.match(self.input, self.pos)
        if match:
            self.pos = match.end()
            return match.group(), match.lastgroup

class ParserError(Exception):
    pass

class Parser(object):
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
        FLOAT
        STRING
    '''

    def __init__(self, input):
        self.ts = Tokenizer(input)

    @classmethod
    def from_file(cls, filename):
        return cls(open(filename).read())

    def parse(self):
        ts = self.ts

        mos = []
        while True:
            ts.get()
            if ts.cur.kind == TokenKind.END:
                break
            mos.append(self.mo())

        return TranslationUnit(mos)

    def mo(self):
        ts = self.ts
        if ts.cur.pair != (TokenKind.KEYW, 'mo'):
            raise ParserError('expected keyword "mo", but got none')

        ts.get()
        if ts.cur.kind != TokenKind.NAME:
            raise ParserError('expected mo name, but got none')

        name = ts.cur.value
        children = []
        fields = []

        ts.get()
        if ts.cur.kind == TokenKind.ARROW:
            children = self.mo_child_list()

        if ts.cur.kind != TokenKind.LCB:
            raise ParserError('expected mo definition, but got none')

        while True:
            ts.get()
            if ts.cur.kind == TokenKind.RCB:
                break
            fields.append(self.field())

        ts.get()
        if ts.cur.kind != TokenKind.SEMI:
            raise ParserError('expected semicolon after mo definition, but got none')

        return Mo(name, fields, children)

    def mo_child_list(self):
        ts = self.ts

        ts.get()
        if ts.cur.kind != TokenKind.NAME:
            raise ParserError('expected mo child name, but got none')
        children = [ts.cur.value]

        while True:
            ts.get()
            if ts.cur.kind != TokenKind.COMMA:
                break

            ts.get()
            if ts.cur.kind != TokenKind.NAME:
                raise ParserError('expected mo child name, but got none')
            children.append(ts.cur.value)

        return children

    def field(self):
        ts = self.ts

        cardinality = 'required'
        if ts.cur.pair == (TokenKind.KEYW, 'repeated'):
            cardinality = 'repeated'
            ts.get()
        elif ts.cur.pair == (TokenKind.KEYW, 'optional'):
            cardinality = 'optional'
            ts.get()

        if not (ts.cur.kind == TokenKind.KEYW and ts.cur.value in ('struct', 'enum', 'int', 'float', 'string')):
            raise ParserError('expected "struct", "enum" or type name "int", "float", "string", but got none')

        if ts.cur.pair == (TokenKind.KEYW, 'struct'):
            type_ = self.struct()
            name = type_.name
        elif ts.cur.pair == (TokenKind.KEYW, 'enum'):
            type_ = self.enum()
            name = type_.name
        else:
            type_, name = self.scalar()

        return Field(name, type_, cardinality)

    def struct(self):
        ts = self.ts

        if ts.cur.pair != (TokenKind.KEYW, 'struct'):
            raise ParserError('expected keyword "struct", but got none')

        ts.get()
        if ts.cur.kind != TokenKind.NAME:
            raise ParserError('expected struct name, but got none')

        name = ts.cur.value
        fields = []

        ts.get()
        if ts.cur.kind != TokenKind.LCB:
            raise ParserError('expected struct definition, but got none')

        while True:
            ts.get()
            if ts.cur.kind == TokenKind.RCB:
                break
            fields.append(self.field())

        ts.get()
        if ts.cur.kind != TokenKind.SEMI:
            raise ParserError('expected semicolon after struct definition, but got none')

        return Struct(name, fields)

    def enum(self):
        ts = self.ts

        if ts.cur.pair != (TokenKind.KEYW, 'enum'):
            raise ParserError('expected keyword "enum", but got none')

        ts.get()
        if ts.cur.kind != TokenKind.NAME:
            raise ParserError('expected enum name, but got none')

        name = ts.cur.value

        ts.get()
        if ts.cur.kind != TokenKind.LCB:
            raise ParserError('expected enum definition, but got none')

        enumerators = self.enumerator_list()

        if ts.cur.kind != TokenKind.RCB:
            raise ParserError('expected enum definition, but got none')

        ts.get()
        if ts.cur.kind != TokenKind.SEMI:
            raise ParserError('expected semicolon after enum definition, but got none')

        return Enum(name, enumerators)

    def enumerator_list(self):
        ts = self.ts

        enumerators = []
        value = 0

        while True:
            ts.get()
            if ts.cur.kind != TokenKind.NAME:
                break

            name = ts.cur.value

            ts.get()
            if ts.cur.kind == TokenKind.ASSIGN:
                ts.get()
                if ts.cur.kind != TokenKind.NUMBER:
                    raise ParserError('expected enumerator value, but got none')
                value = ts.cur.value
                ts.get()

            enumerators.append(Enumerator(name, value))
            value += 1

            if ts.cur.kind != TokenKind.COMMA:
                break

        return enumerators

    def scalar(self):
        ts = self.ts

        if ts.cur.pair == (TokenKind.KEYW, 'int'):
            type_ = Int()
        elif ts.cur.pair == (TokenKind.KEYW, 'float'):
            type_ = Float()
        elif ts.cur.pair == (TokenKind.KEYW, 'string'):
            type_ = String()
        else:
            raise ParserError('expected type name "int", "float", "string", but got none')

        ts.get()
        if ts.cur.kind != TokenKind.NAME:
            raise ParserError('expected field name, but got none')

        name = ts.cur.value

        ts.get()
        if ts.cur.kind != TokenKind.SEMI:
            raise ParserError('expected semicolon closing field definition, but got none')

        return type_, name

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
            'mo %s\n' % self.name +
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

class Float(Scalar):
    pass

class String(Scalar):
    pass

class Generator(object):
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
            out += '%s %s;\n' % (field.type, field.name)
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

def parse_options():
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
            sys.exit('melina' + ': error: ' + msg)

    parser = ArgumentParser('melina')
    parser.add_argument('input',
                        type = readable_file,
                        nargs = '?',
                        help = ('Input in melina language.'))
    parser.add_argument('--meta-out',
                        metavar = 'OUT_DIR',
                        type = readable_dir,
                        help = 'Generate C++ simple POD-based codec header and source files.')

    opts = parser.parse_args()
    if not opts.input:
        parser.print_help()
        sys.exit()
    return opts

class DriverError(Exception):
    pass

def main():
    '''TODO: add driver tests'''
    '''TODO: add driver error printing'''

    def make_meta_name(opts):
        return opts.meta_out + '/' + os.path.splitext(os.path.basename(opts.input))[0] + '.meta'
    def check_for_overwrite(opts):
        if os.path.abspath(opts.input) == os.path.abspath(make_meta_name(opts)):
            raise DriverError('file "%s" would be overwritten' % opts.input)

    opts = parse_options()
    check_for_overwrite(opts)
    tu = Parser.from_file(opts.input).parse()
    Generator(tu).to_file(make_meta_name(opts))

if __name__ == '__main__':
    main()
