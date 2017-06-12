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
        r'(?P<number>[1-9][0-9]*)',
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
        if ts.cur != (TokenKind.KEYW, 'mo'):
            raise ParserError('expected mo keyword, but got none')

        ts.get()
        if ts.cur.kind != TokenKind.NAME:
            raise ParserError('expected mo name, but got none')

        name = ts.cur.value
        children = []
        fields = []

        ts.get()
        if ts.cur.kind == TokenKind.ARROW:
            children = self.mo_children()

        if ts.cur.kind != TokenKind.LCB:
            raise ParserError('expected mo definition, but got none')

        while True:
            ts.get()
            if ts.cur.kind == TokenKind.RCB:
                break
            fields.append(self.field())

        return Mo(name, children, fields)

    def mo_children(self):
        pass
        '''TODO implement'''

    def field(self):
        ts = self.ts

        cardinality = 'required'
        if ts.cur == (TokenKind.KEYW, 'repeated'):
            cardinality = 'repeated'
            ts.get()
        elif ts.cur == (TokenKind.KEYW, 'optional'):
            cardinality = 'optional'
            ts.get()

        if ts.cur == (TokenKind.KEYW, 'struct'):
            type_ = self.struct()
            name = type_.name
        elif ts.cur == (TokenKind.KEYW, 'enum'):
            type_ = self.enum()
            name = type_.name
        elif ts.cur == (TokenKind.KEYW, 'scalar'):
            type_, name = self.scalar()

        return Field(name, cardinality, type_)

    def struct(self):
        pass
        '''TODO implement'''

    def enum(self):
        pass
        '''TODO implement'''

    def scalar(self):
        pass
        '''TODO implement'''

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
'''

class TranslationUnit(object):
    pass
    '''TODO implement'''

class Mo(object):
    pass
    '''TODO implement'''

class Field(object):
    pass
    '''TODO implement'''

class Struct(object):
    pass
    '''TODO implement'''

class Enum(object):
    pass
    '''TODO implement'''

def parse_options():
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

    opts = parser.parse_args()
    if not opts.input:
        parser.print_help()
        sys.exit()
    return opts

def get_tokens(tokenizer):
    toks = []
    while True:
        tok = tokenizer.get()
        toks.append(tok)
        if tok.kind == TokenKind.END:
            return toks

def main():
    opts = parse_options()
    input = open(opts.input)
    ts = Tokenizer(input)

    for tok in get_tokens(ts):
        print tok
    for com in ts.comments:
        print com

    print "ok"

if __name__ == '__main__':
    main()
