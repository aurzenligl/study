#!/usr/bin/env python

import os
import sys
import argparse
import enum
import re

class Location(object):
    __slots__ = ('pos', 'line', 'col')

    def __init__(self):
        self.pos = 0
        self.line = 1
        self.col = 0

    def progress(self, string):
        slen = len(string)
        nlcount = string.count('\n')
        self.pos += slen
        self.line += nlcount
        if nlcount:
            self.col = slen - string.rfind('\n') - 1
        else:
            self.col += slen

class Span(object):
    __slots__ = 'span'

    def __init__(self, span):
        self.span = span

    def __repr__(self):
        return '%s:%s-%s:%s' % (self.span[0][0], self.span[0][1], self.span[1][0], self.span[1][1])

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
        if self.locs:
            repr = "<Token %s %s" % (self.span, self.kind.name)
        else:
            repr = "<Token %s" % self.kind.name

        if self.kind in self._repr_direct:
            return repr + " %s>" % self.value
        reprval = self._repr_vals.get(self.kind)
        if reprval:
            return repr + " %s>" % reprval
        return repr + ">"

_mega_is = re.compile('|'.join((
    r'(?P<name>[a-zA-Z_][a-zA-Z0-9_]*)',
    r'(?P<number>[1-9][0-9]*)',
    r'(?P<operator>(->)|[{};,=])',
    r'(?P<comment>(//.*\n|/\*(\*(?!/)|[^*])*\*/))',
    r'(?P<space>\s+)',
)))

class TokenizerInput(object):
    def __init__(self, input):
        self.input = input.read()
        self.loc = Location()

    def is_end(self):
        return self.loc.pos == len(self.input)

    def peek(self):
        return not self.is_end() and self.input[self.loc.pos] or ''

    def read_re(self, re):
        match = re.match(self.input, self.loc.pos)
        if match:
            string = match.group()
            '''TODO use match.end() to move pos and handle newlines in some clever way'''
            self.loc.progress(string)
            return string, match.lastgroup

class TokenizerError(Exception):
    pass

class Tokenizer(object):
    def __init__(self, input):
        self.input = TokenizerInput(input)
        self.token = Token(TokenKind.END)
        self.comments = []

    @classmethod
    def from_file(cls, filename):
        return Tokenizer(open(filename))

    def get(self):
        self.token = token = self._get()
        return token

    @property
    def cur(self):
        return self.token

    @property
    def _loc(self):
        return (self.input.loc.line, self.input.loc.col)

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
        '''TODO syntax sugar location passing'''

        while True:
            loc = (self.input.loc.line, self.input.loc.col + 1)

            pack = self.input.read_re(_mega_is)
            if pack:
                string, group = pack
                if group == 'name':
                    if string in self._keywords:
                        return Token(TokenKind.KEYW, string, span=Span((loc, self._loc)))
                    else:
                        return Token(TokenKind.NAME, string, span=Span((loc, self._loc)))
                elif group == 'number':
                    return Token(TokenKind.NUMBER, int(string), span=Span((loc, self._loc)))
                elif group == 'operator':
                    return Token(self._operators[string], span=Span((loc, self._loc)))
                elif group == 'comment':
                    self.comments.append(Token(TokenKind.COMMENT, string, span=Span((loc, self._loc))))
                    continue
                elif group == 'space':
                    continue
                assert False, "o-oh, we shouldn't end up here"

            if self.input.is_end():
                return Token(TokenKind.END)

            ch = self.input.peek()
            raise TokenizerError("unexpected character: '%s', ord=%s" % (ch, ord(ch)))

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
        MO name -> mo_children_list
        MO name

    mo_children_list:
        name , mo_children_list
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
