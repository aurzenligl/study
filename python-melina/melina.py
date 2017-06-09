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

    def clone(self):
        loc = Location()
        loc.pos = self.pos
        loc.line = self.line
        loc.col = self.col
        return loc

    def newchar(self, n = 1):
        self.pos += n
        self.col += n

    def newline(self):
        self.pos += 1
        self.line += 1
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

    def __repr__(self):
        return '%s:%s' % (self.line, self.col)

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
    __slots__ = ('kind', 'value', 'locs')

    _repr_vals = {
        TokenKind.LCB: '{',
        TokenKind.RCB: '}',
        TokenKind.SEMI: ';',
        TokenKind.COMMA: ',',
        TokenKind.ASSIGN: '=',
        TokenKind.ARROW: '->',
    }

    _repr_direct = (TokenKind.KEYW, TokenKind.NAME, TokenKind.NUMBER, TokenKind.COMMENT)

    def __init__(self, kind, value = None, locs = None):
        self.kind = kind
        self.value = value
        self.locs = locs

    def __repr__(self):
        if self.locs:
            repr = "<Token %s-%s %s" % (self.locs[0], self.locs[1], self.kind.name)
        else:
            repr = "<Token %s" % self.kind.name

        if self.kind in self._repr_direct:
            return repr + " %s>" % self.value
        reprval = self._repr_vals.get(self.kind)
        if reprval:
            return repr + " %s>" % reprval
        return repr + ">"

_is_name = re.compile(r'[a-zA-Z_][a-zA-Z0-9_]*')
_is_number = re.compile(r'[1-9][0-9]*')
_is_space = re.compile(r'\s*')
_is_comment = re.compile(r'(//.*\n|/\*(\*(?!/)|[^*])*\*/)')
_is_operator = re.compile(r'(->)|[{};,=]')

class TokenizerInput(object):
    def __init__(self, input):
        '''TODO preload entire file and use regex'''
        self.input = input.read()
        self.loc = Location()

    def is_end(self):
        return self.loc.pos == len(self.input)

    def read(self):
        if self.loc.pos < len(self.input):
            ch = self.input[self.loc.pos]
            self._advance_loc(ch)
            return ch
        else:
            return ''

    def read_re(self, re):
        match = re.match(self.input, self.loc.pos)
        if match:
            string = match.group()
            '''TODO use match.end() to move pos and handle newlines in some clever way'''
            self.loc.progress(string)
            return string

    def read_all(self, pred):
        pos = origpos = self.loc.pos
        while self.loc.pos < len(self.input):
            if pred(self.input[self.loc.pos]):
                self._advance_loc(self.input[self.loc.pos])
            else:
                break
        return self.input[origpos:self.loc.pos]

    def read_until(self, string):
        read = ''
        while True:
            read += self.read()
            if read.endswith(string):
                return read

    def _advance_loc(self, ch):
        if ch == '\n':
            self.loc.newline()
        else:
            self.loc.newchar()

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
        return self.input.loc.clone()

    @staticmethod
    def _within(ch, first, last):
        return ord(first) <= ord(ch) <= ord(last)

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
            loc = self._loc
            loc.newchar()

            # KEYW [_a-zA-Z]
            # NAME [_a-zA-Z]
            string = self.input.read_re(_is_name)
            if string:
                if string in self._keywords:
                    return Token(TokenKind.KEYW, string, locs=(loc, self._loc))
                else:
                    return Token(TokenKind.NAME, string, locs=(loc, self._loc))

            # NUMBER [0-9]*
            string = self.input.read_re(_is_number)
            if string:
                return Token(TokenKind.NUMBER, int(string), locs=(loc, self._loc))

            # SPACE <ignored> ' \t\n'
            string = self.input.read_re(_is_space)
            if string:
                continue

            # COMMENT <ignored, stored> '//\n' '/**/'
            string = self.input.read_re(_is_comment)
            if string:
                self.comments.append(Token(TokenKind.COMMENT, string, locs=(loc, self._loc)))
                continue

            # ARROW ->
            # LCB {
            # RCB }
            # SEMI ;
            # COMMA ,
            # ASSIGN =
            string = self.input.read_re(_is_operator)
            if string:
                return Token(self._operators[string], locs=(loc, self._loc))

            # END
            if self.input.is_end():
                return Token(TokenKind.END)

            ch = self.input.read()
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
