#!/usr/bin/env python

import os
import sys
import argparse
import io
import enum

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
    _repr_vals = {
        TokenKind.LCB: '{',
        TokenKind.RCB: '}',
        TokenKind.SEMI: ';',
        TokenKind.COMMA: ',',
        TokenKind.ASSIGN: '=',
        TokenKind.ARROW: '->',
    }

    _repr_direct = (TokenKind.KEYW, TokenKind.NAME, TokenKind.NUMBER, TokenKind.COMMENT)

    def __init__(self, kind, value = None):
        self.kind = kind
        self.value = value

    def __repr__(self):
        if self.kind in self._repr_direct:
            return "<Token %s %s>" % (self.kind.name, self.value)
        reprval = self._repr_vals.get(self.kind)
        if reprval:
            return "<Token %s %s>" % (self.kind.name, reprval)
        return "<Token %s>" % (self.kind.name)

class TokenizerInput(object):
    def __init__(self, input):
        self.input = input

    def read(self):
        return self.input.read(1)

    def back(self):
        self.input.seek(self.input.tell() - 1)

class TokenizerError(Exception):
    pass

class Tokenizer(object):
    def __init__(self, input):
        self.input = TokenizerInput(input)
        self.token = Token(TokenKind.END)
        '''TODO keep comments'''

    def get(self):
        while True:
            token = self._get()
            if token:
                self.token = token
                return token

    @property
    def cur(self):
        return self.token

    @staticmethod
    def _within(ch, first, last):
        return ord(first) <= ord(ch) <= ord(last)

    def _read_all(self, pred):
        string = ''
        while True:
            ch = self.input.read()
            if pred(ch):
                string += ch
                continue
            if ch:
                self.input.back()
            return string

    def _read_until(self, string):
        read = ''
        while True:
            read += self.input.read()
            if read.endswith(string):
                return read

    _keywords = ('mo', 'struct', 'enum', 'repeated', 'optional', 'int', 'float', 'string')

    _unitokens = {
        '{': TokenKind.LCB,
        '}': TokenKind.RCB,
        ';': TokenKind.SEMI,
        ',': TokenKind.COMMA,
        '=': TokenKind.ASSIGN,
    }

    def _get(self):
        '''TODO location'''
        '''TODO comments'''

        ch = self.input.read()

        # END
        if ch == '':
            return Token(TokenKind.END)

        # SPACE <ignored> ' \t\n'
        if ch == ' ' or ch == '\t' or ch == '\n':
            string = ch + self._read_all(lambda ch: ch == ' ' or ch == '\t' or ch == '\n')
            return

        # COMMENT <ignored, stored> '//\n' '/**/'
        if ch == '/':
            ch = self.input.read()
            if ch == '/':
                string = '//' + self._read_all(lambda ch: ch != '\n')
                return Token(TokenKind.COMMENT, string)
            elif ch == '*':
                string = '/*' + self._read_until('*/')
                return Token(TokenKind.COMMENT, string)
            else:
                raise TokenizerError("character '/' can be used only as part of '//' or '/*' comment opening")

        # KEYW [_a-zA-Z]
        # NAME [_a-zA-Z]
        if self._within(ch, 'A', 'Z') or self._within(ch, 'a', 'z') or ch == '_':
            string = ch + self._read_all(lambda ch: (
                self._within(ch, '0', '9') or
                self._within(ch, 'A', 'Z') or self._within(ch, 'a', 'z') or ch == '_'
            ))
            if string in self._keywords:
                return Token(TokenKind.KEYW, string)
            else:
                return Token(TokenKind.NAME, string)

        # NUMBER [0-9]*
        if self._within(ch, '0', '9'):
            string = ch + self._read_all(lambda ch: self._within(ch, '0', '9'))
            return Token(TokenKind.NUMBER, int(string))

        # ARROW ->
        if ch == '-':
            ch = self.input.read()
            if ch == '>':
                return Token(TokenKind.ARROW)
            else:
                raise TokenizerError("character '-' can be used only as part of arrow operator '->'")

        # LCB {
        # RCB }
        # SEMI ;
        # COMMA ,
        # ASSIGN =
        kind = self._unitokens.get(ch)
        if kind:
            return Token(kind)

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
    input = io.FileIO(opts.input)
    ts = Tokenizer(input)
    
    for tok in get_tokens(ts):
        print tok

    print "ok"

if __name__ == '__main__':
    main()
