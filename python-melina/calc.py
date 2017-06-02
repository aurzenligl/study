#!/usr/bin/env python

import os
import sys
import argparse
import io
import enum

'''
program:
    end
    expr_list end

expr_list:
    expression print
    expression print expr_list

expression:
    expression + term
    expression - term
    term

term:
    term / primary
    term * primary
    primary

primary:
    number
    name
    name = expression
    - primary
    ( expression )
'''

class TokenKind(enum.Enum):
    NAME = 1
    NUMBER = 2
    END = 3
    PLUS = 4  # +
    MINUS = 5  # -
    MUL = 6  # *
    DIV = 7  # /
    PRINT = 8  # ;
    ASSIGN = 9  # =
    LP = 10  # (
    RP = 11  # )

class Token(object):
    def __init__(self, kind, value = None):
        self.kind = kind
        self.value = value

    def __repr__(self):
        if self.kind == TokenKind.NAME or self.kind == TokenKind.NUMBER:
            return "<Token %s %s>" % (self.kind.name, self.value)
        else:
            return "<Token %s>" % (self.kind.name)

class TokenStream(object):
    def __init__(self, input):
        self.input = input
        self.token = self.get()

    def get(self):
        self.token = token = self._get()
        return token

    _unitokens = {
        ord('+'): TokenKind.PLUS,
        ord('-'): TokenKind.MINUS,
        ord('*'): TokenKind.MUL,
        ord('/'): TokenKind.DIV,
        ord(';'): TokenKind.PRINT,
        ord('='): TokenKind.ASSIGN,
        ord('('): TokenKind.LP,
        ord(')'): TokenKind.RP,
    }

    @staticmethod
    def _is_numeric(ch):
        return 48 <= ord(ch) < 58 or ch == '.'

    @staticmethod
    def _is_alpha(ch):
        return ch.isalpha()

    @staticmethod
    def _is_space(ch):
        return ch.isspace()

    def _get_remaining(self, ch, pred):
        entity = ch
        while True:
            ch = self.input.read(1)
            if pred(ch):
                entity += ch
                continue
            if ch:
                self.input.seek(self.input.tell() - 1)
            return entity

    def _get(self):
        self._get_remaining('', self._is_space)
        ch = self.input.read(1)
        if ch == '':
            return Token(TokenKind.END)
        elif ord(ch) in self._unitokens:
            return Token(self._unitokens[ord(ch)])
        elif self._is_numeric(ch):
            ent = self._get_remaining(ch, self._is_numeric)
            return Token(TokenKind.NUMBER, int(ent))
        elif ch.isalpha():
            ent = self._get_remaining(ch, self._is_alpha)
            return Token(TokenKind.NAME, ent)
        raise Exception('Unknown character: %s, ord=%s' % (ch, ord(ch)))

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

def main():
    opts = parse_options()
    input = io.FileIO(opts.input)
    tokenizer = TokenStream(input)
    while True:
        print tokenizer.token
        if tokenizer.get().kind == TokenKind.END:
            return

if __name__ == '__main__':
    main()
