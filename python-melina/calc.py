#!/usr/bin/env python

'''
Desk calc example inspired by 10.2 chapter of Bjarne Stroustrup's book The C++ Programming Language.
'''

import os
import sys
import argparse
import io
import enum

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

class Tokenizer(object):
    def __init__(self, input):
        self.input = input
        self.token = Token(TokenKind.END)

    def get(self):
        self.token = token = self._get()
        return token

    def current(self):
        return self.token

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
            return Token(TokenKind.NUMBER, float(ent))
        elif ch.isalpha():
            ent = self._get_remaining(ch, self._is_alpha)
            return Token(TokenKind.NAME, ent)
        raise Exception('Unknown character: %s, ord=%s' % (ch, ord(ch)))

class Parser(object):
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

    def __init__(self, tokenizer):
        self.ts = tokenizer
        self.table = {}

    def parse(self):
        while True:
            self.ts.get()
            if self.ts.current().kind == TokenKind.END:
                break
            if self.ts.current().kind == TokenKind.PRINT:
                continue
            print self.expr(False)

    def expr(self, get):
        left = self.term(get)

        while True:
            if self.ts.current().kind == TokenKind.PLUS:
                left += self.term(True)
            elif self.ts.current().kind == TokenKind.MINUS:
                left -= self.term(True)
            else:
                return left

    def term(self, get):
        left = self.prim(get)

        while True:
            if self.ts.current().kind == TokenKind.MUL:
                left *= self.prim(True)
            elif self.ts.current().kind == TokenKind.DIV:
                left /= self.prim(True)
            else:
                return left

    def prim(self, get):
        if get:
            self.ts.get()

        if self.ts.current().kind == TokenKind.NUMBER:
            v = self.ts.current().value
            self.ts.get()
            return v
        elif self.ts.current().kind == TokenKind.NAME:
            n = self.ts.current().value
            if self.ts.get().kind == TokenKind.ASSIGN:
                self.table[n] = self.expr(True)
            return self.table[n]
        elif self.ts.current().kind == TokenKind.MINUS:
            return -self.prim(True)
        elif self.ts.current().kind == TokenKind.LP:
            e = self.expr(True)
            if self.ts.current().kind != TokenKind.RP:
                raise Exception("')' expected")
            self.ts.get()
            return e
        else:
            raise Exception("Primary expected")

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
    ts = Tokenizer(input)
    parser = Parser(ts)
    parser.parse()

    print parser.table

if __name__ == '__main__':
    main()
