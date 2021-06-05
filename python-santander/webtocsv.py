#!/usr/bin/env python3

import argparse
import collections
import pathlib
import pdb, traceback, sys
import re
import sys

import pandas as pd

class FatalError(Exception):
    pass

def existing_path(string):
    path = pathlib.Path(string)
    if not path.is_file():
        raise FatalError('file not found "%s"' % path)
    return path

def parse_opts():
    parser = argparse.ArgumentParser(
        description='Santander fio web text to csv',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('files', nargs='+', type=existing_path, help='paths to txt files')
    parser.add_argument('-o', '--out', type=pathlib.Path, help='output path')
    parser.add_argument('--pdb', action='store_true', help='drop to pdb on unexpected error')
    return parser.parse_args()

def pop_front(lst, sz):
    elems = lst[:sz]
    del lst[:sz]
    return elems

def is_date(string):
    return re.match('\d{2}\.\d{2}\.\d{4}', string)

def to_number(string):
    return '0' if string == '-' else string.replace('PLN', '').replace(' ', '').replace(',', '.')

class Parse:
    @staticmethod
    def data(lines):
        raw = lines.pop(0)
        assert is_date(raw)
        return raw

    @staticmethod
    def typ(lines):
        raw = lines.pop(0)
        assert raw in ['ZAKUP', 'SPRZEDAŻ', 'ZAMIANA']
        return {'ZAKUP': 'K', 'SPRZEDAŻ': 'S', 'ZAMIANA': 'Z'}[raw]

    @staticmethod
    def fundusz(lines):
        if lines[0].startswith('z Santander') or lines[0].startswith('na Santander'):
            lines.pop(0)

        raw = pop_front(lines, 3)
        assert raw[0].startswith('Santander')
        assert raw[1] == 'SANTANDER FIO'
        assert raw[2].startswith('<#/investment/account/')
        return raw[0].replace('Santander ', '')

    @staticmethod
    def status(lines):
        raw = lines.pop(0)
        assert raw in ['ZREALIZOWANE']
        return {'ZREALIZOWANE': 'Z'}[raw]

    @staticmethod
    def kwota(lines):
        raw = lines.pop(0)
        assert raw.endswith('PLN')
        return to_number(raw)

    @staticmethod
    def oplata(lines):
        raw = pop_front(lines, 4)
        assert raw[0].endswith('PLN')
        assert raw[1] == '?'
        assert raw[2] == ''
        assert raw[3] == 'Więcej'
        return to_number(raw[0])

    @staticmethod
    def nr_zlecenia(lines):
        raw = lines.pop(0).split(' ')
        assert len(raw) == 4
        assert raw[:3] == ['Szczegóły', 'zlecenia', 'nr']
        return raw[-1]

    @staticmethod
    def data_wyceny(lines):
        raw = pop_front(lines, 2)
        assert raw[0] == 'Data wyceny'
        return raw[1]

    @staticmethod
    def jednostki(lines):
        raw = pop_front(lines, 4)
        assert raw[0] == 'Liczba jednostek'
        assert raw[1] == '?'
        assert raw[2] == ''

        jed, kat = raw[3].split(' (kat. ')
        assert len(kat) == 2

        lines.insert(0, kat[0])
        return to_number(jed)

    @staticmethod
    def kategoria(lines):
        raw = lines.pop(0)
        assert len(raw) == 1
        return raw

    @staticmethod
    def podatek(lines):
        raw = pop_front(lines, 2)
        assert raw[0] == 'Podatek'
        assert raw[1].endswith('PLN') or raw[1] == '-'
        return to_number(raw[1])

    @staticmethod
    def nr_rejestru(lines):
        raw = pop_front(lines, 2)
        assert raw[0] == 'Numer subrejestru'
        assert raw[1].count('-') == 2
        return raw[1]

    @staticmethod
    def wycena(lines):
        raw = pop_front(lines, 2)
        assert raw[0] == 'Wycena jednostki'
        assert raw[1].endswith('PLN')
        return to_number(raw[1])

    @staticmethod
    def saldo(lines):
        raw = pop_front(lines, 3)
        assert raw[0] == 'Saldo jednostek po transakcji'
        assert raw[2] == ''

        if raw[1] == '-':
            return 0

        jed, kat = raw[1].split(' (kat. ')
        assert len(kat) == 2
        return to_number(jed)

def to_records(lines):
    FIELDS = ['data',
              'typ',
              'fundusz',
              'status',
              'kwota',
              'oplata',
              'nr_zlecenia',
              'data_wyceny',
              'jednostki',
              'kategoria',
              'podatek',
              'nr_rejestru',
              'wycena',
              'saldo']

    Record = collections.namedtuple('Record', FIELDS)

    while is_date(lines[0]):
        record = Record(*[getattr(Parse, field)(lines) for field in FIELDS])
        yield record

def parse_records(path):
    START = 'DATA ZŁOŻENIA ZLECENIA'
    NCOLS = 6

    lines = path.read_text().splitlines()
    idx = lines.index(START)
    return list(to_records(lines[idx + NCOLS:]))

def main(opts):
    FIELDS = ['nr_zlecenia',
              'data',
              'data_wyceny',
              'status',
              'typ',
              'fundusz',
              'nr_rejestru',
              'kwota',
              'oplata',
              'podatek',
              'wycena',
              'jednostki',
              'saldo',
              'kategoria']

    nested_records = [parse_records(path) for path in opts.files]
    records = [r for records in nested_records for r in records]
    records = sorted(set(records), key=records.index)
    records = list(reversed(records))
    table = pd.DataFrame(records).reindex(columns=FIELDS)

    if opts.out:
        opts.out.write_text(table.to_csv())
    else:
        print(table)

if __name__ == '__main__':
    opts = parse_opts()
    try:
        main(opts)
    except FatalError as e:
        sys.exit('error: %s' % str(e))
    except:
        if not opts.pdb:
            raise
        extype, value, tb = sys.exc_info()
        traceback.print_exc()
        pdb.post_mortem(tb)
