#!/usr/bin/env python

import sys
import os
import pkg_resources

def require_one(one):
    try:
        pkg_resources.require(one)
    except pkg_resources.DistributionNotFound as e:
        return 'package not found: %s' % e
    except pkg_resources.VersionConflict as e:
        return 'package version conflict: found: %s, expected: %s' % (str(e[0]), str(e[1]))

def format_message(errors):
    return '\n'.join([
        'Python requirements not met:',
    ] + ['    ' + e for e in errors] + [
        '',
        'Install them using:',
        '    $ [sudo] pip install -r requirements.txt',
        'Or:',
        '    $ pip install -r requirements.txt -t .pip',
        '    $ PYTHONPATH=.pip py.test',
    ])

reqpath = os.path.join(os.path.dirname(__file__), 'requirements.txt')
errors = filter(None, (require_one(x) for x in open(reqpath)))
if errors:
    sys.exit(format_message(errors))
