#!/bin/bash

scriptdir="$( cd "$( dirname "${BASH_SOURCE[0]}"  )" >/dev/null 2>&1 && pwd  )"
cd $scriptdir

mkdir -p packages/codec
touch packages/codec/__init__.py
prophyc --python_out packages/codec/ b.prophy d.prophy
