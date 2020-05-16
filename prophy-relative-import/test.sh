#!/bin/bash

scriptdir="$( cd "$( dirname "${BASH_SOURCE[0]}"  )" >/dev/null 2>&1 && pwd  )"
cd $scriptdir

export PYTHONPATH="$scriptdir/packages"

python -c "from codec import d; print('imported successfully')"
