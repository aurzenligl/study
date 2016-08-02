#!/usr/bin/env python

import os
import io
import subprocess
import pytest

def test_port(capsys, user_log_path, socket_count, portalloc):
    def to_str(nums):
        return ' '.join((str(num) for num in nums))

    user = os.path.join(os.path.dirname(__file__), 'user.py')
    ports = portalloc.allocate_n(socket_count)
    out = subprocess.check_output('%s %s' % (user, to_str(ports)), shell=True)
    print out.rstrip()
    portalloc.free_n(ports)

    with io.open(user_log_path, 'ab') as f:
        f.write(capsys.readouterr()[0])

    if 'error:' in out:
        pytest.fail('Already in use')

if __name__ == '__main__':
    print reserve_port()
