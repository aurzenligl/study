#!/usr/bin/env python

import os
import io
import subprocess
from ports import reserve_port, free_port

def test_port(capsys, user_log_path, socket_count):
    def to_str(nums):
        return ' '.join((str(num) for num in nums))

    user = os.path.join(os.path.dirname(__file__), 'user.py')
    ports = [reserve_port() for _ in range(socket_count)]
    print subprocess.check_output('%s %s' % (user, to_str(ports)), shell=True)
    [free_port(port) for port in ports]
    with io.open(user_log_path, 'ab') as f:
        f.write(capsys.readouterr()[0])

if __name__ == '__main__':
    print reserve_port()
