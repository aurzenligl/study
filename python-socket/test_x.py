import os
import socket
import io
import subprocess

def get_free_port():
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind(("",0))
    s.listen(1)
    port = s.getsockname()[1]
    s.close()
    return port

def test_port(capsys, user_log_path):
    user = os.path.join(os.path.dirname(__file__), 'user.py')
    port = get_free_port()
    portstring = ' '.join([str(get_free_port()) for _ in range(100)])
    x = subprocess.check_output('%s %s' % (user, portstring), shell=True)
    with io.open(user_log_path, 'ab') as f:
        f.write(x)
