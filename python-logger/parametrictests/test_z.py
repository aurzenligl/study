import pytest

@pytest.mark.parametrize('param1,param2', [
    (2, 'abc'),
    (4.127, 'de')
])
def test_param(param1, param2):
    pass
