import pytest
import thelib

@pytest.mark.favourites
@pytest.mark.deplorables
def test_foo():
    thelib.fbar.fbaz()
    thelib.fbar.fbaz()
