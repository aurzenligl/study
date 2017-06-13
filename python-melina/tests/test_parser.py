import os
import melina

datadir = os.path.abspath(__file__ + '/../data')

def parse(filename):
    return melina.Parser.from_file(datadir + '/' + filename).parse()

class TestParser():
    def test_example(self):
        tu = parse('example.meta')
