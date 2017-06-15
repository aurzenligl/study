import os
import melina

datadir = os.path.abspath(__file__ + '/../data')

def parse(filename):
    return melina.Parser.from_file(datadir + '/' + filename).parse()

def generate(tu):
    return melina.Generator(tu).to_string()

class TestGenerator():
    def test_example(self):
        tu = parse('example.meta')
        generate(tu)
        '''TODO assert expectation'''
