import os
import timeit
import pytest
import melina

datadir = os.path.abspath(__file__ + '/../data')

def parse(filename):
    return melina.MetaParser.from_file(datadir + '/' + filename).parse()

class TestParserErrors():
    def test_error_tokenizer(self):
        with pytest.raises(melina.MetaParserError) as e:
            tu = parse('meta_errors/tokenizer.meta')
        assert e.value.origin.endswith('tokenizer.meta:1:4')
        assert e.value.message == '''unexpected character: "?", ord=63'''
        assert e.value.prettymsg.endswith('''\
tokenizer.meta:1:4: error: unexpected character: "?", ord=63
mo ? SHORT
   ^
''')

    def test_error_mo_expected_mo(self):
        with pytest.raises(melina.MetaParserError) as e:
            tu = parse('meta_errors/parser_mo_expected_mo.meta')
        assert e.value.prettymsg.endswith('''\
parser_mo_expected_mo.meta:2:3: error: expected keyword "mo"
  does not start with mo
  ^
''')

class TestParser():
    def test_example(self):
        tu = parse('example.meta')
        assert str(tu) == '''\
mo MACHINE_L: SENSOR WHEEL ARM
    required struct StateBox
        repeated enum FaultStatus
            Empty = 0
            Disconnected = 1
            RoofFlewOff = 2
        required enum AdminStatus
            Locked = 0
            Unlocked = 1
        required enum AvailStatus
            Online = 0
            Offline = 1
    optional struct Core
        repeated enum Types
            T1 = 1
            T2 = 2
        repeated struct Numbers
            required int x
            required int y
        required int a
        required int b
    required int x
    required int y
'''

    def test_configure(self):
        tu = parse('configure.meta')
        assert str(tu) == '''\
mo CONFIGURE_MECHANISM_TASK: RESULT
    required struct alphaDelta
        repeated struct modified
            required string dn
            required int param
    required struct betaDelta
        repeated struct added
            required string devDn
            required int id
            required int param
        repeated struct modified
            required string dn
            required int param
        repeated struct removed
            required string dn
    required struct gammaDelta
        repeated struct modified
            required string dn
            optional struct config
                optional int param
                optional struct gammaConfig
                    required enum attitude
                        Disabled = 0
                        Enabled = 1
                repeated struct gammaGimmickConfig
                    required string dn
                    required int rate
                    required int size
'''

    def hardskip_test_performance(self):
        print timeit.timeit("parse('longexample.meta')", "from %s import parse" % __name__, number=1)
