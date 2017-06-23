import os
import timeit
import pytest
import melina

datadir = os.path.abspath(__file__ + '/../data')

def parse(filename):
    return melina.MetaParser.from_file(datadir + '/' + filename).parse()

def id_func(param):
    if isinstance(param, str):
        name, ext = os.path.splitext(param)
        if ext == '.meta':
            return name
        else:
            return 'msg'

class TestParserErrors():
    @pytest.mark.parametrize(
        'filename, message',
        [
            ('tokenizer.meta', ':1:4: error: unexpected character: "?", ord=63\nmo ? SHORT\n   ^\n'),
            ('parser_mo_keyw_mo.meta', ':2:3: error: expected keyword "mo"\n  does not start with mo\n  ^\n'),
            ('parser_mo_name.meta', ':1:4: error: expected mo name\nmo {};\n   ^\n'),
            ('parser_mo_lcb.meta', ':1:8: error: expected mo definition\nmo NAME;\n       ^\n')
        ],
        ids=id_func
    )
    def test_errors(self, filename, message):
        with pytest.raises(melina.MetaParserError) as e:
            tu = parse('meta_errors/' + filename)
        actualmsg = e.value.prettymsg
        shortpath_actualmsg = actualmsg.split('/tests/data/meta_errors/')[1]
        assert shortpath_actualmsg == filename + message

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
