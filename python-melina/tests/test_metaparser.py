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
            ('parser_mo_lcb.meta', ':1:8: error: expected mo definition\nmo NAME;\n       ^\n'),
            ('parser_mo_semi.meta', ':4:1: error: expected semicolon after mo definition\n}\n^\n'),
            ('parser_mo_child_list_name.meta', ':2:1: error: expected mo child name\n{};\n^\n'),
            ('parser_mo_child_list_nocomma.meta', ':1:19: error: expected mo definition\nmo NAME -> CHILD1 CHILD2\n                  ^\n'),
            ('parser_mo_child_list_2ndname.meta', ':2:1: error: expected mo child name\n{};\n^\n'),
            ('parser_field_keyw.meta', ':3:5: error: expected field definition\n    unknown x;\n    ^\n'),
            ('parser_struct_name.meta', ':3:20: error: expected struct name\n    optional struct;\n                   ^\n'),
            ('parser_struct_lcb.meta', ':3:25: error: expected struct definition\n    optional struct Name;\n                        ^\n'),
            ('parser_struct_semi.meta', ':3:27: error: expected semicolon after struct definition\n    optional struct Name {}\n                          ^\n'),
            ('parser_enum_name.meta', ':3:19: error: expected enum name\n    repeated enum {};\n                  ^\n'),
            ('parser_enum_lcb.meta', ':3:23: error: expected enum definition\n    repeated enum Name;\n                      ^\n'),
            ('parser_enum_rcb.meta', ':6:9: error: expected brace closing enum definition\n        B = 2\n        ^\n'),
            ('parser_enum_semi.meta', ':6:5: error: expected semicolon after enum definition\n    }\n    ^\n'),
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
