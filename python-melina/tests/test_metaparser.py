import os
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
            ('parser_struct_lcb.meta', ':3:25: error: expected struct definition\n    optional struct name;\n                        ^\n'),
            ('parser_struct_semi.meta', ':3:27: error: expected semicolon after struct definition\n    optional struct name {}\n                          ^\n'),
            ('parser_enum_name.meta', ':3:19: error: expected enum name\n    repeated enum {};\n                  ^\n'),
            ('parser_enum_lcb.meta', ':3:23: error: expected enum definition\n    repeated enum name;\n                      ^\n'),
            ('parser_enum_rcb.meta', ':6:9: error: expected brace closing enum definition\n        B = 2\n        ^\n'),
            ('parser_enum_semi.meta', ':6:5: error: expected semicolon after enum definition\n    }\n    ^\n'),
            ('parser_enumerator_list_value.meta', ':5:13: error: expected enumerator value\n        A = novalue\n            ^\n'),
            ('parser_scalar_name.meta', ':3:9: error: expected scalar name\n    int int;\n        ^\n'),
            ('parser_scalar_semi.meta', ':3:9: error: expected semicolon closing field definition\n    int x\n        ^\n'),
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
pdmeta: "1.7", domain: "foo", product: "bar", release: "AX-19.2", version: "1.5.3.0.beta", revision: "1982713"
mo(cd) MACHINE_L: SENSOR(1) WHEEL(12) ARM  // This is an example managed object: The Machine.
    required struct StateBox
        repeated enum FaultStatus
            Empty = 0
            Disconnected = 1
            RoofFlewOff = 2
        required enum AdminStatus
            Locked = 0
            Unlocked = 1
        required enum AvailStatus [default = 1]  // Enum can be documented too.
            Online = 0
            Offline = 1
    optional struct Core
        repeated enum Types
            T1 = 1
            T2 = 2
            42Val = 3
            000 = 4
        repeated struct Numbers  // This is the heart of the machine.
            required int x [default = 0]
            required int(-12..42) xx [default = -4]
            required int(-0.0001, 0.0002, 0.0000001) xxx [default = -0.00007]
            required int(-1, 1, 0.01) xxxx [default = 0, units = "bazes"]
            required string y
            required string(2..15) yy
            required bool z [default = true]
        required int a [units = ""]
        required int b [units = "petabytes"]
    repeated(42) bool x  // comment about something
    required int y  // another comment
'''

    def test_configure(self):
        tu = parse('configure.meta')
        assert str(tu) == '''\
pdmeta: "", domain: "", product: "", release: "", version: "", revision: ""
mo CONFIGURE_MECHANISM_TASK: RESULT
    required struct AlphaDelta
        repeated struct Modified
            required string dn [default = ""]
            required int param
    required struct BetaDelta
        repeated struct Added
            required string devDn [default = "foo-bar"]
            required int id
            required int param
        repeated struct Modified
            required string dn
            required int param
        repeated struct Removed
            required string dn
    required struct GammaDelta  // Gamma can be explained here. There is no added/removed as Gamma Software deployment/hardware decide how many gammas exist in given execution.
        repeated struct Modified
            required string dn
            optional struct Config
                optional int param
                optional struct GammaConfig
                    required enum Attitude
                        Disabled = 0
                        Enabled = 1
                repeated struct GammaGimmickConfig
                    required string dn
                    required int rate
                    required int size
'''

    def hardskip_test_performance(self):
        import timeit
        print timeit.timeit("parse('longexample.meta')", "from %s import parse" % __name__, number=1)
