import os
import timeit
import pytest
import melina

datadir = os.path.abspath(__file__ + '/../data')

def parse(filename):
    return melina.XmlParser.from_file(datadir + '/' + filename).parse()

def id_func(param):
    if isinstance(param, str):
        name, ext = os.path.splitext(param)
        if ext == '.xml':
            return name
        else:
            return 'msg'

class TestParserErrors():
    @pytest.mark.parametrize(
        'filename, message',
        [
            ('xml_syntax.xml', ':3:11: error: error parsing attribute name\n  <managed<>?)\n          ^\n'),
            ('parser_mo_name.xml', ':3: error: expected "class" attribute in managedObject tag\n  <managedObject>\n'),
            ('parser_mo_child_name.xml', ':4: error: expected "class" attribute in childManagedObject tag\n    <childManagedObject/>\n'),
            ('parser_field_name.xml', ':4: error: expected "name" attribute in p tag\n    <p/>\n'),
            ('parser_field_max_occurs.xml', ':4: error: expected "maxOccurs" attribute in p tag\n    <p name="member"/>\n'),
            ('parser_field_max_occurs_positive.xml', ':4: error: expected positive integer in "maxOccurs"\n    <p name="member" maxOccurs="0"/>\n'),
            ('parser_field_priority.xml', ':5: error: expected "priority" attribute in creation tag\n      <creation/>\n'),
            ('parser_field_priority_unknown.xml', ':5: error: expected "optional" or "mandatory" cardinality\n      <creation priority="unknown"/>\n'),
        ],
        ids=id_func
    )
    def test_errors(self, filename, message):
        with pytest.raises(melina.XmlParserError) as e:
            tu = parse('xml_errors/' + filename)
        actualmsg = e.value.prettymsg
        shortpath_actualmsg = actualmsg.split('/tests/data/xml_errors/')[1]
        assert shortpath_actualmsg == filename + message

class TestParser():
    def test_example(self):
        tu = parse('example.xml')
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
        tu = parse('configure.xml')
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
