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
            ('parser_mo_name.xml', ':3: error: expected "class" attribute in "managedObject" tag\n  <managedObject>\n'),
            ('parser_mo_child_name.xml', ':4: error: expected "class" attribute in "childManagedObject" tag\n    <childManagedObject/>\n'),
            ('parser_field_name.xml', ':4: error: expected "name" attribute in "p" tag\n    <p/>\n'),
            ('parser_field_max_occurs_positive.xml', ':4: error: expected positive integer in "maxOccurs"\n    <p name="member" maxOccurs="0"/>\n'),
            ('parser_field_priority.xml', ':5: error: expected "priority" attribute in "creation" tag\n      <creation/>\n'),
            ('parser_field_priority_unknown.xml', ':5: error: expected "optional" or "mandatory" cardinality\n      <creation priority="unknown"/>\n'),
            ('parser_field_type.xml', ':4: error: expected "simpleType" or "complexType" tag under "p" tag\n    <p name="member" maxOccurs="1"/>\n'),
            ('parser_field_enum_text.xml', ':6: error: expected "text" attribute in "enumeration" tag\n        <enumeration/>\n'),
            ('parser_field_enum_value.xml', ':6: error: expected "value" attribute in "enumeration" tag\n        <enumeration text="Disabled"/>\n'),
            ('parser_field_enum_value_int.xml', ':6: error: expected integer enumerator value\n        <enumeration value="nonint" text="Disabled"/>\n'),
            ('parser_scalar_base.xml', ':5: error: expected "base" attribute in "simpleType" tag\n      <simpleType/>\n'),
            ('parser_scalar_base_unknown.xml', ':5: error: expected "boolean", "integer" or "string" in "base" attribute\n      <simpleType base="unknown"/>\n'),
            ('parser_scalar_int_minincl.xml', ':7: error: expected "minIncl" attribute in "range" tag\n          <range maxIncl="9500"/>\n'),
            ('parser_scalar_int_minincl_float.xml', ':7: error: expected float in "minIncl"\n          <range minIncl="nonfloat" maxIncl="9500"/>\n'),
            ('parser_scalar_int_maxincl.xml', ':7: error: expected "maxIncl" attribute in "range" tag\n          <range minIncl="0"/>\n'),
            ('parser_scalar_int_maxincl_float.xml', ':7: error: expected float in "maxIncl"\n          <range minIncl="0" maxIncl="nonfloat"/>\n'),
            ('parser_scalar_string_minlen.xml', ':6: error: expected "value" attribute in "minLength" tag\n        <minLength/>\n'),
            ('parser_scalar_string_minlen_nonneg.xml', ':6: error: expected non-negative integer in "value" attribute\n        <minLength value="-1"/>\n'),
            ('parser_scalar_string_maxlen.xml', ':7: error: expected "value" attribute in "maxLength" tag\n        <maxLength/>\n'),
            ('parser_scalar_string_maxlen_nonneg.xml', ':7: error: expected non-negative integer in "value" attribute\n        <maxLength value="-1"/>\n'),
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
            On_line = 0
            Offline = 1
    optional struct Core
        repeated enum Types
            T1 = 1
            T2 = 2
        repeated struct Numbers
            required int x
            required string y
            required string(2..15) yy
        required int a
        required int b
    required bool x
    required int y
'''

    def test_configure(self):
        tu = parse('configure.xml')
        assert str(tu) == '''\
mo CONFIGURE_MECHANISM_TASK: RESULT  // Configure mechanism task
    required struct alphaDelta  // Alpha configuration
        repeated(100) struct modified  // Modified field
            required string dn  // The dn param
            required int(0..9500) param  // The param
    required struct betaDelta  // Beta configuration
        repeated(100) struct added  // Added field
            required string devDn  // The dn param
            required int(0..65535) id  // The id
            required int(0..9500) param  // The param
        repeated(100) struct modified  // Modified field
            required string dn  // The dn param
            required int(0..9500) param  // The param
        repeated(100) struct removed  // Removed field
            required string dn  // The dn param
    required struct gammaDelta  // Gamma configuration
        repeated(10) struct modified  // Modified field
            required string dn  // The dn param
            optional struct config  // The config
                optional int(0, 9.00, 0.00000001) param  // The param
                optional struct gammaConfig  // The gamma config
                    required enum attitude  // The attitude
                        Disabled = 0
                        Enabled = 1
                repeated(9999) struct gammaGimmickConfig  // The gamma gimmick config
                    required string dn  // The dn param
                    required int rate  // The rate param
                    required int size  // The size param
'''
