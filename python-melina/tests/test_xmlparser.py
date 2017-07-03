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
            ('parser_mo_name.xml', ':4: error: expected "class" attribute in "managedObject" tag\n  <managedObject>\n'),
            ('parser_mo_child_name.xml', ':5: error: expected "class" attribute in "childManagedObject" tag\n    <childManagedObject/>\n'),
            ('parser_field_name.xml', ':5: error: expected "name" attribute in "p" tag\n    <p/>\n'),
            ('parser_field_max_occurs_positive.xml', ':5: error: expected positive integer in "maxOccurs"\n    <p name="member" maxOccurs="0"/>\n'),
            ('parser_field_priority.xml', ':6: error: expected "priority" attribute in "creation" tag\n      <creation/>\n'),
            ('parser_field_priority_unknown.xml', ':6: error: expected "optional" or "mandatory" cardinality\n      <creation priority="unknown"/>\n'),
            ('parser_field_type.xml', ':5: error: expected "simpleType" or "complexType" tag under "p" tag\n    <p name="member" maxOccurs="1"/>\n'),
            ('parser_field_enum_text.xml', ':7: error: expected "text" attribute in "enumeration" tag\n        <enumeration/>\n'),
            ('parser_field_enum_value.xml', ':7: error: expected "value" attribute in "enumeration" tag\n        <enumeration text="Disabled"/>\n'),
            ('parser_field_enum_value_int.xml', ':7: error: expected integer enumerator value\n        <enumeration value="nonint" text="Disabled"/>\n'),
            ('parser_scalar_base.xml', ':6: error: expected "base" attribute in "simpleType" tag\n      <simpleType/>\n'),
            ('parser_scalar_base_unknown.xml', ':6: error: expected "boolean", "integer" or "string" in "base" attribute\n      <simpleType base="unknown"/>\n'),
            ('parser_scalar_int_minincl.xml', ':8: error: expected "minIncl" attribute in "range" tag\n          <range maxIncl="9500"/>\n'),
            ('parser_scalar_int_minincl_float.xml', ':8: error: expected int in "minIncl" attribute\n          <range minIncl="nonfloat" maxIncl="9500"/>\n'),
            ('parser_scalar_int_maxincl.xml', ':8: error: expected "maxIncl" attribute in "range" tag\n          <range minIncl="0"/>\n'),
            ('parser_scalar_int_maxincl_float.xml', ':8: error: expected int in "maxIncl" attribute\n          <range minIncl="0" maxIncl="nonfloat"/>\n'),
            ('parser_scalar_string_minlen.xml', ':7: error: expected "value" attribute in "minLength" tag\n        <minLength/>\n'),
            ('parser_scalar_string_minlen_nonneg.xml', ':7: error: expected non-negative integer in "value" attribute\n        <minLength value="-1"/>\n'),
            ('parser_scalar_string_maxlen.xml', ':8: error: expected "value" attribute in "maxLength" tag\n        <maxLength/>\n'),
            ('parser_scalar_string_maxlen_nonneg.xml', ':8: error: expected non-negative integer in "value" attribute\n        <maxLength value="-1"/>\n'),
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
pdmeta: "1.7", domain: "foo", product: "bar", release: "AX-19.2", version: "1.5.3.0.beta", revision: "1982713"
mo(cd) MACHINE_L: SENSOR(1) WHEEL(12) ARM  // Mo documentation
    required struct StateBox
        repeated enum FaultStatus
            Empty = 0
            Disconnected = 1
            RoofFlewOff = 2
        required enum AdminStatus
            Locked = 0
            Unlocked = 1
        required enum AvailStatus [default = 1]
            On_line = 0
            Offline = 1
    optional struct Core
        repeated enum Types
            T1 = 1
            T2 = 2
        repeated struct Numbers
            required int x [default = 0]
            required int(-212..12) xx [default = 5]
            required int(-212.23, 12.20001111, 0.00000002) xxx [default = -57.91]
            required string y
            required string(0..15) yy
            required bool z [default = true]
        required int a
        required int b
    required bool x
    required int y
'''

    def test_configure(self):
        tu = parse('configure.xml')
        assert str(tu) == '''\
pdmeta: "", domain: "", product: "", release: "", version: "", revision: ""
mo CONFIGURE_MECHANISM_TASK: RESULT(1)  // Configure mechanism task
    required struct AlphaDelta  // Alpha configuration
        repeated(100) struct Modified  // Modified field
            required string(0..32767) dn [default = ""]  // The dn param
            required int(0..9500) param [units = "octets"]  // The param
    required struct BetaDelta  // Beta configuration
        repeated(100) struct Added  // Added field
            required string(0..32767) devDn [default = "foo-bar"]  // The dn param
            required int(0..65535) id [units = ""]  // The id
            required int(0..9500) param  // The param
        repeated(100) struct Modified  // Modified field
            required string(0..32767) dn  // The dn param
            required int(0..9500) param  // The param
        repeated(100) struct Removed  // Removed field
            required string(0..32767) dn  // The dn param
    required struct GammaDelta  // Gamma configuration
        repeated(10) struct Modified  // Modified field
            required string(0..32767) dn  // The dn param
            optional struct Config  // The config
                optional int(0, 9.00, 0.00000001) param  // The param
                optional struct GammaConfig  // The gamma config
                    required enum Attitude [default = 0]  // The attitude
                        Disabled = 0
                        Enabled = 1
                repeated(9999) struct GammaGimmickConfig  // The gamma gimmick config
                    required string(0..32767) dn  // The dn param
                    required int rate [units = "octets"]  // The rate param
                    required int size  // The size param
'''
