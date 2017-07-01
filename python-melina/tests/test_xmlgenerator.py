import os
import melina

datadir = os.path.abspath(__file__ + '/../data')

def parse(filename):
    return melina.XmlParser.from_file(datadir + '/' + filename).parse()

def generate(tu):
    return melina.XmlGenerator(tu).to_string()

class TestGenerator():
    def test_example(self):
        tu = parse('example.xml')
        xml = generate(tu)
        assert xml == '''\
<?xml version="1.0" encoding="utf-8"?>
<pdmeta>
  <managedObject class="MACHINE_L">
    <childManagedObject class="SENSOR"/>
    <childManagedObject class="WHEEL"/>
    <childManagedObject class="ARM"/>
    <p name="StateBox" maxOccurs="1">
      <complexType>
        <p name="FaultStatus" maxOccurs="999999">
          <simpleType base="integer">
            <enumeration value="0" text="Empty"/>
            <enumeration value="1" text="Disconnected"/>
            <enumeration value="2" text="RoofFlewOff"/>
          </simpleType>
        </p>
        <p name="AdminStatus" maxOccurs="1">
          <simpleType base="integer">
            <enumeration value="0" text="Locked"/>
            <enumeration value="1" text="Unlocked"/>
          </simpleType>
        </p>
        <p name="AvailStatus" maxOccurs="1">
          <simpleType base="integer">
            <enumeration value="0" text="On_line"/>
            <enumeration value="1" text="Offline"/>
          </simpleType>
        </p>
      </complexType>
    </p>
    <p name="Core" maxOccurs="1">
      <creation priority="optional"/>
      <complexType>
        <p name="Types" maxOccurs="999999">
          <simpleType base="integer">
            <enumeration value="1" text="T1"/>
            <enumeration value="2" text="T2"/>
          </simpleType>
        </p>
        <p name="Numbers" maxOccurs="999999">
          <complexType>
            <p name="x" maxOccurs="1">
              <simpleType base="integer">
                <default value="0"/>
              </simpleType>
            </p>
            <p name="xx" maxOccurs="1">
              <simpleType base="integer">
                <editing>
                  <range minIncl="-212" maxIncl="12"/>
                </editing>
                <default value="5"/>
              </simpleType>
            </p>
            <p name="xxx" maxOccurs="1">
              <simpleType base="integer">
                <editing>
                  <range minIncl="-212.23" maxIncl="12.20001111" step="0.00000002"/>
                </editing>
                <default value="-57.91"/>
              </simpleType>
            </p>
            <p name="y" maxOccurs="1">
              <simpleType base="string"/>
            </p>
            <p name="yy" maxOccurs="1">
              <simpleType base="string">
                <minLength value="0"/>
                <maxLength value="15"/>
              </simpleType>
            </p>
          </complexType>
        </p>
        <p name="a" maxOccurs="1">
          <simpleType base="integer"/>
        </p>
        <p name="b" maxOccurs="1">
          <simpleType base="integer"/>
        </p>
      </complexType>
    </p>
    <p name="x" maxOccurs="1">
      <simpleType base="boolean"/>
    </p>
    <p name="y" maxOccurs="1">
      <simpleType base="integer"/>
    </p>
  </managedObject>
</pdmeta>
'''

    def test_configure(self):
        tu = parse('configure.xml')
        xml = generate(tu)
        assert xml == '''\
<?xml version="1.0" encoding="utf-8"?>
<pdmeta>
  <managedObject class="CONFIGURE_MECHANISM_TASK" fullName="Configure mechanism task">
    <childManagedObject class="RESULT"/>
    <p name="alphaDelta" fullName="Alpha configuration" maxOccurs="1">
      <complexType>
        <p name="modified" fullName="Modified field" maxOccurs="100">
          <complexType>
            <p name="dn" fullName="The dn param" maxOccurs="1">
              <simpleType base="string">
                <minLength value="0"/>
                <maxLength value="32767"/>
                <default value=""/>
              </simpleType>
            </p>
            <p name="param" fullName="The param" maxOccurs="1">
              <simpleType base="integer">
                <editing units="octets">
                  <range minIncl="0" maxIncl="9500"/>
                </editing>
              </simpleType>
            </p>
          </complexType>
        </p>
      </complexType>
    </p>
    <p name="betaDelta" fullName="Beta configuration" maxOccurs="1">
      <complexType>
        <p name="added" fullName="Added field" maxOccurs="100">
          <complexType>
            <p name="devDn" fullName="The dn param" maxOccurs="1">
              <simpleType base="string">
                <minLength value="0"/>
                <maxLength value="32767"/>
                <default value="foo-bar"/>
              </simpleType>
            </p>
            <p name="id" fullName="The id" maxOccurs="1">
              <simpleType base="integer">
                <editing units="">
                  <range minIncl="0" maxIncl="65535"/>
                </editing>
              </simpleType>
            </p>
            <p name="param" fullName="The param" maxOccurs="1">
              <simpleType base="integer">
                <editing>
                  <range minIncl="0" maxIncl="9500"/>
                </editing>
              </simpleType>
            </p>
          </complexType>
        </p>
        <p name="modified" fullName="Modified field" maxOccurs="100">
          <complexType>
            <p name="dn" fullName="The dn param" maxOccurs="1">
              <simpleType base="string">
                <minLength value="0"/>
                <maxLength value="32767"/>
              </simpleType>
            </p>
            <p name="param" fullName="The param" maxOccurs="1">
              <simpleType base="integer">
                <editing>
                  <range minIncl="0" maxIncl="9500"/>
                </editing>
              </simpleType>
            </p>
          </complexType>
        </p>
        <p name="removed" fullName="Removed field" maxOccurs="100">
          <complexType>
            <p name="dn" fullName="The dn param" maxOccurs="1">
              <simpleType base="string">
                <minLength value="0"/>
                <maxLength value="32767"/>
              </simpleType>
            </p>
          </complexType>
        </p>
      </complexType>
    </p>
    <p name="gammaDelta" fullName="Gamma configuration" maxOccurs="1">
      <complexType>
        <p name="modified" fullName="Modified field" maxOccurs="10">
          <complexType>
            <p name="dn" fullName="The dn param" maxOccurs="1">
              <simpleType base="string">
                <minLength value="0"/>
                <maxLength value="32767"/>
              </simpleType>
            </p>
            <p name="config" fullName="The config" maxOccurs="1">
              <creation priority="optional"/>
              <complexType>
                <p name="param" fullName="The param" maxOccurs="1">
                  <creation priority="optional"/>
                  <simpleType base="integer">
                    <editing>
                      <range minIncl="0" maxIncl="9.00" step="0.00000001"/>
                    </editing>
                  </simpleType>
                </p>
                <p name="gammaConfig" fullName="The gamma config" maxOccurs="1">
                  <creation priority="optional"/>
                  <complexType>
                    <p name="attitude" fullName="The attitude" maxOccurs="1">
                      <simpleType base="integer">
                        <enumeration value="0" text="Disabled"/>
                        <enumeration value="1" text="Enabled"/>
                      </simpleType>
                    </p>
                  </complexType>
                </p>
                <p name="gammaGimmickConfig" fullName="The gamma gimmick config" maxOccurs="9999">
                  <complexType>
                    <p name="dn" fullName="The dn param" maxOccurs="1">
                      <simpleType base="string">
                        <minLength value="0"/>
                        <maxLength value="32767"/>
                      </simpleType>
                    </p>
                    <p name="rate" fullName="The rate param" maxOccurs="1">
                      <simpleType base="integer">
                        <editing units="octets"/>
                      </simpleType>
                    </p>
                    <p name="size" fullName="The size param" maxOccurs="1">
                      <simpleType base="integer"/>
                    </p>
                  </complexType>
                </p>
              </complexType>
            </p>
          </complexType>
        </p>
      </complexType>
    </p>
  </managedObject>
</pdmeta>
'''
