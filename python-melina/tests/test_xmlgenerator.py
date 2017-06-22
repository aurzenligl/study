import os
import melina

datadir = os.path.abspath(__file__ + '/../data')

def parse(filename):
    return melina.XmlParser.from_file(datadir + '/' + filename).parse()

def generate(tu):
    return melina.XmlGenerator(tu).to_string()

class TestGenerator():
    def test_configure(self):
        tu = parse('configure.xml')
        meta = generate(tu)
        assert meta == '''\
<?xml version="1.0" encoding="utf-8"?>
<pdmeta>
  <managedObject class="CONFIGURE_MECHANISM_TASK">
    <childManagedObject class="RESULT"/>
    <p name="alphaDelta" maxOccurs="1">
      <complexType>
        <p name="modified" maxOccurs="999999">
          <complexType>
            <p name="dn" maxOccurs="1">
              <simpleType base="string">
                <minLength value="0"/>
                <maxLength value="2147483647"/>
              </simpleType>
            </p>
            <p name="param" maxOccurs="1">
              <simpleType base="integer"/>
            </p>
          </complexType>
        </p>
      </complexType>
    </p>
    <p name="betaDelta" maxOccurs="1">
      <complexType>
        <p name="added" maxOccurs="999999">
          <complexType>
            <p name="devDn" maxOccurs="1">
              <simpleType base="string">
                <minLength value="0"/>
                <maxLength value="2147483647"/>
              </simpleType>
            </p>
            <p name="id" maxOccurs="1">
              <simpleType base="integer"/>
            </p>
            <p name="param" maxOccurs="1">
              <simpleType base="integer"/>
            </p>
          </complexType>
        </p>
        <p name="modified" maxOccurs="999999">
          <complexType>
            <p name="dn" maxOccurs="1">
              <simpleType base="string">
                <minLength value="0"/>
                <maxLength value="2147483647"/>
              </simpleType>
            </p>
            <p name="param" maxOccurs="1">
              <simpleType base="integer"/>
            </p>
          </complexType>
        </p>
        <p name="removed" maxOccurs="999999">
          <complexType>
            <p name="dn" maxOccurs="1">
              <simpleType base="string">
                <minLength value="0"/>
                <maxLength value="2147483647"/>
              </simpleType>
            </p>
          </complexType>
        </p>
      </complexType>
    </p>
    <p name="gammaDelta" maxOccurs="1">
      <complexType>
        <p name="modified" maxOccurs="999999">
          <complexType>
            <p name="dn" maxOccurs="1">
              <simpleType base="string">
                <minLength value="0"/>
                <maxLength value="2147483647"/>
              </simpleType>
            </p>
            <p name="config" maxOccurs="1">
              <creation priority="optional"/>
              <complexType>
                <p name="param" maxOccurs="1">
                  <creation priority="optional"/>
                  <simpleType base="integer"/>
                </p>
                <p name="gammaConfig" maxOccurs="1">
                  <creation priority="optional"/>
                  <complexType>
                    <p name="attitude" maxOccurs="1">
                      <simpleType base="integer">
                        <enumeration value="0" text="Disabled"/>
                        <enumeration value="1" text="Enabled"/>
                      </simpleType>
                    </p>
                  </complexType>
                </p>
                <p name="gammaGimmickConfig" maxOccurs="999999">
                  <complexType>
                    <p name="dn" maxOccurs="1">
                      <simpleType base="string">
                        <minLength value="0"/>
                        <maxLength value="2147483647"/>
                      </simpleType>
                    </p>
                    <p name="rate" maxOccurs="1">
                      <simpleType base="integer"/>
                    </p>
                    <p name="size" maxOccurs="1">
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
