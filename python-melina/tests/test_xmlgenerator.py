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
    <p name="alphaDelta" maxOccurs="1"/>
    <p name="betaDelta" maxOccurs="1"/>
    <p name="gammaDelta" maxOccurs="1"/>
  </managedObject>
</pdmeta>
'''
