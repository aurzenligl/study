import os
import timeit
import melina

datadir = os.path.abspath(__file__ + '/../data')

def parse(filename):
    return melina.XmlParser.from_file(datadir + '/' + filename).parse()

class TestParser():
    def test_configure(self):
        tu = parse('configure.xml')
        assert str(tu) == '''\
mo CONFIGURE_MECHANISM_TASK
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
