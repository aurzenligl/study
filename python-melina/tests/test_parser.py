import os
import timeit
import melina

datadir = os.path.abspath(__file__ + '/../data')

def parse(filename):
    return melina.Parser.from_file(datadir + '/' + filename).parse()

class TestParser():
    def test_example(self):
        tu = parse('example.meta')
        assert str(tu) == '''\
mo MACHINE_L
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
            required Float x
            required Float y
        required Int a
        required Int b
    required Int x
    required Int y
'''

    def test_configure(self):
        tu = parse('configure.meta')
        assert str(tu) == '''\
mo CONFIGURE_MECHANISM_TASK
    required struct alphaDelta
        repeated struct modified
            required String dn
            required Int param
    required struct betaDelta
        repeated struct added
            required String devDn
            required Int id
            required Int param
        repeated struct modified
            required String dn
            required Int param
        repeated struct removed
            required String dn
    required struct gammaDelta
        repeated struct modified
            required String dn
            optional struct config
                optional Int param
                optional struct gammaConfig
                    required Int placeholderJustToCompileMeta
                repeated struct gammaGimmickConfig
                    required String dn
'''

    def test_performance(self):
        print timeit.timeit("parse('longexample.meta')", "from %s import parse" % __name__, number=1)
