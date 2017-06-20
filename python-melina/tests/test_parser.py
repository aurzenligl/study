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
            required float x
            required float y
        required int a
        required int b
    required int x
    required int y
'''

    def test_configure(self):
        tu = parse('configure.meta')
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

    def hardskip_test_performance(self):
        print timeit.timeit("parse('longexample.meta')", "from %s import parse" % __name__, number=1)
