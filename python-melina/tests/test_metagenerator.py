import os
import melina

datadir = os.path.abspath(__file__ + '/../data')

def parse(filename):
    return melina.MetaParser.from_file(datadir + '/' + filename).parse()

def generate(tu):
    return melina.MetaGenerator(tu).to_string()

class TestGenerator():
    def test_example(self):
        tu = parse('example.meta')
        meta = generate(tu)
        assert meta == '''\
/**
 * This is an example managed object: The Machine.
 */
mo MACHINE_L -> SENSOR, WHEEL, ARM
{
    struct StateBox
    {
        repeated enum FaultStatus
        {
            Empty = 0,
            Disconnected = 1,
            RoofFlewOff = 2
        };

        enum AdminStatus
        {
            Locked = 0,
            Unlocked = 1
        };

        /**
         * Enum can be documented too.
         */
        enum AvailStatus
        {
            Online = 0,
            Offline = 1
        };
    };

    optional struct Core
    {
        repeated enum Types
        {
            T1 = 1,
            T2 = 2,
            42Val = 3,
            000 = 4
        };

        /**
         * This is the heart of the machine.
         */
        repeated struct Numbers
        {
            int x;
            int(-12..42) xx;
            int(-0.0001, 0.0002, 0.0000001) xxx;
            string y;
            string(2..15) yy;
        };

        int a [units = ""];
        int b [units = "petabytes"];
    };

    repeated(42) bool x;  /// comment about something
    int y;  /// another comment
};
'''

    def test_configure(self):
        tu = parse('configure.meta')
        meta = generate(tu)
        assert meta == '''\
mo CONFIGURE_MECHANISM_TASK -> RESULT
{
    struct alphaDelta
    {
        repeated struct modified
        {
            string dn;
            int param;
        };
    };

    struct betaDelta
    {
        repeated struct added
        {
            string devDn;
            int id;
            int param;
        };

        repeated struct modified
        {
            string dn;
            int param;
        };

        repeated struct removed
        {
            string dn;
        };
    };

    /**
     * Gamma can be explained here. There is no added/removed as Gamma Software deployment/hardware decide how many gammas exist in given execution.
     */
    struct gammaDelta
    {
        repeated struct modified
        {
            string dn;

            optional struct config
            {
                optional int param;

                optional struct gammaConfig
                {
                    enum attitude
                    {
                        Disabled = 0,
                        Enabled = 1
                    };
                };

                repeated struct gammaGimmickConfig
                {
                    string dn;
                    int rate;
                    int size;
                };
            };
        };
    };
};
'''
