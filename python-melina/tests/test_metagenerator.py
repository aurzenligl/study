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
/// pdmeta: "1.7", domain: "foo", product: "bar", release: "AX-19.2", version: "1.5.3.0.beta", revision: "1982713"

/**
 * This is an example managed object: The Machine.
 */
mo(cd) MACHINE_L -> SENSOR(1), WHEEL(12), ARM
{
    struct stateBox
    {
        repeated enum faultStatus
        {
            Empty = 0,
            Disconnected = 1,
            RoofFlewOff = 2
        };

        enum adminStatus
        {
            Locked = 0,
            Unlocked = 1
        };

        /**
         * Enum can be documented too.
         */
        enum availStatus [default = 1]
        {
            Online = 0,
            Offline = 1
        };
    };

    optional struct core
    {
        repeated enum types
        {
            T1 = 1,
            T2 = 2,
            42Val = 3,
            000 = 4
        };

        /**
         * This is the heart of the machine.
         */
        repeated struct numbers
        {
            int x [default = 0];
            int(-12..42) xx [default = -4];
            int(-0.0001, 0.0002, 0.0000001) xxx [default = -0.00007];
            int(-1, 1, 0.01) xxxx [default = 0, units = "bazes"];
            string y;
            string(2..15) yy;
            bool z [default = true];
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
/// pdmeta: "", domain: "", product: "", release: "", version: "", revision: ""

mo CONFIGURE_MECHANISM_TASK -> RESULT
{
    struct alphaDelta
    {
        repeated struct modified
        {
            string dn [default = ""];
            int param;
        };
    };

    struct betaDelta
    {
        repeated struct added
        {
            string devDn [default = "foo-bar"];
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
