meta recursive-descent parser
----------------------------------------------------------------------

1. [meta -> tu] parser meliny
    - use on examples / test
    - design and implement syntax spec definitions
    - move comments from Tokenizer to Parser
    - handle errors
2. [xml -> tu]
    - rozpoznaj język po rozszerzeniu
    - opcja cmdlinowa + --xml --meta
3. [tu -> xml] generator meliny
    - generuj xml
4. [tu -> meta]
5. [tu -> proto]
6. [tu -> hpp/cpp]

[parsers/generators]
parsers
    meta
    xml
generators
    meta
    xml
    proto
    hpp/cpp

[value sets]
optional int(0..17) mac;  // comment

[compiler errors]
dupa.meta:12:23: expected comma, got xx instead:
mo MACHINE -> a xx, v, c, d
                ^^

[prepare ast]
Mo
    name
    children
    fields (Struct, Enum, Scalar)
Field
    name
    qual
    type
Struct
    name
    fields (...)
Enum
    name
    enumerators (...)
Int
Float

grammar
----------------------------------------------------------------------

specification:
    mo_list end
    end

mo_list:
    mo mo_list
    mo

mo:
    mo_head { field_list } ;

mo_head:
    MO name -> mo_children_list
    MO name

mo_children_list:
    name , mo_children_list
    name

field_list:
    field field_list
    field

field:
    field_qualifier field_definition
    field_definition

field_qualifier:
    repeated
    optional

field_definition:
    struct
    enum
    scalar

struct:
    STRUCT name { field_list } ;

enum:
    ENUM name { enumerator_list } ;

enumerator_list:
    enumerator , enumerator_list
    enumerator

enumerator:
    name = number
    name

scalar:
    scalar_type name ;

scalar_type:
    INT
    FLOAT
    STRING

example
----------------------------------------------------------------------

mo MACHINE_L -> SENSOR, WHEEL, ARM
{
    struct StateBox
    {
        repeated enum FaultStatus { Empty, Disconnected, RoofFlewOff };
        enum AdminStatus { Locked, Unlocked };
        enum AvailStatus { Online, Offline };
    };

    optional struct Core
    {
        repeated enum Types
        {
            T1,
            T2
        };

        /**
         * This is the heart of the machine.
         */
        repeated struct Numbers
        {
            float x;
            float y;
        };

        int a;
        int b;
    };

    int x;  // comment about something
    int y;  // another comment
};
