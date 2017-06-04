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
