meta recursive-descent parser
----------------------------------------------------------------------

tu -> xml: [generator] generate xml
xml -> tu: [driver] implement input/output logic (--meta/xml-(std)out, --meta, --xml)

tu,
meta -> tu,
tu -> meta,
xml -> tu,
tu -> xml: [spec/parser/generator] solve langfeature todos

[melina cmdline]

// just check syntax when no output specified
melina text.meta

// figure out input type by extension
melina --meta-out=. text.meta
melina --xml-out=. text.xml

// when extension is not xml or meta, user must provide it explicitly
melina --meta-out=. --meta text
melina --meta-out=. --xml text.ext

// stdout - only one type of output possible
melina --meta-stdout text.meta
melina --xml-stdout text.xml

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
