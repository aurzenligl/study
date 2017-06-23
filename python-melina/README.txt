meta recursive-descent parser
----------------------------------------------------------------------

xml -> tu: [driver] implement input/output logic (--meta/xml-(std)out, --meta, --xml)

tu,
meta -> tu,
tu -> meta,
xml -> tu,
tu -> xml: [spec/parser/generator] solve langfeature todos

[melina cmdline]
// stdout - only one type of output possible
melina --meta-stdout text.meta
melina --xml-stdout text.xml

[reciprocity tests]
check if all meta/xml inputs parse/generate the same way to/from meta/xml

[value sets]
optional int(0..17) mac;  // comment
