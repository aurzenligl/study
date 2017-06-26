meta recursive-descent parser
----------------------------------------------------------------------

tu,
meta -> tu,
tu -> meta,
xml -> tu,
tu -> xml: [spec/parser/generator] solve langfeature todos

[reciprocity tests]
check if all meta/xml inputs parse/generate the same way to/from meta/xml

[value sets]
optional int(0..17) mac;  // comment
