meta recursive-descent parser
----------------------------------------------------------------------

tu,
meta -> tu,
tu -> meta,
xml -> tu,
tu -> xml: [spec/parser/generator] solve langfeature todos

[default value]
<default value="0"/>

[mo child count]
<childManagedObject class="X" maxOccurs="1"/>

[mo flags]
hidden="false" create="true" update="true" delete="true"

[header]
/// pdmeta: 2.2, domain: asf, product: okas, release: aoka, version: 1022.3, revision: $Revision$

[reciprocity tests]
check if all meta/xml inputs parse/generate the same way to/from meta/xml

study -> ownrepo: [repoize melina]
