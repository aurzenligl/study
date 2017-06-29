meta recursive-descent parser
----------------------------------------------------------------------

tu,
meta -> tu,
tu -> meta,
xml -> tu,
tu -> xml: [spec/parser/generator] solve langfeature todos

[int units]
<editing units="bananas">

[default value]
<default value="0"/>

[mo child count]
<childManagedObject class="X" maxOccurs="1"/>

[mo flags]
hidden="false" create="true" update="true" delete="true"

[reciprocity tests]
check if all meta/xml inputs parse/generate the same way to/from meta/xml

study -> ownrepo: [repoize melina]
