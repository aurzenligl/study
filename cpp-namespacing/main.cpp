#include <stdio.h>
#include "x.h"

int main() {
    Get<tag::Abc>();
    Set<tag::Abc>("xyz");

    Get<tag::Def>("left");
    Set<tag::Def>("left", "xyz");

    Get<tag::Ghi>("left", 32);
    Set<tag::Ghi>("left", 12, 819);
}
