#ifndef MESSAGES_HPP_
#define MESSAGES_HPP_

#include "prophy.hpp"

struct Ra : prophy::message<Ra>
{
    uint16_t ra = 5;

    typedef prophy::desc<
        prophy::field<Ra, uint16_t, &Ra::ra, prophy::chars<'r', 'a'>, 0>
    > _desc;
};

struct Foo : prophy::message<Foo>
{
    uint16_t foo;
    uint32_t pod = 12;
    Ra ra;

    typedef prophy::desc<
        prophy::field<Foo, uint16_t, &Foo::foo, prophy::chars<'f', 'o', 'o'>, 0>,
        prophy::field<Foo, uint32_t, &Foo::pod, prophy::chars<'p', 'o', 'd'>, 0>,
    prophy::field<Foo, Ra, &Foo::ra, prophy::chars<'r', 'a'>, 0>
    > _desc;
};

struct Bar : prophy::message<Bar>
{
    uint64_t x;
    Foo y;
    std::vector<uint32_t> z;

    typedef prophy::desc<
        prophy::field<Bar, uint64_t, &Bar::x, prophy::chars<'x'>, 0>,
        prophy::field<Bar, Foo, &Bar::y, prophy::chars<'y'>, 2>,
        prophy::len_field<Bar, std::vector<uint32_t>, &Bar::z, uint32_t, 0>,
        prophy::field<Bar, std::vector<uint32_t>, &Bar::z, prophy::chars<'z'>, -8>
    > _desc;
};

struct Baz : prophy::message<Baz>
{
    std::vector<Foo> baz;

    typedef prophy::desc<
        prophy::field<Baz, std::vector<Foo>, &Baz::baz, prophy::chars<'b', 'a', 'z'>, 0>
    > _desc;
};

#endif /* MESSAGES_HPP_ */
