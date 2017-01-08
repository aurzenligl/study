#ifndef MESSAGES_HPP_
#define MESSAGES_HPP_

#include "prophy.hpp"

struct Foo : prophy::message<Foo>
{
    uint16_t foo;

    typedef prophy::desc<
        prophy::field<Foo, uint16_t, &Foo::foo, prophy::chars<'f', 'o', 'o'>, 0>
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

#endif /* MESSAGES_HPP_ */
