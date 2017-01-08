#ifndef MESSAGES_HPP_
#define MESSAGES_HPP_

#include "prophy.hpp"

struct Foo : prophy::message<Foo>
{
    uint16_t foo;

    struct _names
    {
        static const char foo[];
    };

    typedef prophy::desc<
        prophy::field<Foo, uint16_t, &Foo::foo, _names::foo, 0>
    > _desc;
};

struct Bar : prophy::message<Bar>
{
    uint64_t x;
    Foo y;
    std::vector<uint32_t> z;

    struct _names
    {
        static const char x[];
        static const char y[];
        static const char z[];
    };

    typedef prophy::desc<
        prophy::field<Bar, uint64_t, &Bar::x, _names::x, 0>,
        prophy::field<Bar, Foo, &Bar::y, _names::y, 2>,
        prophy::len_field<Bar, std::vector<uint32_t>, &Bar::z, uint32_t, 0>,
        prophy::field<Bar, std::vector<uint32_t>, &Bar::z, _names::z, -8>
    > _desc;
};

#endif /* MESSAGES_HPP_ */
