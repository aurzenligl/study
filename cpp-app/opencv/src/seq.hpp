#ifndef CPP_APP_SRC_SEQ_HPP_
#define CPP_APP_SRC_SEQ_HPP_

#include <array>
#include <stdint.h>

namespace app
{

template <unsigned... Is>
struct seq{};

template <unsigned N, unsigned... Is>
struct gen_seq : gen_seq<N-1, N-1, Is...>{};

template <unsigned... Is>
struct gen_seq<0, Is...> : seq<Is...>{};

template <unsigned N, typename UnaryOp, unsigned... Is>
constexpr std::array<uint8_t, N> make_lut_(seq<Is...>)
{
    return {{ UnaryOp()(Is)... }};
}

template <unsigned N, typename UnaryOp>
constexpr std::array<uint8_t, N> make_lut()
{
    return make_lut_<N, UnaryOp>(gen_seq<N>());
}

}

#endif /* CPP_APP_SRC_SEQ_HPP_ */
