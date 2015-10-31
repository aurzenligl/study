#ifndef PARSE_TUPLE_HPP_
#define PARSE_TUPLE_HPP_

#include <tuple>
#include <Python.h>
#include "call.hpp"
#include "npywrap.hpp"

namespace detail
{

template <typename T>
struct parse_tuple_traits;

template <>
struct parse_tuple_traits<double>
{
    static const char value = 'd';
};

template <>
struct parse_tuple_traits<PyObject*>
{
    static const char value = 'O';
};

template <typename T>
struct parse_tuple_traits<npyarray<T>>
{
    static const char value = 'O';
};

template <typename T, typename ...Ts>
struct parse_tuple_fmt_impl
{
    template <char ...Chars>
    struct inner
    {
        static constexpr auto& value = parse_tuple_fmt_impl<Ts...>::template inner<Chars..., parse_tuple_traits<T>::value>::value;
    };
};

template <typename T>
struct parse_tuple_fmt_impl<T>
{
    template <char ...Chars>
    struct inner
    {
        static constexpr char value[] = { Chars..., 0 };
    };
};

template <typename T>
template <char ...Chars>
constexpr char parse_tuple_fmt_impl<T>::inner<Chars...>::value[];

template <typename ...Ts>
struct parse_tuple_fmt
{
    static constexpr auto& value = parse_tuple_fmt_impl<Ts..., bool>::template inner<>::value;
};

template <typename T>
T* ptr(T& t, PyObject**&)
{
    return &t;
}

template <typename T>
PyObject** ptr(npyarray<T>&, PyObject**& it)
{
    return it++;
}

template <typename P, typename T>
bool parse(P*, T&)
{
    return true;
}

template <typename T>
bool parse(PyObject** obj, npyarray<T>& value)
{
    npyarray<T>(*obj).swap(value);
    return !!value;
}

template <int N = 0, typename Tuple>
bool parse_npy(Tuple&& t)
{
    return true;
}

template <int N = 0, typename Tuple, typename T, typename ...Ts>
bool parse_npy(Tuple&& t, T& value, Ts&... values)
{
    return parse(std::get<N>(t), value) && parse_npy<N + 1>(t, values...);
}

}  // namespace detail

template <typename ...Ts>
inline bool parse_tuple(PyObject* args, Ts&... values)
{
    using namespace detail;

    PyObject* objs[sizeof...(Ts)] = {};
    PyObject** it = objs;
    auto preamble = std::make_pair(args, parse_tuple_fmt<Ts...>::value);
    auto parseptrs = std::make_tuple(ptr(values, it)...);
    if (call(PyArg_ParseTuple, std::tuple_cat(preamble, parseptrs)))
    {
        return parse_npy(parseptrs, values...);
    }
    return true;
}

#endif /* PARSE_TUPLE_HPP_ */
