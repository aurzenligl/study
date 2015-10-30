#ifndef PARSE_TUPLE_HPP_
#define PARSE_TUPLE_HPP_

#include <Python.h>

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

template <typename ...Ts>
inline bool parse_tuple(PyObject* args, Ts&... values)
{
    return PyArg_ParseTuple(args, parse_tuple_fmt<Ts...>::value, &values...);
}

#endif /* PARSE_TUPLE_HPP_ */
