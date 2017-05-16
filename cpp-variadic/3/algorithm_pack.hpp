#pragma once

#include <type_traits>
#include <utility>

template<typename Test, template<typename...> class Ref>
struct is_specialization : std::false_type {};

template<template<typename...> class Ref, typename... Args>
struct is_specialization<Ref<Args...>, Ref>: std::true_type {};

template <template<class> class MetaPred, typename... Args>
struct meta_count_if;

template <template<class> class MetaPred>
struct meta_count_if<MetaPred>
{
    enum { value = 0 };
};

template <template<class> class MetaPred, typename Arg, typename... Args>
struct meta_count_if<MetaPred, Arg, Args...>
{
    enum { value = MetaPred<Arg>::value + meta_count_if<MetaPred, Args...>::value };
};

template <typename UnaryFun, typename Arg>
void call_if_impl(UnaryFun fun, Arg&& arg, std::true_type)
{
    fun(std::forward<Arg>(arg));
}

template <typename UnaryFun, typename Arg>
void call_if_impl(UnaryFun fun, Arg&& arg, std::false_type)
{}

template <template<class> class MetaPred, typename UnaryFun, typename Arg>
void call_if(UnaryFun fun, Arg&& arg)
{
    call_if_impl(fun, std::forward<Arg>(arg), std::integral_constant<bool, MetaPred<Arg>::value>());
}

template <template<class> class MetaPred, typename UnaryFun>
void for_args_if(UnaryFun fun)
{}

template <template<class> class MetaPred, typename UnaryFun, typename Arg, typename... Args>
void for_args_if(UnaryFun fun, Arg&& arg, Args&&... args)
{
    call_if<MetaPred>(fun, std::forward<Arg>(arg));
    for_args_if<MetaPred>(fun, std::forward<Args>(args)...);
}
