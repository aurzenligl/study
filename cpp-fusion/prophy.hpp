#ifndef PROPHY_HPP
#define PROPHY_HPP

#include <string>
#include <stdint.h>
#include <vector>
#include <type_traits>
#include <sstream>
#include <iostream>

namespace prophy
{

template <typename... Ts>
struct desc
{};

struct field_tag{};
struct len_field_tag{};

template <typename T>
struct is_composite
{
    enum { value = true };
};

template <>
struct is_composite<uint8_t>
{
    enum { value = false };
};

template <>
struct is_composite<uint16_t>
{
    enum { value = false };
};

template <>
struct is_composite<uint32_t>
{
    enum { value = false };
};

template <>
struct is_composite<uint64_t>
{
    enum { value = false };
};

template <typename T>
struct is_range
{
    enum { value = false };
};

template <typename V>
struct is_range<std::vector<V>>
{
    enum { value = true };
};

template <char ...Chars>
struct chars
{
    static constexpr const char value[] = { Chars..., 0 };
};

template <char ...Chars>
constexpr char chars<Chars...>::value[];

template <typename T>
struct get_value_type
{
    typedef T type;
};

template <typename T>
struct get_value_type<std::vector<T>>
{
    typedef T type;
};

template <typename T, typename V, V T::*D, typename Chars, int P>
struct field
{
    typedef T parent_type;
    typedef V type;
    typedef typename get_value_type<V>::type value_type;

    typedef field_tag tag;
    enum { range = is_range<type>::value };
    enum { composite = is_composite<value_type>::value };
    enum { padding = P };

    static type& get(parent_type& t)
    {
        return t.*D;
    }

    static const type& get(const parent_type& t)
    {
        return t.*D;
    }

    static const char* name()
    {
        return Chars::value;
    }
};

template <typename T, typename V, V T::*D, typename L, int P>
struct len_field
{
    typedef len_field_tag tag;
};

inline std::ostream& put_spaces(std::ostream& out, int count)
{
    while (count)
    {
        out << "  ";
        --count;
    }
    return out;
}

template <typename T, typename Ctx>
void print_scalar(const char* name, const T& value, Ctx& ctx)
{
    put_spaces(ctx.out, ctx.indent) << name << ": " << value << '\n';
}

template <typename T, typename Ctx>
void print_composite(const char* name, const T& value, Ctx& ctx)
{
    put_spaces(ctx.out, ctx.indent) << name << " {\n";
    ctx.indent++;
    value._print(ctx);
    ctx.indent--;
    put_spaces(ctx.out, ctx.indent) << "}\n";
}

struct Printer
{
    struct Ctx
    {
        std::ostream& out;
        int indent;
    };

    template <typename Field>
    static void scalar(Ctx& ctx, const typename Field::type& value)
    {
        print_scalar(Field::name(), value, ctx);
    }

    template <typename Field>
    static void scalar_range(Ctx& ctx, const typename Field::type& value)
    {
        for (const typename Field::value_type& x: value)
        {
            print_scalar(Field::name(), x, ctx);
        }
    }

    template <typename Field>
    static void scalar_optional(Ctx& ctx, const typename Field::type& value)
    {
    }

    template <typename Field>
    static void composite(Ctx& ctx, const typename Field::type& value)
    {
        print_composite(Field::name(), value, ctx);
    }

    template <typename Field>
    static void composite_range(Ctx& ctx, const typename Field::type& value)
    {
        for (const typename Field::value_type& x: value)
        {
            print_composite(Field::name(), x, ctx);
        }
    }

    template <typename Field>
    static void composite_optional(Ctx& ctx, const typename Field::type& value)
    {
    }

    template <typename Field>
    static void len(Ctx& ctx, const typename Field::type& value)
    {
    }
};

template <int Padding, typename Ctx>
typename std::enable_if<Padding < 0, void>::type pad(Ctx& ctx)
{
    // TODO
}

template <int Padding, typename Ctx>
typename std::enable_if<Padding >= 0, void>::type pad(Ctx& ctx)
{
    ctx.pos += Padding;
}

template <typename T, typename Ctx>
inline void encode_int(Ctx& ctx, T in)
{
    *reinterpret_cast<T*>(ctx.pos) = in;
    ctx.pos += sizeof(T);
}

struct Encoder
{
    struct Ctx
    {
        uint8_t* pos;
    };

    template <typename Field>
    static void scalar(Ctx& ctx, const typename Field::type& value)
    {
        encode_int(ctx, value);
        pad<Field::padding>(ctx);
    }

    template <typename Field>
    static void scalar_range(Ctx& ctx, const typename Field::type& value)
    {
        for (const typename Field::value_type& x: value)
        {
            encode_int(ctx, x);
        }
        pad<Field::padding>(ctx);
    }

    template <typename Field>
    static void scalar_optional(Ctx& ctx, const typename Field::type& value)
    {
    }

    template <typename Field>
    static void composite(Ctx& ctx, const typename Field::type& value)
    {
        value._encode(ctx);
        pad<Field::padding>(ctx);
    }

    template <typename Field>
    static void composite_range(Ctx& ctx, const typename Field::type& value)
    {
        for (const typename Field::value_type& x: value)
        {
            x._encode(ctx);
        }
        pad<Field::padding>(ctx);
    }

    template <typename Field>
    static void composite_optional(Ctx& ctx, const typename Field::type& value)
    {
    }

    template <typename Field>
    static void len(Ctx& ctx, const typename Field::type& value)
    {
    }
};

template <typename Algorithm, typename Field, typename T>
typename std::enable_if<
    std::is_same<typename Field::tag, len_field_tag>::value,
void>::type dispatch_field(typename Algorithm::Ctx& ctx, const T& t)
{
}

template <typename Algorithm, typename Field, typename T>
typename std::enable_if<
    std::is_same<typename Field::tag, field_tag>::value and not Field::composite and not Field::range,
void>::type dispatch_field(typename Algorithm::Ctx& ctx, const T& t)
{
    Algorithm::template scalar<Field>(ctx, Field::get(t));
}

template <typename Algorithm, typename Field, typename T>
typename std::enable_if<
    std::is_same<typename Field::tag, field_tag>::value and not Field::composite and Field::range,
void>::type dispatch_field(typename Algorithm::Ctx& ctx, const T& t)
{
    Algorithm::template scalar_range<Field>(ctx, Field::get(t));
}

template <typename Algorithm, typename Field, typename T>
typename std::enable_if<
    std::is_same<typename Field::tag, field_tag>::value and Field::composite and not Field::range,
void>::type dispatch_field(typename Algorithm::Ctx& ctx, const T& t)
{
    Algorithm::template composite<Field>(ctx, Field::get(t));
}

template <typename Algorithm, typename Field, typename T>
typename std::enable_if<
    std::is_same<typename Field::tag, field_tag>::value and Field::composite and Field::range,
void>::type dispatch_field(typename Algorithm::Ctx& ctx, const T& t)
{
    Algorithm::template composite_range<Field>(ctx, Field::get(t));
}

template <typename Algorithm, typename T>
void dispatch_iter(typename Algorithm::Ctx& ctx, const T& t)
{}

template <typename Algorithm, typename T, typename Field, typename ...Fields>
void dispatch_iter(typename Algorithm::Ctx& ctx, const T& t)
{
    dispatch_field<Algorithm, Field>(ctx, t);
    dispatch_iter<Algorithm, T, Fields...>(ctx, t);
}

template <typename Algorithm, typename T, typename ...Fields>
void dispatch_exec(typename Algorithm::Ctx& ctx, const T& t, desc<Fields...>)
{
    dispatch_iter<Algorithm, T, Fields...>(ctx, t);
}

template <typename Algorithm, typename T>
void dispatch(typename Algorithm::Ctx& ctx, const T& t)
{
    dispatch_exec<Algorithm>(ctx, t, typename T::_desc());
}

template <typename T>
struct message
{
    void _encode(Encoder::Ctx& ctx) const
    {
        dispatch<Encoder>(ctx, *static_cast<const T*>(this));
    }

    size_t encode(void* data)
    {
        uint8_t* pos = static_cast<uint8_t*>(data);
        Encoder::Ctx ctx{pos};
        _encode(ctx);
        return ctx.pos - pos;
    }

    void _print(Printer::Ctx& ctx) const
    {
        dispatch<Printer>(ctx, *static_cast<const T*>(this));
    }

    std::string print() const
    {
        std::stringstream ss;
        Printer::Ctx ctx{ss, 0};
        _print(ctx);
        return ss.str();
    }
};

}

#endif // PROPHY_HPP
