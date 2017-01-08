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
{
};

struct field_tag{};
struct len_field_tag{};

template <typename T>
struct is_composite
{
    enum { value = true };
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

template <typename T>
void print_scalar(const char* name, const T& value, std::ostream& out, int indent)
{
    put_spaces(out, indent) << name << ": " << value << '\n';
}

template <typename T>
void print_composite(const char* name, const T& value, std::ostream& out, int indent)
{
    put_spaces(out, indent) << name << " {\n";
    value._print(out, indent + 1);
    put_spaces(out, indent) << "}\n";
}

template <typename Field, typename T>
typename std::enable_if<
    not std::is_same<typename Field::tag, field_tag>::value,
void>::type print(std::ostream& out, int indent, const T& t)
{
}

template <typename Field, typename T>
typename std::enable_if<
    std::is_same<typename Field::tag, field_tag>::value and Field::range and not Field::composite,
void>::type print(std::ostream& out, int indent, const T& t)
{
    for (const typename Field::type::value_type& x : Field::get(t))
    {
        print_scalar(Field::name(), x, out, indent);
    }
}

template <typename Field, typename T>
typename std::enable_if<
    std::is_same<typename Field::tag, field_tag>::value and Field::range and Field::composite,
void>::type print(std::ostream& out, int indent, const T& t)
{
    for (const typename Field::type::value_type& x : Field::get(t))
    {
        print_composite(Field::name(), x, out, indent);
    }
}

template <typename Field, typename T>
typename std::enable_if<
    std::is_same<typename Field::tag, field_tag>::value and not Field::range and Field::composite,
void>::type print(std::ostream& out, int indent, const T& t)
{
    print_composite(Field::name(), Field::get(t), out, indent);
}

template <typename Field, typename T>
typename std::enable_if<
std::is_same<typename Field::tag, field_tag>::value and not Field::range and not Field::composite,
void>::type print(std::ostream& out, int indent, const T& t)
{
    print_scalar(Field::name(), Field::get(t), out, indent);
}

template <typename T>
void print_iter(std::ostream& out, int indent, const T& t)
{}

template <typename T, typename Field, typename ...Ts>
void print_iter(std::ostream& out, int indent, const T& t)
{
    print<Field>(out, indent, t);
    print_iter<T, Ts...>(out, indent, t);
}

template <typename T, typename ...Fields>
void print_exec(std::ostream& out, int indent, const T& t, desc<Fields...>)
{
    print_iter<T, Fields...>(out, indent, t);
}

template <int Padding>
typename std::enable_if<
    Padding < 0,
uint8_t*>::type pad(uint8_t* out)
{
    // TODO
    return out;
}

template <int Padding>
typename std::enable_if<
    Padding >= 0,
uint8_t*>::type pad(uint8_t* out)
{
    return out + Padding;
}

template <typename T>
inline uint8_t* encode_int(uint8_t* out, T in)
{
    *reinterpret_cast<T*>(out) = in;
    return out + sizeof(T);
}

template <int Padding, typename T>
inline uint8_t* encode_scalar(uint8_t* pos, const T& in)
{
    return pad<Padding>(encode_int(pos, in));
}

template <int Padding, typename T>
inline uint8_t* encode_composite(uint8_t* pos, const T& in)
{
    return pad<Padding>(in._encode(pos));
}

template <typename Field, typename T>
typename std::enable_if<
    std::is_same<typename Field::tag, len_field_tag>::value,
uint8_t*>::type encode(uint8_t* pos, const T& t)
{
    // TODO
    return pos;
}

template <typename Field, typename T>
typename std::enable_if<
    std::is_same<typename Field::tag, field_tag>::value and not Field::range and not Field::composite,
uint8_t*>::type encode(uint8_t* pos, const T& t)
{
    return encode_scalar<Field::padding>(pos, Field::get(t));
}

template <typename Field, typename T>
typename std::enable_if<
    std::is_same<typename Field::tag, field_tag>::value and Field::range and not Field::composite,
uint8_t*>::type encode(uint8_t* pos, const T& t)
{
    // TODO
    return pos;
}

template <typename Field, typename T>
typename std::enable_if<
    std::is_same<typename Field::tag, field_tag>::value and not Field::range and Field::composite,
uint8_t*>::type encode(uint8_t* pos, const T& t)
{
    return encode_composite<Field::padding>(pos, Field::get(t));
}

template <typename Field, typename T>
typename std::enable_if<
    std::is_same<typename Field::tag, field_tag>::value and Field::range and Field::composite,
uint8_t*>::type encode(uint8_t* pos, const T& t)
{
    // TODO
    return pos;
}

template <typename T>
uint8_t* encode_iter(uint8_t* pos, const T& t)
{
    return pos;
}

template <typename T, typename Field, typename ...Ts>
uint8_t* encode_iter(uint8_t* pos, const T& t)
{
    uint8_t* mid = encode<Field>(pos, t);
    return encode_iter<T, Ts...>(mid, t);
}

template <typename T, typename ...Fields>
uint8_t* encode_exec(uint8_t* pos, const T& t, desc<Fields...>)
{
    return encode_iter<T, Fields...>(pos, t);
}

template <typename T>
struct message
{
    uint8_t* _encode(uint8_t* pos) const
    {
        return encode_exec(pos, *static_cast<const T*>(this), typename T::_desc());
    }

    size_t encode(void* data)
    {
        uint8_t* pos = static_cast<uint8_t*>(data);
        uint8_t* end = _encode(pos);
        return end - pos;
    }

    void _print(std::ostream& out, int indent) const
    {
        print_exec(out, indent, *static_cast<const T*>(this), typename T::_desc());
    }

    std::string print() const
    {
        std::stringstream ss;
        _print(ss, 0);
        return ss.str();
    }
};

}

#endif // PROPHY_HPP
