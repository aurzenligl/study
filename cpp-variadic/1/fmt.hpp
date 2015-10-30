template <typename T>
struct fmt_traits;

template <>
struct fmt_traits<int>
{
    static const char value = 'd';
};

template <>
struct fmt_traits<double>
{
    static const char value = 'f';
};

template <>
struct fmt_traits<const char*>
{
    static const char value = 's';
};

template <typename T, typename ...Ts>
struct fmt_impl
{
    template <char ...Chars>
    struct inner
    {
        static constexpr auto& value = fmt_impl<Ts...>::template inner<Chars..., '%', fmt_traits<T>::value, ' '>::value;
    };
};

template <typename T>
struct fmt_impl<T>
{
    template <char ...Chars>
    struct inner
    {
        static constexpr char value[] = { Chars..., '\n', 0 };
    };
};

template <typename T>
template <char ...Chars>
constexpr char fmt_impl<T>::inner<Chars...>::value[];

template <typename ...Ts>
struct fmt
{
    static constexpr auto& value = fmt_impl<Ts..., bool>::template inner<>::value;
};
