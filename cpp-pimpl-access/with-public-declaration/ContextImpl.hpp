#ifndef CONTEXT_IMPL_HPP_
#define CONTEXT_IMPL_HPP_

#include "Context.hpp"
#include <string>

class Context::impl
{
public:
    static Context::impl& get(Context& ctx)
    {
        return *ctx.pimpl_;
    }

    static const Context::impl& get(const Context& ctx)
    {
        return *ctx.pimpl_;
    }

    std::string name;
};

// not implementing getImpl as member function allows to make various decisions by implementer:
//   - const, non-const or both?
//   - getters for whole Context::impl or for smaller parts?

inline Context::impl& getImpl(Context& ctx)
{
    return Context::impl::get(ctx);
}

inline const Context::impl& getImpl(const Context& ctx)
{
    return Context::impl::get(ctx);
}

#endif  // CONTEXT_IMPL_HPP_
