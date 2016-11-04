#include <string>

struct Ctx
{
    Ctx(){}
    ~Ctx(){}
private:
    struct impl;
    impl* impl_;
};

struct CtxImpl
{
    /// actual implementation, internally (in lib) accessible

    static CtxImpl& fromCtx(Ctx& ctx);
};

struct Ctx::impl : public CtxImpl
{
    static CtxImpl& toCtxImpl(Ctx& ctx)
    {
        return *ctx.impl_;
    }
    friend CtxImpl& toCtxImpl(Ctx& ctx)
    {
        return impl::toCtxImpl(ctx);
    }
};

CtxImpl& toCtxImpl(Ctx& ctx);

CtxImpl& CtxImpl::fromCtx(Ctx& ctx)
{
    return toCtxImpl(ctx);
}

Ctx ctx;
CtxImpl& impl = CtxImpl::fromCtx(ctx);

int main()
{
}
