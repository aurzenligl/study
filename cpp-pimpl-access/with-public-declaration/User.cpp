#include <iostream>

// API for user
#include "Context.hpp"

// API for internal user (library implementer)
#include "ContextImpl.hpp"

static void print(const Context& ctx)
{
    std::cout << getImpl(ctx).name << '\n';
}

static void changeName(Context& ctx, const char* newName)
{
    getImpl(ctx).name = newName;
}

int main()
{
    // user API used
    Context ctx("foo");

    // implementer API used
    print(ctx);  // const usage
    changeName(ctx, "bar");  // non-const usage
    print(ctx);  // const usage

    return 0;
}
