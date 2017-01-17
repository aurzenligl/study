#ifndef CONTEXT_HPP_
#define CONTEXT_HPP_

#include <memory>

class Context
{
public:
    // It's important to have this declaration public.
    // This allows simple access without inline friend and inheritance.
    class impl;

    Context(const char* name);
    ~Context();

private:
    std::unique_ptr<impl> pimpl_;
};

#endif  // CONTEXT_HPP_
