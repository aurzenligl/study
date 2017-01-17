#include "ContextImpl.hpp"

Context::Context(const char* name) : pimpl_(new impl{name})
{}

Context::~Context()
{}
