#ifndef MO_HPP_
#define MO_HPP_

#include <string>
#include <vector>

struct base
{
    virtual void i_enable_dynamic_casting(){}

    std::string name;
};

struct hwport : public base
{
    base* device;
};

struct dsp : public base
{
    std::vector<hwport*> ports;
    int id;
};

struct bbswitch : public base
{
    std::vector<hwport*> ports;
};

struct connector : public base
{
};

struct hwlink : public base
{
    base* src;
    base* dst;
};

struct cablink : public base
{
    base* src;
    base* dst;
};

#endif /* MO_HPP_ */
