#ifndef MOUTIL_HPP_
#define MOUTIL_HPP_

#include <string>
#include <vector>
#include "mo.hpp"

void vacuum(...)
{ }

template <typename T>
T make(const std::string& name)
{
    T t;
    t.name = name;
    return t;
}

template <typename Dev, typename... Args>
void link_device(Dev& dev, Args& ...ports)
{
    vacuum((dev.ports.push_back(&ports), 0)...);
    vacuum((ports.device = &dev, 0)...);
}

struct complete_dsp
{
    complete_dsp(const std::string& name, int id)
    {
        p1 = make<hwport>(name + "/P1");
        p2 = make<hwport>(name + "/P2");
        p3 = make<hwport>(name + "/P3");
        p4 = make<hwport>(name + "/P4");
        d = make<dsp>(name);
        d.id = id;
        link_device(d, p4, p3, p2, p1);
    }

    hwport p1;
    hwport p2;
    hwport p3;
    hwport p4;
    dsp d;
};

struct complete_bbswitch
{
    complete_bbswitch(const std::string& name)
    {
        p1 = make<hwport>(name + "/P1");
        p2 = make<hwport>(name + "/P2");
        p3 = make<hwport>(name + "/P3");
        p4 = make<hwport>(name + "/P4");
        p5 = make<hwport>(name + "/P5");
        p6 = make<hwport>(name + "/P6");
        b = make<bbswitch>(name);
        link_device(b, p6, p5, p4, p3, p2, p1);
    }

    hwport p1;
    hwport p2;
    hwport p3;
    hwport p4;
    hwport p5;
    hwport p6;
    bbswitch b;
};

hwlink make_hwlink(const char* name, base& src, base& dst)
{
    hwlink l;
    l.name = name;
    l.src = &src;
    l.dst = &dst;
    return l;
}

cablink make_cablink(const char* name, base& src, base& dst)
{
    cablink l;
    l.name = name;
    l.src = &src;
    l.dst = &dst;
    return l;
}

#endif /* MOUTIL_HPP_ */
