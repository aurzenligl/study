#include <iostream>
#include "graph.hpp"
#include "mo.hpp"
#include "moutil.hpp"
#include "user_who_wants_to_mock_away.hpp"

void print(const graph& gr)
{
    for (const graph::node& node : gr.nodes_)
    {
        std::cout << node.mo->name << '\n';
        for (const graph::node* neighbor : node.neighbors)
        {
            std::cout << "    " << neighbor->mo->name << '\n';
        }
    }
}

void print(const base* b)
{
    if (b)
    {
        std::cout << "found: " << b->name << '\n';
    }
    else
    {
        std::cout << "not found" << '\n';
    }
}

void print(const dsp* d)
{
    if (d)
    {
        std::cout << "found dsp: " << d->name << ' ' << d->id << '\n';
    }
    else
    {
        std::cout << "not found" << '\n';
    }
}

void print(const std::vector<const base*> path)
{
    for (const base* b : path)
    {
        std::cout << "path: " << b->name << ": ";
        if (const dsp* x = dynamic_cast<const dsp*>(b))
        {
            std::cout << "dsp: " << x->id;
        }
        else if (const bbswitch* x = dynamic_cast<const bbswitch*>(b))
        {
            std::cout << "bbswitch: " << "number of ports: " << x->ports.size();
        }
        else if (const hwport* x = dynamic_cast<const hwport*>(b))
        {
            std::cout << "hwport: " << "parent device: " << x->device->name;
        }
        else if (const connector* x = dynamic_cast<const connector*>(b))
        {
            (void)x;
            std::cout << "connector: " << "nothing special";
        }
        else
        {
            std::cout << "unknown node";
        }
        std::cout << '\n';
    }
}

auto by_name = [](const char* name)
{
    return [=](const base& b)
    {
        return b.name == name;
    };
};

auto by_dsp_id = [](int id)
{
    return [=](const dsp& d)
    {
        return d.id == id;
    };
};

auto by_ptr = [](const base* expected)
{
    return [=](const base& x)
    {
        return &x == expected;
    };
};

struct pathfinder_impl : public pathfinder
{
    explicit pathfinder_impl(graph& g) : gr(g) {}

    std::vector<const base*> find_path(const base* start, const base* finish)
    {
        return gr.find_path(start, by_ptr(finish));
    }

    graph& gr;
};

struct dspfinder_impl : public dspfinder
{
    explicit dspfinder_impl(graph& g) : gr(g) {}

    const dsp* find_dsp(const base* start, int dsp_id)
    {
        return gr.find<dsp>(start, by_dsp_id(dsp_id));
    }

    graph& gr;
};

int main()
{
    // p1    p4 -- p1    p4 -\   p1    p6 -- p1    p6                p1    p6
    //    d1          d2      \- p2 b1 p5 -- p2 b2 p5 -- c1 -- c2 -- p2 b3 p5
    // p2    p3 -- p2    p3      p3    p4 -- p3    p4                p3    p4

    complete_dsp d1("/D1", 111);
    complete_dsp d2("/D2", 222);
    complete_bbswitch b1("/B1");
    complete_bbswitch b2("/B2");
    complete_bbswitch b3("/B3");
    connector c1 = make<connector>("/C1");
    connector c2 = make<connector>("/C2");

    hwlink l1 = make_hwlink("/L1", d1.p4, d2.p1);
    hwlink l2 = make_hwlink("/L2", d1.p3, d2.p2);
    hwlink l3 = make_hwlink("/L3", d2.p4, b1.p2);
    hwlink l4 = make_hwlink("/L4", b1.p6, b2.p1);
    hwlink l5 = make_hwlink("/L5", b1.p5, b2.p2);
    hwlink l6 = make_hwlink("/L6", b1.p4, b2.p3);
    hwlink l7 = make_hwlink("/L7", b2.p5, c1);
    hwlink l8 = make_hwlink("/L8", c2, b3.p2);
    cablink cl1 = make_cablink("/CL1", c1, c2);

    graph gr;
    gr.add_device(&d1.d);
    gr.add_device(&d2.d);
    gr.add_device(&b1.b);
    gr.add_device(&b2.b);
    gr.add_device(&b3.b);
    gr.add_connector(&c1);
    gr.add_connector(&c2);
    gr.link(&l1);
    gr.link(&l2);
    gr.link(&l3);
    gr.link(&l4);
    gr.link(&l5);
    gr.link(&l6);
    gr.link(&l7);
    gr.link(&l8);
    gr.link(&cl1);

    print(gr);
    print(gr.find_path(&b1.b, by_ptr(&c2)));
    print(gr.find(&d1.p1, by_name("/D2/P3")));
    print(gr.find_path(&d1.p1, by_name("/B3/P6")));
    print(gr.find<dsp>(&b3.b, by_dsp_id(222)));
    print(gr.find_path<dsp>(&b3.p5, by_dsp_id(111)));

    pathfinder_impl pfi(gr);
    dspfinder_impl dfi(gr);
    you_can_unit_test_me_easily ycutme(pfi, dfi);
    std::cout << "ycutme.foo = " << ycutme.foo(&d1.d, &b3.b) << '\n';
    std::cout << "ycutme.bar = " << ycutme.bar(&d1.d, 222) << '\n';

    return 0;
}
