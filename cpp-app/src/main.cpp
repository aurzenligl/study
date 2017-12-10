#include <iostream>
#include "opts.hpp"
#include "process.hpp"

int main(int ac, char* av[])
{
    app::cmdline_opts opts;
    if (app::cmdline_status st = app::parse(ac, av, opts))
    {
        return st.return_code;
    }

    std::cout << "Operation " << opts.operation << " set.\n";
    std::cout << "File " << opts.filename << " set.\n";

    switch (opts.operation)
    {
    case 0:
        app::show_image(opts.filename);
        break;
    case 1:
        app::convert_image(opts.filename);
        break;
    case 2:
        app::reduce_color_space(opts.filename);
        break;
    case 3:
        app::sharpen(opts.filename);
        break;
    default:
        std::cout << "Operation " << opts.operation << " has no action.\n";
        break;
    }
}
