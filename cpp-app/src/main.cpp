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

    std::cout << "Compression level " << opts.compression << " set.\n";
    std::cout << "File " << opts.filename << " set.\n";

    app::show_image(opts.filename);
}
