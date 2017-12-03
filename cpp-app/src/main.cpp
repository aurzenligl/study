#include <string>
#include <iostream>
#include <boost/program_options.hpp>

namespace app
{

namespace po = boost::program_options;

bool parse_cmdline(int ac, char* av[], po::variables_map& vm)
{
    po::positional_options_description pos;
    pos.add("input-file", -1);

    int compr;
    po::options_description generic("Options");
    generic.add_options()
        ("help", "produce help message")
        ("compression,c", po::value<int>(&compr)->default_value(0), "compression level");

    po::options_description hidden("Positional");
    hidden.add_options()
        ("input-file", po::value<std::string>(), "input file");

    po::options_description cmdline_options;
    cmdline_options.add(generic).add(hidden);

    po::store(po::command_line_parser(ac, av).options(cmdline_options).positional(pos).run(), vm);
    po::notify(vm);

    if (vm.count("help"))
    {
        std::cout << generic << "\n";
        return false;
    }

    if (not vm.count("input-file"))
    {
        std::cout << "No input file, nothing to do\n";
        return false;
    }

    return true;
}

}

int main(int ac, char* av[])
{
    app::po::variables_map opts;
    try
    {
        if (not app::parse_cmdline(ac, av, opts))
        {
            return 0;
        }
    }
    catch (std::exception& ex)
    {
        std::cout << ex.what() << '\n';
        return 1;
    }

    std::cout << "Compression level " << opts["compression"].as<int>() << " set.\n";
    std::cout << "File " << opts["input-file"].as<std::string>() << " set.\n";
}
