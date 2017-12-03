#include <iostream>
#include <sys/stat.h>
#include <boost/program_options.hpp>
#include "opts.hpp"

namespace app
{

static bool exists(const std::string& name)
{
    struct stat buffer;
    return stat(name.c_str(), &buffer) == 0;
}

static bool check_if_file(const std::string& name)
{
    if (not exists(name))
    {
        std::cout << "File not found: " << name << "\n";
        return false;
    }
    return true;
}

cmdline_status parse(int ac, char* av[], cmdline_opts& opts)
{
    namespace po = boost::program_options;

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

    po::variables_map vm;
    try
    {
        po::store(po::command_line_parser(ac, av).options(cmdline_options).positional(pos).run(), vm);
    }
    catch (std::exception& exc)
    {
        std::cout << exc.what() << '\n';
        return cmdline_status(1);
    }
    po::notify(vm);

    if (vm.count("help"))
    {
        std::cout << generic << "\n";
        return cmdline_status(0);
    }

    if (not vm.count("input-file"))
    {
        std::cout << "No input file, nothing to do\n";
        return cmdline_status(1);
    }

    opts.compression = vm["compression"].as<int>();
    opts.filename = vm["input-file"].as<std::string>();

    if (not check_if_file(opts.filename))
    {
        return cmdline_status(1);
    }

    return {};
}

}
