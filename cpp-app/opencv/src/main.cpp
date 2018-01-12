#include <iostream>
#include <string>
#include "process.hpp"
#include "clara/clara.hpp"
#include <sys/stat.h>

inline bool exists(const std::string& filename)
{
    struct stat buffer;
    return stat(filename.c_str(), &buffer) == 0;
}

auto check_file = [](std::string& out)
{
    return [&](const std::string& str)
    {
        if (!exists(str))
        {
            return clara::ParserResult::runtimeError("Expected existing file, got: '" + str + "'");
        }
        out = str;
        return clara::ParserResult::ok(clara::ParseResultType::Matched);
    };
};

int main(int ac, char* av[])
{
    bool help = false;
    int operation = 0;
    std::string filename;

    auto cli
        = clara::Help(help)
        | clara::Opt(operation, "operation")
            ["-o"]["--oper"]
            ("operation to apply to image")
        | clara::Arg(check_file(filename), "image file")
            ("image file");

    auto result = cli.parse(clara::Args(ac, av));
    if (!result)
    {
        std::cerr << "Error in command line: " << result.errorMessage() << std::endl;
        exit(1);
    }

    if (help or filename.empty())
    {
        std::cout << cli;
        exit(0);
    }

    std::cout << "Operation " << operation << " set.\n";
    std::cout << "File " << filename << " set.\n";

    switch (operation)
    {
    case 0:
        app::show_image(filename);
        break;
    case 1:
        app::convert_image(filename);
        break;
    case 2:
        app::reduce_color_space(filename);
        break;
    case 3:
        app::sharpen(filename);
        break;
    default:
        std::cout << "Operation " << operation << " has no action.\n";
        break;
    }
}
