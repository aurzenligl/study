#include <vector>
#include <string>
#include <unordered_map>

#define DLL_PUBLIC __attribute__ ((visibility ("default")))

bool make(std::unordered_map<std::string, std::vector<std::string>>& x)
{
    using T = std::vector<std::string>;
    x.emplace("01982109820918123", T{"asiosiojasiojaoisj","asiosiojasiojaoisj","asiosiojasiojaoisj"});
    return true;
}

std::unordered_map<std::string, std::vector<std::string>> x;
bool _ = make(x);

std::unordered_map<std::string, std::vector<std::string>> x1;
std::unordered_map<std::string, std::vector<std::string>> x2;
std::unordered_map<std::string, std::vector<std::string>> x3;

DLL_PUBLIC void fun()
{
    make(x2);
}

DLL_PUBLIC void gun()
{
    for (int i=0; i<3; i++)
    {
        make(x3);
    }
    x3.clear();
}

DLL_PUBLIC void hun()
{
    new std::string(10000, 'c');
}
