#include <vector>
#include <string>
#include <unordered_map>

void fun();
void gun();
void hun();

void g()
{
    void* a = malloc(100);
    (void)a;
}

void h()
{
    void* a = malloc(78);
    (void)a;
}

void f()
{
    h();
    for (int i=0; i<3; i++)
    {
        g();
    }
    auto& x = * new std::unordered_map<std::string, std::string>;
    x.emplace("ajs", "asaos");
    x.emplace("ajssdokfsdpokfpsodk", "asdpdpfkspdokfspodkfposdkfpossaos");
    x.emplace("ajssdokfsdpokfpsodk", "asdpdpfkspdokfspodkfposdkfpossaos");
    x.emplace("ajssdokfsdpokfpsodk", "asdpdpfkspdokfspodkfposdkfpossaos");
    x.emplace("ajssdokfsdpokfpsodk", "asdpdpfkspdokfspodkfposdkfpossaos");
    x.emplace("ajssdokfsdpokfpsodk", "asdpdpfkspdokfspodkfposdkfpossaos");
}

void a()
{
    std::vector<std::string> x;
    x.push_back("abc");
    gun();
    x.push_back("abc");
    fun();
    x.push_back("abc");
    hun();
    x.push_back("abc");
    h();
    g();
    f();
    hun();
    x.push_back("abc");
    x.push_back("abc");
    hun();
    x.push_back("abc");
    gun();
    x.push_back("abc");
    x.push_back("abc");
}

int main()
{
    for (int i=0; i<1000; i++)
    {
        a();
    }
}
