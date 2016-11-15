#include <cstdio>

int main()
{
    FILE *f = popen("echo popen called", "w");
    pclose(f);
}
