#include <stdio.h>

char * hello(char * what)
{
    printf("%s\n", what);
    return "alabama";
}

void waste_time()
{
    int i, n;
    for (i = 0; i < 1024*1024*1024; i++) {
        if ((i % 2) == 0) n++;
    }
}
