#include <string>
#include <boost/preprocessor/repetition/repeat.hpp>

#define DLL_PUBLIC __attribute__ ((visibility ("default")))

std::string x = "1234567890";
std::string y = "123456789-";

#define DECLCOMP_(z, n2, n) bool compare ## n ## _ ## n2();
#define DECLCOMP(z, n, n2) BOOST_PP_REPEAT(n2, DECLCOMP_, n)
BOOST_PP_REPEAT(10, DECLCOMP, 10)

DLL_PUBLIC int fun()
{
    int r = 0;
#define EXPRCOMP_(z, n2, n) r += compare ## n ## _ ## n2();
#define EXPRCOMP(z, n, n2) BOOST_PP_REPEAT(n2, EXPRCOMP_, n)
    BOOST_PP_REPEAT(10, EXPRCOMP, 10)
    return r;
}

DLL_PUBLIC void gun()
{
}

DLL_PUBLIC void hun()
{
}
