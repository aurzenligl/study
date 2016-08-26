#include <jsonrpccpp/client.h>
#include <jsonrpccpp/client/connectors/httpclient.h>
#include <iostream>

using namespace jsonrpc;
using namespace std;

Json::Value a30_b6()
{
    Json::Value params;
    params["a"] = 30;
    params["b"] = 6;
    return params;
}

Json::Value _30_6()
{
    Json::Value params;
    params.append(30);
    params.append(6);
    return params;
}

Json::Value _verbose_true()
{
    Json::Value params;
    params["verbose"] = true;
    return params;
}

int main()
{
    HttpClient client("http://localhost:8080");
    Client c(client);

    try
    {
        for (int x=0; x<1; x++)
        {
            cout << c.CallMethod("add", a30_b6()) << endl;
            cout << c.CallMethod("sub", _30_6()) << endl;
            cout << c.CallMethod("mul", a30_b6()) << endl;
            cout << c.CallMethod("div", _30_6()) << endl;
        }

        if (0)
        {
            cout << c.CallMethod("printme", {}).asString() << endl;
            cout << c.CallMethod("printme", _verbose_true()).asString() << endl;
        }
    }
    catch (JsonRpcException& e)
    {
        cerr << e.what() << endl;
    }
}
