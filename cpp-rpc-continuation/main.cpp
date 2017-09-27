#include <functional>

////////////////////////////
// part of common library //
////////////////////////////

// new subscribe variant accepting responder and returning void
//   nowadays people do handlers like:
//     bool handler(const Obj& obj);
//   enable them to do:
//     void handler(Responder& resp, const Obj& obj);

namespace lib
{

class Responder
{
public:
    // this is supposed to pass control to non-terminal continuation
    std::function<void()> then(std::function<void(Responder&)> fun)
    {
        Responder resp = *this;
        return [fun, resp]() mutable
        {
            try
            {
                fun(resp);
            }
            catch (const std::exception& e)
            {
                resp.fail(e);
            }
        };
    }

    // this is supposed to pass control to final continuation
    std::function<void()> then(std::function<bool()> fun)
    {
        Responder resp = *this;
        return [fun, resp]() mutable
        {
            try
            {
                if (fun())
                {
                    resp.success();
                }
                else
                {
                    resp.fail();
                }
            }
            catch (const std::exception& e)
            {
                resp.fail(e);
            }
        };
    }

private:
    // sends result failure; logs
    void fail()
    {
        printf("fail\n");
    }

    // sends result failure; logs
    void fail(const std::exception& e)
    {
        printf("fail: %s\n", e.what());
    }

    // sends result success
    void success()
    {
        printf("success\n");
    }

    // has data allowing to send result and log
    // must be copyable in order to capture it in lambda (std::function requires it)
    // for copyability: can be implemented with a shared_ptr
};

}  // namespace lib

///////////////////////
// part of user code //
///////////////////////

void async_sth(std::function<void()> fun)
{
    // for the sake of simplicity this calls callback immediately
    // any async function accepting continuation handler will do
    fun();
}

struct Foo{};

struct Service
{
    void handle(lib::Responder& resp, const Foo& foo)
    {
        async_sth(resp.then([this](lib::Responder& resp){ handle2(resp, 12, 42); }));
        // with a bit of metaprogramming this could be similar to std::thread ctor:
        //   acquire(resp.then(handle2, this, 12, 42);
    }

    void handle2(lib::Responder& resp, int x, int y)
    {
        // throw std::runtime_error("something early and bad happened");
        async_sth(resp.then([this](){ return handle3(14); }));
        // with a bit of metaprogramming this could be similar to std::thread ctor:
        //   acquire(resp.then(handle3, this, 14);
    }

    bool handle3(int x)
    {
        // throw std::runtime_error("something bad happened");
        // return false;
        return true;
    }
};

///////////////////
// test of sorts //
///////////////////

// uncomment throws and false return above to see different results

int main()
{
    Service svc;
    lib::Responder resp;
    Foo foo;
    svc.handle(resp, foo);
}
