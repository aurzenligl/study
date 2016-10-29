#include <cstddef>
#include "codec.hpp"
#include "message.hpp"
#include "dispatcher.hpp"

class Service
{
public:
    /// handlers can accept both message and payload as args
    /// handlers don't need to mention message ids at all
    /// as many sizing/encoding details as possible may reside in codec

    void foo(CertainMessage& req, StructView<Abc> abc)
    {
        StructView<Def> def;
        CertainMessage resp(ProphyCodec(), req, def);
        def->x = 1;
        def->y = 2;
        send(resp);
    }

    void bar(CertainMessage& req, ProtoMessageXyz& xyz)
    {
        ProtoMessageZyx zyx;
        zyx.set_x(1);
        zyx.set_y("...");
        send(CertainMessage(ProtobufCodec(), req, zyx));
    }
};

inline void connect(Service& svc, Dispatcher<CertainMessage>& disp)
{
    /// no message ids needed here

    disp.subscribe<ProphyCodec>(&svc, &Service::foo);
    disp.subscribe<ProtobufCodec>(&svc, &Service::bar);
}

int main()
{
    return 0;
}
