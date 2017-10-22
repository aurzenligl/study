#pragma once

#include <algorithm>
#include <list>
#include "net.hpp"

namespace app
{

template <typename To, typename From>
inline To horrible_cast(From from)
{
    // I'm really sorry for doing this, but I find no other
    // way to type erase iterator and regenerate it from pointer memento.
    // Without type erasure I'd need to search in to_client or remove.
    // I guess I'd need to write my own list to do it seamlessly.

    static_assert(sizeof(To) == sizeof(From), "sizes differ");
    static_assert(alignof(To) == alignof(From), "alignments differ");

    To to;
    memcpy(&to, &from, sizeof(To));
    return to;
}

struct client
{
    net::socket sock;
    sockaddr_in addr = {};
    uint64_t data_processed = 0;
};

typedef void* client_memento;

class database
{
public:
    client_memento add(client cl)
    {
        auto it = _clients.insert(_clients.end(), std::move(cl));
        return horrible_cast<client_memento>(it);
    }

    void remove(client_memento mem)
    {
        auto it = horrible_cast<std::list<client>::iterator>(mem);
        _data_processed += it->data_processed;
        _clients.erase(it);
    }

    static client& to_client(client_memento mem)
    {
        auto it = horrible_cast<std::list<client>::iterator>(mem);
        return *it;
    }

    const std::list<client>& clients() const
    {
        return _clients;
    }

    uint64_t data_processed() const
    {
        return _data_processed;
    }

private:
    std::list<client> _clients;
    uint64_t _data_processed = 0;
};

void process_data(app::client& cli, std::string& msg)
{
    std::replace(msg.begin(), msg.end(), '\n', '.');
    cli.data_processed += msg.size();
    printf("message from: %d %08x:%04x {%s}\n", cli.sock.fd(),
            ntohl(cli.addr.sin_addr.s_addr), ntohs(cli.addr.sin_port), msg.c_str());
}

}  // namespace app
