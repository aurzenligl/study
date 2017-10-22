#include <iostream>
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
    sockaddr_in addr;
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

private:
    std::list<client> _clients;
};

void process_data(app::client& cli, std::string& msg)
{
    std::replace(msg.begin(), msg.end(), '\n', '.');
    printf("message from: %d %08x:%04x {%s}\n", cli.sock.fd(), ntohl(cli.addr.sin_addr.s_addr), ntohs(cli.addr.sin_port), msg.c_str());
}

}  // namespace app

enum
{
    server_port = 10000,
    server_backlog = 1,
    read_buffer_len = 10,
    max_epoll_events = 10
};

int main()
{
    // TODO make an API for shutting down gracefully using signals using self-pipe trick

    net::socket serv(AF_INET, SOCK_STREAM);
    serv.setsockopt(SO_REUSEADDR, true);
    serv.setflags(O_NONBLOCK);
    printf("server allocated fd: %d\n", serv.fd());

    serv.bind(INADDR_ANY, server_port);
    printf("bound port: %d\n", server_port);

    serv.listen(server_backlog);
    printf("listen succeeded\n");

    net::epoll epoll;
    epoll.add(serv, EPOLLIN, &serv);

    // server data would lie here
    app::database db;

    while (true)
    {
        for (epoll_event& ev : epoll.wait<max_epoll_events>())
        {
            if (ev.data.ptr == &serv)
            {
                sockaddr_in cli_addr;
                net::socket cli = serv.accept(&cli_addr);
                if (!cli)
                {
                    continue;  // spurious accept
                }
                printf("accept got fd: %d %08x:%04x\n", cli.fd(), ntohl(cli_addr.sin_addr.s_addr), ntohs(cli_addr.sin_port));

                cli.setflags(O_NONBLOCK);
                app::client_memento mem = db.add({std::move(cli), cli_addr});
                epoll.add(db.to_client(mem).sock, EPOLLIN | EPOLLET, mem);
            }
            else
            {
                app::client_memento mem = ev.data.ptr;
                app::client& cli = db.to_client(mem);

                std::string msg;
                while (cli.sock.recv(&msg, read_buffer_len))
                {
                    if (msg.empty())
                    {
                        printf("closing fd: %d %08x:%04x\n", cli.sock.fd(), ntohl(cli.addr.sin_addr.s_addr), ntohs(cli.addr.sin_port));
                        db.remove(mem);
                        break;
                    }
                    app::process_data(cli, msg);
                }
            }
        }
    }
    return 0;
}
