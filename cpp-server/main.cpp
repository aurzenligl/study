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
    // I guess I'd need to write my own list to make it seamlessly.

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

    static const client& to_client(client_memento mem)
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

void process_data(const app::client& cli, std::string& msg)
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
    // TODO what about using RAII for socket file descriptors?
    // TODO and RAII for epoll_fd?
    // TODO make an API for shutting down gracefully using signals using self-pipe trick

    net::socket serv(net::socket_tcp());
    net::setsockopt(serv.fd(), SO_REUSEADDR, true);
    net::setflags(serv.fd(), O_NONBLOCK);
    printf("server allocated fd: %d\n", serv.fd());

    sockaddr_in serv_addr = net::sockaddr_tcp(INADDR_ANY, server_port);
    net::bind(serv.fd(), serv_addr);  // bind to address
    printf("bound port: %d\n", server_port);

    listen(serv.fd(), server_backlog);
    printf("listen succeeded\n");

    int epoll_fd = net::epoll_create();
    net::epoll_ctl(epoll_fd, EPOLL_CTL_ADD, serv.fd(), EPOLLIN, &serv);

    // server data would lie here
    app::database db;

    while (true)
    {
        epoll_event events[max_epoll_events];
        // TODO array_view ?
        for (epoll_event& ev : net::epoll_wait(epoll_fd, events))
        {
            if (ev.data.ptr == &serv)
            {
                sockaddr_in cli_addr;
                net::socket cli(net::accept(serv.fd(), &cli_addr));
                if (!cli.fd())
                {
                    // spurious accept
                    continue;
                }
                printf("accept got fd: %d %08x:%04x\n", cli.fd(), ntohl(cli_addr.sin_addr.s_addr), ntohs(cli_addr.sin_port));

                net::setflags(cli.fd(), O_NONBLOCK);

                int fd = cli.fd();
                app::client_memento mem = db.add({std::move(cli), cli_addr});
                net::epoll_ctl(epoll_fd, EPOLL_CTL_ADD, fd, EPOLLIN | EPOLLET, mem);
            }
            else
            {
                app::client_memento mem = ev.data.ptr;
                const app::client& cli = db.to_client(mem);

                std::string msg;
                while (net::recv(cli.sock.fd(), &msg, read_buffer_len))
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
