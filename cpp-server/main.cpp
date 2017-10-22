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
    int fd;
    sockaddr_in addr;
};

typedef void* client_memento;

class database
{
public:
    client_memento add(client cl)
    {
        auto it = _clients.insert(_clients.end(), cl);
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
    printf("message from: %d %08x:%04x {%s}\n", cli.fd, ntohl(cli.addr.sin_addr.s_addr), ntohs(cli.addr.sin_port), msg.c_str());
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

    int serv_fd = net::socket_tcp();  // allocate tcp socket
    net::setsockopt(serv_fd, SO_REUSEADDR, true);
    net::setflags(serv_fd, O_NONBLOCK);
    printf("server allocated fd: %d\n", serv_fd);

    sockaddr_in serv_addr = net::sockaddr_tcp(INADDR_ANY, server_port);
    net::bind(serv_fd, serv_addr);  // bind to address
    printf("bound port: %d\n", server_port);

    listen(serv_fd, server_backlog);
    printf("listen succeeded\n");

    int epoll_fd = net::epoll_create();
    net::epoll_ctl(epoll_fd, EPOLL_CTL_ADD, serv_fd, EPOLLIN, &serv_fd);

    // server data would lie here
    app::database db;

    while (true)
    {
        epoll_event events[max_epoll_events];
        // TODO array_view ?
        for (epoll_event& ev : net::epoll_wait(epoll_fd, events))
        {
            if (ev.data.ptr == &serv_fd)
            {
                sockaddr_in cli_addr;
                int cli_fd = net::accept(serv_fd, &cli_addr);
                if (!cli_fd)
                {
                    // spurious accept
                    continue;
                }
                printf("accept got fd: %d %08x:%04x\n", cli_fd, ntohl(cli_addr.sin_addr.s_addr), ntohs(cli_addr.sin_port));

                net::setflags(cli_fd, O_NONBLOCK);
                app::client_memento mem = db.add({cli_fd, cli_addr});
                net::epoll_ctl(epoll_fd, EPOLL_CTL_ADD, cli_fd, EPOLLIN | EPOLLET, mem);
            }
            else
            {
                app::client_memento mem = ev.data.ptr;
                const app::client& cli = db.to_client(mem);

                std::string msg;
                while (net::recv(cli.fd, &msg, read_buffer_len))
                {
                    if (msg.empty())
                    {
                        // TODO net close with exception + shutdown
                        close(cli.fd);
                        printf("closing fd: %d %08x:%04x\n", cli.fd, ntohl(cli.addr.sin_addr.s_addr), ntohs(cli.addr.sin_port));
                        db.remove(mem);
                        break;
                    }
                    app::process_data(cli, msg);
                }
            }
        }
    }

    // TODO shutdown first, throw on errors
    for (const app::client& cli : db.clients())
    {
        close(cli.fd);
    }
    close(serv_fd);
    return 0;
}
