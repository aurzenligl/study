// system
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/epoll.h>
#include <netinet/in.h>

// C
#include <assert.h>
#include <string.h>

// C++
#include <stdexcept>
#include <iostream>
#include <string>
#include <algorithm>
#include <vector>
#include <list>

template <typename T>
inline void zero(T& x)
{
    memset(&x, 0, sizeof(T));
}

namespace net
{

typedef unsigned char byte;

struct error : std::runtime_error
{
    using std::runtime_error::runtime_error;
};

void setsockopt(int sockfd, int optname, int val)
{
    if (::setsockopt(sockfd, SOL_SOCKET, optname, &val, sizeof(int)) < 0)
    {
        throw error(
            "setsockopt failed optname: " + std::to_string(optname) +
            " val: " + std::to_string(val));
    }
}

void setflags(int fd, int flags)
{
    int cur = fcntl(fd, F_GETFL);
    if (cur < 0)
    {
        throw error("flags get failed");
    }
    if (fcntl(fd, F_SETFL, cur | flags) < 0)
    {
        throw error("flags set failed");
    }
}

int socket_tcp()
{
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0)
    {
        throw error("socket allocation failed");
    }
    return fd;
}

sockaddr_in sockaddr_tcp(in_addr_t addr, in_port_t port)
{
    sockaddr_in sa;
    zero(sa);  // clear memory
    sa.sin_family = AF_INET;  // set ip family
    sa.sin_addr.s_addr = htonl(addr);  // swap bytes
    sa.sin_port = htons(port);  // swap bytes
    return sa;
}

void bind(int sockfd, sockaddr_in addr)
{
    if (::bind(sockfd, reinterpret_cast<sockaddr*>(&addr), sizeof(sockaddr)) < 0)
    {
        throw error("bind failed");
    }
}

void listen(int sockfd, int backlog)
{
    if (::listen(sockfd, backlog) < 0)
    {
        throw error("listen failed");
    }
}

int epoll_create()
{
    int fd = ::epoll_create1(0);
    if (fd < 0)
    {
        throw error("epoll create failed");
    }
    return fd;
}

void epoll_ctl(int epoll_fd, int op, int fd, uint32_t events, void* data)
{
    epoll_event ev;
    ev.events = events;
    ev.data.ptr = data;
    int ret = ::epoll_ctl(epoll_fd, EPOLL_CTL_ADD, fd, &ev);
    if (ret < 0)
    {
        throw error("epoll control failed");
    }
}

template <int N>
int epoll_wait(int epoll_fd, epoll_event (&events) [N])
{
    int nfds = ::epoll_wait(epoll_fd, events, N, -1);
    if (nfds < 0)
    {
        throw error("epoll wait failed");
    }
    return nfds;
}

// returns 0 when no connection was accepted
int accept(int sockfd, sockaddr_in* addr)
{
    socklen_t len = sizeof(sockaddr_in);
    int fd = ::accept(sockfd, reinterpret_cast<sockaddr*>(addr), &len);
    if (fd < 0)
    {
        if (errno == EAGAIN)
        {
            return 0;
        }
        throw error("accept failed");
    }
    assert(sizeof(sockaddr_in) == len);
    return fd;
}

// returns true if data may be pending in read buffer
bool recv(int sockfd, std::string* msg, size_t len)
{
    msg->resize(len);
    int got = ::recv(sockfd, &msg->front(), msg->size(), 0);
    if (got < 0)
    {
        if (errno == EAGAIN)
        {
            return false;
        }
        throw error("recv failed");
    }
    msg->resize(got);
    return true;
}

}  // namespace net

enum { server_port = 10000 };
enum { server_backlog = 1 };
enum { read_buffer_len = 10 };
enum { max_epoll_events = 10 };

namespace app
{

template <typename To, typename From>
inline To horrible_cast(From from)
{
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

const client& to_client(client_memento mem)
{
    auto it = horrible_cast<std::list<client>::iterator>(mem);
    return *it;
}

class database
{
public:
    client_memento add(client cl)
    {
        auto it = _clients.insert(_clients.end(), cl);
        return horrible_cast<void*>(it);
    }

    void remove(client_memento mem)
    {
        auto it = horrible_cast<std::list<client>::iterator>(mem);
        _clients.erase(it);
    }

    const std::list<client>& clients() const
    {
        return _clients;
    }

private:
    std::list<client> _clients;
};

// returns true on read shutdown
bool serve_client(const app::client& cli)
{
    std::string msg;
    while (net::recv(cli.fd, &msg, read_buffer_len))
    {
        if (msg.empty())
        {
            return true;
        }
        std::replace(msg.begin(), msg.end(), '\n', '.');
        std::cout << "message: {" << msg << "}\n";
    }
    return false;
}

}  // namespace app

int main()
{
    // TODO make application namespace and data
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
        int nfds = net::epoll_wait(epoll_fd, events);
        for (int n = 0; n < nfds; ++n)
        {
            if (events[n].data.ptr == &serv_fd)
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
                app::client_memento mem = events[n].data.ptr;
                const app::client& cli = app::to_client(mem);

                if (app::serve_client(cli))
                {
                    close(cli.fd);
                    printf("closing fd: %d %08x:%04x\n", cli.fd, ntohl(cli.addr.sin_addr.s_addr), ntohs(cli.addr.sin_port));

                    db.remove(mem);
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
