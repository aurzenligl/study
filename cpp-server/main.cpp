#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/epoll.h>

#include <stdexcept>
#include <assert.h>
#include <string.h>
#include <iostream>
#include <string>
#include <algorithm>

#include <netinet/ip.h>

//http://man7.org/linux/man-pages/man7/socket.7.html
// O_NONBLOCK flag on a socket file descriptor using fcntl(2)

//http://man7.org/linux/man-pages/man2/accept.2.html
//If the socket is marked nonblocking and no
//       pending connections are present on the queue, accept() fails with the
//       error EAGAIN or EWOULDBLOCK.
//In order to be notified of incoming connections on a socket, you can
//       use select(2), poll(2), or epoll(7).
//SOCK_NONBLOCK   Set the O_NONBLOCK file status flag on the new open
//               file description.  Using this flag saves extra calls
//               to fcntl(2) to achieve the same result.

//http://man7.org/linux/man-pages/man2/recv.2.html
//If no messages are available at the socket, the receive calls wait
//      for a message to arrive, unless the socket is nonblocking (see
//      fcntl(2)), in which case the value -1 is returned and the external
//      variable errno is set to EAGAIN or EWOULDBLOCK.

template <typename T>
void zero(T& x)
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

    void epoll_ctl(int epoll_fd, int op, int fd, uint32_t events, int data)
    {
        epoll_event ev;
        ev.events = events;
        ev.data.fd = fd;
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

    int accept(int sockfd, sockaddr_in* addr)
    {
        socklen_t len = sizeof(sockaddr_in);
        int fd = ::accept(sockfd, reinterpret_cast<sockaddr*>(addr), &len);
        if (fd < 0)
        {
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
}

enum { server_port = 10000 };
enum { server_backlog = 1 };
enum { read_buffer_len = 10 };
enum { max_epoll_events = 10 };

bool serve_client(int fd);

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
    net::epoll_ctl(epoll_fd, EPOLL_CTL_ADD, serv_fd, EPOLLIN, serv_fd);

    // server data would lie here
    std::vector<int> cfds;

    while (true)
    {
        epoll_event events[max_epoll_events];
        // TODO array_view ?
        int nfds = net::epoll_wait(epoll_fd, events);
        for (int n = 0; n < nfds; ++n)
        {
            if (events[n].data.fd == serv_fd)
            {
                sockaddr_in cli_addr;
                int cli_fd = net::accept(serv_fd, &cli_addr);
                printf("accept got fd: %d %08x:%04x\n", cli_fd, ntohl(cli_addr.sin_addr.s_addr), ntohs(cli_addr.sin_port));

                net::setflags(cli_fd, O_NONBLOCK);
                net::epoll_ctl(epoll_fd, EPOLL_CTL_ADD, cli_fd, EPOLLIN | EPOLLET, cli_fd);
                cfds.push_back(cli_fd);
            }
            else
            {
                if (serve_client(events[n].data.fd))
                {
                    close(events[n].data.fd);
                }
            }
        }
    }

    for (int cfd : cfds)
    {
        close(cfd);
    }
    close(serv_fd);
    return 0;
}

// returns true on read shutdown
bool serve_client(int fd)
{
    std::string msg;
    while (net::recv(fd, &msg, read_buffer_len))
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
