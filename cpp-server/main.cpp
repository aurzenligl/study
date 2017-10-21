#include <sys/types.h>
#include <sys/socket.h>

#include <stdexcept>

#include <netinet/ip.h>
#include <unistd.h>
#include <string.h>
#include <assert.h>

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

template <typename T>
void zero(T& x)
{
    memset(&x, 0, sizeof(T));
}

namespace net
{
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

    int socket_tcp()
    {
        int fd = socket(AF_INET, SOCK_STREAM, 0);
        if (fd < 0)
        {
            throw error("socket allocation failed");
        }
        setsockopt(fd, SO_REUSEADDR, true);
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

    int accept(int sockfd, sockaddr_in& addr)
    {
        socklen_t len = sizeof(sockaddr_in);
        int fd = ::accept(sockfd, reinterpret_cast<sockaddr*>(&addr), &len);
        if (fd < 0)
        {
            throw error("accept failed");
        }
        assert(sizeof(sockaddr_in) == len);
        return fd;
    }
}

enum { server_port = 10000 };

int main()
{
    int serv_fd = net::socket_tcp();  // allocate tcp socket
    printf("server allocated fd: %d\n", serv_fd);

    sockaddr_in serv_addr = net::sockaddr_tcp(INADDR_ANY, server_port);
    net::bind(serv_fd, serv_addr);  // bind to address
    printf("bound port: %d\n", server_port);

    listen(serv_fd, 1);
    printf("listen succeeded\n");

    sockaddr_in cli_addr;
    int cli_fd = net::accept(serv_fd, cli_addr);
    printf("accept got fd: %d %08x:%04x\n", cli_fd, cli_addr.sin_addr.s_addr, cli_addr.sin_port);

    getchar();

    close(cli_fd);
    close(serv_fd);
    getchar();
    return 0;
}
