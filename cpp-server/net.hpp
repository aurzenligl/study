#pragma once

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
#include <string>

#include "gsl/span"

namespace net
{

typedef unsigned char byte;

struct error : std::runtime_error
{
    using std::runtime_error::runtime_error;
};

template <typename T>
inline void zero(T& x)
{
    memset(&x, 0, sizeof(T));
}

inline void setsockopt(int sockfd, int optname, int val)
{
    if (::setsockopt(sockfd, SOL_SOCKET, optname, &val, sizeof(int)) < 0)
    {
        throw error(
            "setsockopt failed optname: " + std::to_string(optname) +
            " val: " + std::to_string(val));
    }
}

inline void setflags(int fd, int flags)
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

inline int socket_tcp()
{
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0)
    {
        throw error("socket allocation failed");
    }
    return fd;
}

inline sockaddr_in sockaddr_tcp(in_addr_t addr, in_port_t port)
{
    sockaddr_in sa;
    zero(sa);  // clear memory
    sa.sin_family = AF_INET;  // set ip family
    sa.sin_addr.s_addr = htonl(addr);  // swap bytes
    sa.sin_port = htons(port);  // swap bytes
    return sa;
}

inline void bind(int sockfd, sockaddr_in addr)
{
    if (::bind(sockfd, reinterpret_cast<sockaddr*>(&addr), sizeof(sockaddr)) < 0)
    {
        throw error("bind failed");
    }
}

inline void listen(int sockfd, int backlog)
{
    if (::listen(sockfd, backlog) < 0)
    {
        throw error("listen failed");
    }
}

inline int epoll_create()
{
    int fd = ::epoll_create1(0);
    if (fd < 0)
    {
        throw error("epoll create failed");
    }
    return fd;
}

inline void epoll_ctl(int epoll_fd, int op, int fd, uint32_t events, void* data)
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
inline gsl::span<epoll_event> epoll_wait(int epoll_fd, epoll_event (&events)[N])
{
    int nfds = ::epoll_wait(epoll_fd, events, N, -1);
    if (nfds < 0)
    {
        throw error("epoll wait failed");
    }
    gsl::span<epoll_event> x(events, nfds);
    return x;
}

// returns 0 when no connection was accepted
inline int accept(int sockfd, sockaddr_in* addr)
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
inline bool recv(int sockfd, std::string* msg, size_t len)
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
