#pragma once

// system
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/epoll.h>
#include <sys/signalfd.h>
#include <netinet/in.h>

// C
#include <assert.h>
#include <string.h>

// C++
#include <stdexcept>
#include <string>
#include <vector>

#include "gsl/span"

namespace net
{

struct error : std::runtime_error
{
    using std::runtime_error::runtime_error;
};

template <typename T>
inline void zero(T& x)
{
    memset(&x, 0, sizeof(T));
}

template <typename... Ts>
sigset_t make_sigset(Ts... signals)
{
    sigset_t mask;
    sigemptyset(&mask);
    std::initializer_list<int>{(sigaddset(&mask, signals), 0)...};
    return mask;
}

template <typename... Ts>
void sigprocmask(int how, Ts... signals)
{
    sigset_t mask = net::make_sigset(signals...);
    int ret = ::sigprocmask(how, &mask, 0);
    if (ret < 0)
    {
        throw error("sigprocmask failed");
    }
}

int signalfd(sigset_t& mask)
{
    int fd = signalfd(-1, &mask, 0);
    if (fd < 0)
    {
        throw error("signalfd failed");
    }
    return fd;
}

inline void open_pipe(int* rd, int* wr)
{
    int rdwr[2];
    int ret = ::pipe(rdwr);
    if (ret < 0)
    {
        throw error("pipe failed");
    }
    *rd = rdwr[0];
    *wr = rdwr[1];
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

inline int open_socket(int domain, int type)
{
    int fd = ::socket(domain, type, 0);
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

// returns true if data may be pending in read buffer
template <typename T>
inline bool read(int sockfd, T* t)
{
    int got = ::read(sockfd, t, sizeof(T));
    if (got < 0)
    {
        if (errno == EAGAIN)
        {
            return false;
        }
        throw error("recv failed");
    }
    if (got != sizeof(T))
    {
        throw error("recv read insufficient data");
    }
    return true;
}

void shutdown(int fd, int how)
{
    int ret = ::shutdown(fd, how);
    if (ret < 0)
    {
        throw error("shutdown failed");
    }
}

void close(int fd)
{
    int ret = ::close(fd);
    if (ret < 0)
    {
        throw error("close failed");
    }
}

class sigfile
{
public:
    template <typename... Ints>
    explicit sigfile(Ints... signals)
    {
        sigset_t mask = net::make_sigset(signals...);
        _fd = net::signalfd(mask);
    }

    sigfile(const sigfile& other) = delete;

    ~sigfile()
    {
        try
        {
            net::close(_fd);
        }
        catch (const net::error& e)
        {
            printf("pipeend dtor error: %s\n", e.what());
        }
    }

    void setflags(int flags)
    {
        net::setflags(_fd, flags);
    }

    // returns true if data may be pending in read buffer
    template <typename T>
    bool read(T* msg)
    {
        return net::read(_fd, msg);
    }

    int fd() const
    {
        return _fd;
    }

private:
    int _fd;
};

class pipeend
{
public:
    pipeend(): _fd(0)
    {}

    explicit pipeend(int fd): _fd(fd)
    {}

    pipeend(pipeend&& pp) : _fd(0)
    {
        std::swap(pp._fd, _fd);
    }

    const pipeend& operator=(pipeend&& pp)
    {
        std::swap(pp._fd, _fd);
        return *this;
    }

    ~pipeend()
    {
        if (_fd)
        {
            try
            {
                net::close(_fd);
            }
            catch (const net::error& e)
            {
                printf("pipeend dtor error: %s\n", e.what());
            }
        }
    }

    static void open(pipeend* rrd, pipeend* rwr)
    {
        int rd;
        int wr;
        net::open_pipe(&rd, &wr);
        *rrd = pipeend(rd);
        *rwr = pipeend(wr);
    }

    void setflags(int flags)
    {
        net::setflags(_fd, flags);
    }

    // returns true if data may be pending in read buffer
    bool recv(std::string* msg, size_t len)
    {
        return net::recv(_fd, msg, len);
    }

    int fd() const
    {
        return _fd;
    }

    explicit operator bool()
    {
        return _fd;
    }

private:
    int _fd;
};

class socket
{
public:
    explicit socket(int fd) : _fd(fd)
    {}

    socket(int domain, int type)
    {
        _fd = net::open_socket(domain, type);
    }

    socket(socket&& sock) : _fd(0)
    {
        std::swap(sock._fd, _fd);
    }

    ~socket()
    {
        if (_fd)
        {
            try
            {
                net::shutdown(_fd, SHUT_RDWR);
                net::close(_fd);
            }
            catch (const net::error& e)
            {
                printf("socket dtor error: %s\n", e.what());
            }
        }
    }

    void setsockopt(int optname, int val)
    {
        net::setsockopt(_fd, optname, val);
    }

    void setflags(int flags)
    {
        net::setflags(_fd, flags);
    }

    void bind(in_addr_t addr, in_port_t port)
    {
        sockaddr_in serv_addr = net::sockaddr_tcp(addr, port);
        net::bind(_fd, serv_addr);
    }

    void listen(int backlog)
    {
        net::listen(_fd, backlog);
    }

    // returns empty socket in case of spurious epoll wakeup
    net::socket accept(sockaddr_in* addr)
    {
        return net::socket(net::accept(_fd, addr));
    }

    // returns true if data may be pending in read buffer
    bool recv(std::string* msg, size_t len)
    {
        return net::recv(_fd, msg, len);
    }

    int fd() const
    {
        return _fd;
    }

    explicit operator bool()
    {
        return _fd;
    }

private:
    int _fd;
};

class epoll
{
public:
    epoll()
    {
        _fd = net::epoll_create();
    }

    epoll(const epoll& other) = delete;

    ~epoll()
    {
        try
        {
            net::close(_fd);
        }
        catch (const net::error& e)
        {
            printf("epoll dtor error: %s\n", e.what());
        }
    }

    template <typename Socket>
    void add(Socket& sock, uint32_t events, void* data)
    {
        net::epoll_ctl(_fd, EPOLL_CTL_ADD, sock.fd(), events, data);
    }

    template <int MaxEvents>
    std::vector<epoll_event> wait()
    {
        epoll_event events[MaxEvents];
        auto got = net::epoll_wait(_fd, events);
        return std::vector<epoll_event>(got.begin(), got.end());
    }

    template <int MaxEvents>
    gsl::span<epoll_event> wait(epoll_event (&events)[MaxEvents])
    {
        return net::epoll_wait(_fd, events);
    }

    int fd() const
    {
        return _fd;
    }

    explicit operator bool()
    {
        return _fd;
    }

private:
    int _fd;
};

}  // namespace net
