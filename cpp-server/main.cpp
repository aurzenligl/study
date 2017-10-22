#include "net.hpp"
#include "app.hpp"

enum
{
    server_port = 10000,
    server_backlog = 1,
    read_buffer_len = 10,
    max_epoll_events = 10
};

void serve_forever(app::database& db, net::sigfile& sig, net::socket& serv, net::epoll& epoll)
{
    while (true)
    {
        for (epoll_event& ev : epoll.wait<max_epoll_events>())
        {
            if (ev.data.ptr == &sig)
            {
                signalfd_siginfo siginfo;
                sig.read(&siginfo);
                printf("received signal no: %d code: %d\n", siginfo.ssi_signo, siginfo.ssi_code);
                return;
            }
            else if (ev.data.ptr == &serv)
            {
                sockaddr_in cli_addr;
                net::socket cli = serv.accept(&cli_addr);
                if (!cli)
                {
                    continue;  // spurious accept
                }
                printf("accept got fd: %d %08x:%04x\n", cli.fd(),
                        ntohl(cli_addr.sin_addr.s_addr), ntohs(cli_addr.sin_port));

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
                        printf("closing fd: %d %08x:%04x, data processed: %ld\n", cli.sock.fd(),
                                ntohl(cli.addr.sin_addr.s_addr), ntohs(cli.addr.sin_port),
                                cli.data_processed);
                        db.remove(mem);
                        break;
                    }
                    app::process_data(cli, msg);
                }
            }
        }
    }
}

int main()
{
    try
    {
        net::sigprocmask(SIG_BLOCK, SIGINT, SIGQUIT);
        net::sigfile sig(SIGINT, SIGQUIT);

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
        epoll.add(sig, EPOLLIN, &sig);

        // server data would lie here
        app::database db;
        serve_forever(db, sig, serv, epoll);
        printf("summary - data processed %ld\n", db.data_processed());
    }
    catch (const std::exception& e)
    {
        printf("exceptional exit: %s\n", e.what());
        return 1;
    }

    printf("thank you for graceful shutdown, bye\n");
    return 0;
}
