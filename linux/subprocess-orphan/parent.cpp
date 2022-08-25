#include <boost/asio.hpp>
#include <boost/process.hpp>
#include <iostream>

namespace bp = boost::process;

int main() {
  boost::asio::io_service ios;
  bp::async_pipe in(ios);
  bp::child c("./child", bp::std_in < in);
  ios.run();
  c.wait();
  int result = c.exit_code();
  std::cout << "result: " << result << '\n';
}
