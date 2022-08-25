#include <iostream>
#include <signal.h>

void sig_handler(int signo) {
  std::cerr << "child: signal " << signo << '\n';
}

int main() {
  signal(SIGINT, sig_handler);
  signal(SIGTERM, sig_handler);
  signal(SIGPIPE, sig_handler);

  std::cerr << "child: starts" << '\n';
  int x = getchar();

  std::cerr << "child: getchar got " << x << '\n';
  std::cerr << "child: stops" << '\n';
  return 3;
}
