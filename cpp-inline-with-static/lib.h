inline int increment(int x) {
  static int sum = 0;
  sum += x;
  return sum;
}

int get();
