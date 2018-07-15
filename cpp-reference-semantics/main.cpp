#include <utility>

struct X {
  X(){}
  X(X&){}  // non-const copy operator prohibits promoting const to non-const
  X& operator=(X&){ return *this; }
  X(X&&) {};
  X& operator=(X&&) { return *this; }
  ~X(){}

  // deep-copying
  X Clone() const { return {}; }  // returns mutable

  // return non-mutable
  const X Child() const { return X(); };

  // TODO: const reading API
  // TODO: non-const mutating API
};

// TODO: XLocked contains shared ownership and lock ownership, read-write

X gen_x() {
  X x;
  return x;
}

struct holder_x {
  operator X& () { return x; }
  X x;
};

void X_to_X(X x) {
  X cpyctor(x);
  X cpyassign; cpyassign = x;
  { X tomove = x; X movector(std::move(tomove)); }
  { X tomove = x; X moveassign; moveassign = std::move(tomove); }
}

void XRef_to_X(X& x) {
  X cpyctor(x);
  X cpyassign; cpyassign = x;
  { X tomove = x; X movector(std::move(tomove)); }
  { X tomove = x; X moveassign; moveassign = std::move(tomove); }
}

void XConst_to_X(const X x) {
#if ILLEGAL
  // error: binding 'const X' to reference of type 'X&' discards qualifiers
  X cpyctor(x);
  // error: binding 'const X' to reference of type 'X&' discards qualifiers
  X cpyassign; cpyassign = x;
  // error: binding 'std::remove_reference<const X&>::type {aka const X}' to reference of type 'X&&' discards qualifiers
  { const X tomove = x.Clone(); X movector(std::move(tomove)); }
  // error: binding 'std::remove_reference<const X&>::type {aka const X}' to reference of type 'X&&' discards qualifiers
  { const X tomove = x.Clone(); X moveassign; moveassign = std::move(tomove); }
#endif
}

void XConstRef_to_X(const X& x) {
#if ILLEGAL
  // error: binding 'const X' to reference of type 'X&' discards qualifiers
  X cpyctor(x);
  // error: binding 'const X' to reference of type 'X&' discards qualifiers
  X cpyassign; cpyassign = x;
  // error: binding 'std::remove_reference<const X&>::type {aka const X}' to reference of type 'X&&' discards qualifiers
  { const X tomove = x.Clone(); X movector(std::move(tomove)); }
  // error: binding 'std::remove_reference<const X&>::type {aka const X}' to reference of type 'X&&' discards qualifiers
  { const X tomove = x.Clone(); X moveassign; moveassign = std::move(tomove); }
#endif
}

void accept_XRef(X& x){}
void accept_XConstRef(const X& x){}
void access_XConst(const X& x) {
#if ILLEGAL
  // error: binding 'const X' to reference of type 'X&&' discards qualifiers
  X y = x.Child();
  // error: invalid initialization of non-const reference of type 'X&' from an rvalue of type 'const X'
  accept_XRef(x.Child());
#endif
  // assigning temporary to const& prolongs the lifetime of temporary,
  // which is _exactly_ what we need to hold ownership of data for as long
  // as it's needed, not just until expression `x.Child()` is evaluated.
  const X& z = x.Child(); (void)z;
  accept_XConstRef(x.Child());
}

int main() {
  // conversions
  X_to_X(gen_x());
  XRef_to_X(holder_x());
  XConst_to_X(gen_x());  // compile-time error
  XConstRef_to_X(holder_x());  // compile-time error

  // read-only const X
  access_XConst(holder_x());  // compile-time error if trying to construct mutable X
}
