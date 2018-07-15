#include <utility>

struct XConst;

struct X {
  X(){}
  X(const X&){}
  X& operator=(const X&){ return *this; }
  X(X&&) {};
  X& operator=(X&&) { return *this; }
  ~X(){}

  // deep-copying
  X Clone() { return {}; }  // returns mutable

  // TODO: reading API
  // TODO: mutating API
  // TODO: child-access API returns X
};

struct XConst {
  XConst(){}
  XConst(const XConst&){}
  XConst& operator=(const XConst&){ return *this; }
  XConst(XConst&&){}
  XConst& operator=(XConst&&){ return *this; }
  ~XConst(){}

  // converting ctors: X -> XConst
  XConst(const X&){}
  XConst& operator=(const X&){ return *this; }
  XConst(X&&){}
  XConst& operator=(X&&){ return *this; }

  // deep-copying
  X Clone() { return {}; }  // returns mutable

  // TODO: reading API
  // TODO: child-access API returns XConst
};

// TODO: XLocked contains shared ownership and lock ownership, read-write
// TODO: XConstLocked contains shared ownership and lock ownership, read-only

X gen_x() {
  X x;
  return x;
}

void X_to_X(X x) {
  X cpyctor(x);
  X cpyassign; cpyassign = x;
  { X tomove = x; X movector(std::move(tomove)); }
  { X tomove = x; X moveassign; moveassign = std::move(tomove); }
}

void X_to_XConst(X x) {
  XConst cpyctor(x);
  XConst cpyassign; cpyassign = x;
  { X tomove = x; XConst movector(std::move(tomove)); }
  { X tomove = x; XConst moveassign; moveassign = std::move(tomove); }
}

void XConst_to_XConst(XConst x) {
  XConst cpyctor(x);
  XConst cpyassign; cpyassign = x;
  { XConst tomove = x; XConst movector(std::move(tomove)); }
  { XConst tomove = x; XConst moveassign; moveassign = std::move(tomove); }
}

void XConst_to_X(XConst x) {
#if ILLEGAL
  X cpyctor(x);  // error: no matching function for call to ‘X::X(XConst&)’
  X cpyassign; cpyassign = x;  // error: no match for ‘operator=’ (operand types are ‘X’ and ‘XConst’)
  { XConst tomove = x; X movector(std::move(tomove)); }  // error: no matching function for call to ‘X::X(std::remove_reference<XConst&>::type)’
  { XConst tomove = x; X moveassign; moveassign = std::move(tomove); }  // error: no match for ‘operator=’ (operand types are ‘X’ and ‘std::remove_reference<XConst&>::type {aka XConst}’)
#endif
}

void X_clone(X x) {
  { X cloned = x.Clone(); }
  { XConst cloned = x.Clone(); }
}

void XClone_clone(XConst x) {
  { X cloned = x.Clone(); }
  { XConst cloned = x.Clone(); }
}

int main() {
  X_to_X(gen_x());
  X_to_XConst(gen_x());
  XConst_to_XConst(gen_x());
  XConst_to_X(gen_x());
  X_clone(gen_x());
  XClone_clone(gen_x());
}
