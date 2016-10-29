#ifndef CODEC_DETAIL_HPP_
#define CODEC_DETAIL_HPP_

/// don't read this. It's only meant to allow main module compile

template <typename Message>
bool hasForeignEndianness(Message);
namespace prophy { void swap(void*); }
template <typename StructType>
struct StructView
{
    StructType* operator->()
    {
        return x;
    };
    StructType* x;
};
struct Abc;
struct Def
{
    int x;
    int y;
};
struct Ghi;
struct ProtoMessageXyz;
struct ProtoMessageZyx
{
    void set_x(int);
    void set_y(const char*);
};
class CertainMessage;
void send(CertainMessage);

#endif /* CODEC_DETAIL_HPP_ */
