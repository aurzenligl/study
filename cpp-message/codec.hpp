#ifndef CODEC_HPP_
#define CODEC_HPP_

#include "codec_detail.hpp"

/// this is only a concept
class Codec
{
    /// meant to transform messages to payloads and vice versa
    /// payload types (kinds) may differ greatly, so there may be numerous codecs
    /// Type2Id may be generated automatically

    template <class Payload>
    struct Type2Id;

    template <class Payload>
    static size_t size(const Payload&);

    template <class Payload, class A1>
    static size_t size(const Payload&, const A1&);

    template <class Payload, class A1, class A2>
    static size_t size(const Payload&, const A1&, const A2&);

    template <class Payload, class Message>
    static void encode(Payload& payload, Message& msg);

    template <class Payload, class Message, class A1>
    static void encode(Payload& payload, Message& msg, const A1&);

    template <class Payload, class Message, class A1, class A2>
    static void encode(Payload& payload, Message& msg, const A1&, const A2&);

    template <class Payload, class Message>
    static void decode(Message& msg, Payload& payload);
};

class ProtobufCodec
{
    template <class Payload>
    struct Type2Id;

    template <class Payload>
    static size_t size(const Payload& pd)
    {
        return pd.ByteSize();
    }

    template <class Payload, class Message>
    static void encode(Payload& payload, Message& msg)
    {
        payload.SerializeToArray(msg.data(), msg.size());
    }

    template <class Payload, class Message>
    static void decode(Message& msg, Payload& payload)
    {
        payload.ParseFromArray(msg.data(), msg.size());
    }
};

class ProphyCodec
{
    template <class Payload>
    struct Type2Id;

    template <class Payload>
    static size_t size(const Payload&)
    {
        /// can/should be overloaded for specific payloads
        return sizeof(Payload::StructType);
    }

    template <class Payload, class Message>
    static void encode(Payload& payload, Message& msg)
    {
        payload.set(msg.data);
        /// just couples payload with message, user needs to write to payload afterwards
    }

    template <class Payload, class Message>
    static void decode(Message& msg, Payload& payload)
    {
        if (hasForeignEndianness(msg))
        {
            prophy::swap<Payload::StructType>(msg.data());
        }
        payload.set(msg.data());
    }
};

template <> struct ProphyCodec::Type2Id<StructView<Abc>> { enum { value = 123 }; };
template <> struct ProphyCodec::Type2Id<StructView<Def>> { enum { value = 124 }; };
template <> struct ProphyCodec::Type2Id<StructView<Ghi>> { enum { value = 125 }; };

#endif /* CODEC_HPP_ */
