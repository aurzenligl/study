#ifndef MESSAGE_HPP_
#define MESSAGE_HPP_

/// this is only a concept
struct Message
{
    void* data();
    size_t size() const;
};

class CertainMessage
{
public:
    /// only datagram, id, sender, reciever, apf
    /// copy/move semantics
    /// may be empty (safe bool idiom)

    template <class Codec, class Payload>
    CertainMessage(Codec codec, const CertainMessage& req, Payload& payload)
    {
        /// can get sender and receiver from req
        /// can get id and size from codec/payload/extra args;
    }

    template <class Codec, class Payload, class A1>
    CertainMessage(Codec codec, const CertainMessage& req, Payload& payload, const A1& a1);

    void* data();
    size_t size() const;
};

class Event
{
public:
    /// only datagram, id
    /// copy/move semantics
    /// may be empty (safe bool idiom)

    void* data();
    size_t size() const;
};

#endif /* MESSAGE_HPP_ */
