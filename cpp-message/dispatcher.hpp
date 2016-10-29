#ifndef DISPATCHER_HPP_
#define DISPATCHER_HPP_

template <class Message>
class Dispatcher
{
public:
    /// gets Payload type from Met (meta-programming)
    /// gets MessageId from Codec (type trait)
    template <class Codec, class Cls, class Met>
    void subscribe(Cls* cls, Met met);

    /// as above
    template <class Codec, class Fun>
    void subscribe(Fun fun);

    void handle(Message msg);
};

#endif /* DISPATCHER_HPP_ */
