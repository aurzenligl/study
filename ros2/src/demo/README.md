# ros2 demo

Main package for experimentation and benchmarking ros2 features.


## comm benchmark

There are two nodes: [publisher](src/topics/talker.cpp) and [subscriber](src/topics/listener.cpp).
Both have SensorDataQoS.
Benchmark runs via [launch file](launch/comm.py). There are following env-var settings:

- PROTO: means of transporting messages between nodes:
    - "ptr": intra-process communication via ptr-passing
    - "shm": inter-process communication via [shared-memory](https://fast-dds.docs.eprosima.com/en/latest/fastdds/transport/shared_memory/shared_memory.html#shared-memory-)
    - "udp": inter-process communication via [udp datagrams](https://docs.ros.org/en/kilted/Tutorials/Advanced/Security/Examine-Traffic.html#overview)

- LOANED: whether application and middleware allocate messages separately and (de)serialize. Loaning is using middleware representation directly at application level, omitting serialization step and thus facilitating [zero-copy](https://design.ros2.org/articles/zero_copy.html) transport via shm. Works only with POD data:
    - "0": no loaning
    - "x": publisher loans (omits serialization)
    - "1": publisher and subscriber both loan (omit (de)serialization, [unsafe](https://docs.ros.org/en/kilted/How-To-Guides/Configure-ZeroCopy-loaned-messages.html#subscriptions))

- ASYNC: choosing [publication mode](https://github.com/ros2/rmw_fastrtps?tab=readme-ov-file#change-publication-mode):
    - "0": synchronous
    - "1": asynchronous

- SUBS: number of subscribers, takes and integer number and instantiates this many subscribers simultaneously accepting messages from the publisher. When running benchmarks we'll pick two settings:
    - "1": unicast - one subscriber
    - "3": multicast - three subscribers

- LARGE: whether published message is small or large
    - "0": small (32 bytes)
    - "1": large (1'000'000 bytes)


### comm results

Let's break down results into four scenarios and two tables per sync/async setting.
Test scenario is one publishing node with single thread sending to one or many subscribed nodes
each with its own thread too. Values in table are the average frequencies [kHz] of message reception in a single subscriber. There are four sub-scenarios: small or large, single or many subscribers. In order to have more stable results, parent bash process was set to RT priority:

```
$ sudo renice -20 $$
$ sudo ionice -c 1 -n 0 -p $$
$ PROTO=ptr LOANED=0 ASYNC=0 SUBS=1 LARGE=0 ros2 launch src/demo/launch/comm.py
(...)
```

#### small unicast

| ASYNC=0 | PROTO=ptr | PROTO=shm | PROTO=udp | ASYNC=1 | PROTO=ptr | PROTO=shm | PROTO=udp
|-|-|-|-|-|-|-|-
| **LOANED=0** | 66.3 | 52.5 | 53.1 | **LOANED=0** | 61.6 | 55.2 | 59.5
| **LOANED=x** | 77.9 | 50.7 | 48.7 | **LOANED=x** | 51.7 | 58.4 | 60.3
| **LOANED=1** | 73.2 | 50.5 | 48.8 | **LOANED=1** | 50.6 | 57.7 | 53.9


#### small multicast

| ASYNC=0 | PROTO=ptr | PROTO=shm | PROTO=udp | ASYNC=1 | PROTO=ptr | PROTO=shm | PROTO=udp
|-|-|-|-|-|-|-|-
| **LOANED=0** | 54.2 | 26.5 | 27.8 | **LOANED=0** | 37.2 | 56.8 | 56.5
| **LOANED=x** | 48.7 | 26.2 | 25.1 | **LOANED=x** | 33.1 | 58.5 | 59.7
| **LOANED=1** | 48.3 | 26.3 | 25.3 | **LOANED=1** | 31.8 | 59.7 | 55.9


#### large unicast

| ASYNC=0 | PROTO=ptr | PROTO=shm | PROTO=udp | ASYNC=1 | PROTO=ptr | PROTO=shm | PROTO=udp
|-|-|-|-|-|-|-|-
| **LOANED=0** |  7.5 | 1.5 | 1.5 | **LOANED=0** |  7.7 | 1.7 | 1.8
| **LOANED=x** |  8.6 | 1.9 | 2.0 | **LOANED=x** |  7.8 | 1.9 | 1.8
| **LOANED=1** | 35.3 | 2.5 | 2.0 | **LOANED=1** | 33.2 | 2.1 | 2.1


#### large multicast

| ASYNC=0 | PROTO=ptr | PROTO=shm | PROTO=udp | ASYNC=1 | PROTO=ptr | PROTO=shm | PROTO=udp
|-|-|-|-|-|-|-|-
| **LOANED=0** |  3.9 | 0.70 | 0.69 | **LOANED=0** |  5.1 | 0.71 | 0.70
| **LOANED=x** |  6.8 | 0.79 | 0.77 | **LOANED=x** |  5.0 | 0.72 | 0.76
| **LOANED=1** | 27.0 | 0.91 | 0.91 | **LOANED=1** | 23.9 | 0.91 | 0.85


### discussion

- PROTO=ptr is the champion, with LOANED=1 is outclasses everything
- PROTO=shm is very close to PROTO=udp in practically all scenarios
- ASYNC=1 improves only high-frequency publishing, multicasting via shm/udp especially
- ASYNC=1 sometimes pessimizes
- LOANED=1 (copy elision) matters only for large messages
- LOANED=1 (pub + sub) is much better than LOANED=x (only pub), but it's [discouraged as unsafe](https://github.com/ros2/rcl/pull/1110)
