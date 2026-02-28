import os
from simple_launch import SimpleLauncher


def launch(sl: SimpleLauncher):

    # TODO: transport: isolated node-container with 2+ nodes inside

    proto = os.getenv("PROTO", "shm")
    assert proto in ("shm", "udp"), 'proto must be picked from: "shm", "udp"'
    if proto == "udp":
        os.environ["FASTDDS_BUILTIN_TRANSPORTS"] = "UDPv4"
    print(f"proto: {proto}")

    loaned = os.getenv("LOANED", "0")
    assert loaned in ("0", "1", "x"), 'loaned must be picked from: "0", "1", "x"'
    if loaned == "0":
        os.environ["ROS_DISABLE_LOANED_MESSAGES"] = "1"
    elif loaned == "1":
        os.environ["ROS_DISABLE_LOANED_MESSAGES"] = "0"
    print(f"loaned: {loaned}")

    async_ = int(os.getenv("ASYNC", "0"))
    if async_:
        os.environ["RMW_FASTRTPS_PUBLICATION_MODE"] = "ASYNCHRONOUS"
    print(f"async: {async_}")

    n_subs = int(os.getenv("SUBS", "1"))
    print(f"subs: {n_subs}")

    large = int(os.getenv("LARGE", "0"))
    print(f"async: {large}")

    sl.node(package="demo", executable="talker")
    for i in range(n_subs):
        sl.node(package="demo", executable="listener")


generate_launch_description = lambda: launch(sl := SimpleLauncher()) or sl.launch_description()


# FASTDDS_BUILTIN_TRANSPORTS=UDPv4
# ROS_DISABLE_LOANED_MESSAGES=0
# RMW_FASTRTPS_PUBLICATION_MODE=ASYNCHRONOUS

# ASYNC=1 matters when frequency is high
# PROTO=shm LOANED=1 matters when messages are large
# LOANED=1 is risky as subscriber should not loan msgs yet
# LOANED=x does not give much in multi subscriber scenario
