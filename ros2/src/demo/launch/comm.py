import os
from simple_launch import SimpleLauncher


# usage example: PROTO=shm LOANED=0 ASYNC=0 SUBS=1 LARGE=0 ros2 launch src/demo/launch/comm.py


def launch(sl: SimpleLauncher):
    proto = os.getenv("PROTO", "shm")
    assert proto in ("ptr", "shm", "udp"), 'proto must be picked from: "ptr", "shm", "udp"'
    containerize = (proto == "ptr")
    if proto == "shm":
        os.environ["FASTDDS_BUILTIN_TRANSPORTS"] = "UDPv4"
    elif proto == "udp":
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

    if containerize:
        with sl.container(name="demo_container", executable="component_container_isolated"):
            sl.node(package="demo", plugin="Talker")
            for i in range(n_subs):
                sl.node(package="demo", plugin="Listener")
    else:
        sl.node(package="demo", executable="talker")
        for i in range(n_subs):
            sl.node(package="demo", executable="listener")


generate_launch_description = lambda: launch(sl := SimpleLauncher()) or sl.launch_description()


# FASTDDS_BUILTIN_TRANSPORTS=UDPv4
# ROS_DISABLE_LOANED_MESSAGES=0
# RMW_FASTRTPS_PUBLICATION_MODE=ASYNCHRONOUS


# ASYNC=1 matters when frequency is high
# PROTO=shm LOANED=1 matters when messages are large
# PROTO=shm ~= PROTO=udp when LOANED=0
# LOANED=1 is supposedly risky for subscriber, thus spoke ros community
# LOANED=x does not give much in multi subscriber scenario
# PROTO=ptr best by a large margin
# PROTO=ptr LOANED=1 outclasses anything else
