import os
from simple_launch import SimpleLauncher


def launch(sl: SimpleLauncher):

    subs = os.getenv("SUBS", "one")
    assert subs in ("one", "many"), "error: bad SUBS value, pick from: one, many"
    n_subs = {"one": 1, "many": 3}[subs]
    print(f"subs: {n_subs}")

    print("transport: shm")
    sl.node(package="demo", executable="talker")
    for i in range(n_subs):
        sl.node(package="demo", executable="listener")

    # print("transport: udp")
    # os.environ["FASTDDS_BUILTIN_TRANSPORTS"] = "UDPv4"
    # sl.node(package="demo", executable="talker")
    # for i in range(n_subs):
    #     sl.node(package="demo", executable="listener")


# FASTDDS_BUILTIN_TRANSPORTS=UDPv4
# RMW_FASTRTPS_PUBLICATION_MODE=ASYNCHRONOUS


generate_launch_description = lambda: launch(sl := SimpleLauncher()) or sl.launch_description()
