import os
from simple_launch import SimpleLauncher


def launch(sl: SimpleLauncher):

    # TODO: transport: loaned msgs for shm zero-copy

    # TODO: define POD messages
    # TODO: size: small, large

    # TODO: transport: isolated node-container with 2+ nodes inside

    # TODO: publishing mode: sync, async

    subs = os.getenv("SUBS", "1")
    n_subs = int(subs)
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
