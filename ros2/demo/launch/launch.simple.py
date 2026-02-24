from simple_launch import SimpleLauncher


def generate_launch_description():
    sl = SimpleLauncher()
    sl.node(package="demo_nodes_cpp", executable="listener")
    sl.node(package="demo_nodes_cpp", executable="talker")
    return sl.launch_description()
