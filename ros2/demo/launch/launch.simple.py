from simple_launch import SimpleLauncher


def launch(sl: SimpleLauncher):
    sl.node(package="demo_nodes_cpp", executable="listener")
    sl.node(package="demo_nodes_cpp", executable="talker")


generate_launch_description = lambda: launch(sl := SimpleLauncher()) or sl.launch_description()
