from simple_launch import SimpleLauncher
from simple_launch.events import When


def launch(sl: SimpleLauncher):
    turtlesim_ns = sl.declare_arg("turtlesim_ns", "turtlesim1")
    use_provided_red = sl.declare_arg("use_provided_red", False)
    new_background_r = sl.declare_arg("new_background_r", 200)

    sl.node(package="turtlesim", executable="turtlesim_node", namespace=turtlesim_ns, name="sim")
    sl.call_service(turtlesim_ns + "/spawn", {"x": 5., "y": 2., "theta": 0.2})
    sl.set_parameters(turtlesim_ns + "/sim", {"background_r": 120})
    with sl.group(when=When(delay=2.), if_condition=sl.py_eval(new_background_r, ' == 200 and ', use_provided_red)):
        sl.set_parameters(turtlesim_ns + "/sim", {'background_r': new_background_r})


generate_launch_description = lambda: launch(sl := SimpleLauncher()) or sl.launch_description()
