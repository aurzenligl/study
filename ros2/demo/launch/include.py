from simple_launch import SimpleLauncher


def generate_launch_description():
    sl = SimpleLauncher()
    sl.include("demo", "launch.yaml")
    return sl.launch_description()
