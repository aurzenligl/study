from simple_launch import SimpleLauncher


def launch(sl: SimpleLauncher):
    sl.include("demo", "launch.yaml")


generate_launch_description = lambda: launch(sl := SimpleLauncher()) or sl.launch_description()
