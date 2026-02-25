from simple_launch import SimpleLauncher


def launch(sl: SimpleLauncher):
    background_r = 200
    sl.include("demo", "include/example_substitutions.simple.py", "launch", launch_arguments={
        "turtlesim_ns": "turtlesim2",
        "use_provided_red": True,
        "new_background_r": background_r,
    })


generate_launch_description = lambda: launch(sl := SimpleLauncher()) or sl.launch_description()
