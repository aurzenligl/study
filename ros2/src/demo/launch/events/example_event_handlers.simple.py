from simple_launch import SimpleLauncher
from simple_launch.events import When, OnProcessExit, OnProcessStart, OnShutdown, OnExecutionComplete, OnProcessIO
from launch.actions import EmitEvent
from launch.events import Shutdown
from launch.substitutions import EnvironmentVariable, LocalSubstitution


def launch(sl: SimpleLauncher):
    turtlesim_ns = sl.declare_arg('turtlesim_ns', 'turtlesim1')
    use_provided_red = sl.declare_arg('use_provided_red', 'False')
    new_background_r = sl.declare_arg('new_background_r', '200')

    with sl.group(ns=turtlesim_ns):
        sim = sl.node('turtlesim', 'turtlesim_node', name='sim')

        with sl.group(when=When(sim, OnProcessStart)):
            sl.log_info('Turtlesim started, spawning turtle')
            spawn_turtle = sl.call_service('spawn', {'x': 2., 'y': 2., 'theta': 0.2}, verbosity='reqres')

        with sl.group(when=When(spawn_turtle, OnProcessIO, io='stdout')):
            sl.add_action(lambda event:
                sl.log_info('Spawn request says "{}"'.format(event.text.decode().strip())))

        with sl.group(when=When(spawn_turtle, OnExecutionComplete)):
            sl.log_info('Spawn finished')
            sl.set_parameters('sim', {'background_r': 120})

            # TODO: this group acts as if it was added to root group rather than parent (the one from last "with" statement)
            with sl.group(when=When(delay=2.), if_condition=sl.py_eval('0 <= ', new_background_r, ' <= 255', ' and ', use_provided_red)):
                sl.set_parameters('sim', {'background_r': new_background_r})

        with sl.group(when=When(sim, OnProcessExit)):
            sl.log_info([EnvironmentVariable(name='USER'),' closed the turtlesim window']),
            sl.add_action(EmitEvent(event=Shutdown(reason='Window closed')))

        with sl.group(when=When(event=OnShutdown)):
            sl.log_info(['Launch was asked to shutdown: ', LocalSubstitution('event.reason')])


generate_launch_description = lambda: launch(sl := SimpleLauncher()) or sl.launch_description()
