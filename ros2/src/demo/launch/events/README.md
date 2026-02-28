Perform test:
```
$ ros2 launch launch/events/example_event_handlers.simple.py turtlesim_ns:='turtlesim3' use_provided_red:='True' new_background_r:=200
```

With bug fix:
```
$ cat /home/aurzenligl/.ros/log/2026-02-26-11-34-57-575431-legion5-docker-3459244/launch.log
1772105697.5765109 [INFO] [launch]: All log files can be found below /home/aurzenligl/.ros/log/2026-02-26-11-34-57-575431-legion5-docker-3459244
1772105697.5767016 [INFO] [launch]: Default logging verbosity is set to INFO
1772105697.6502585 [INFO] [turtlesim_node-1]: process started with pid [3459257]
1772105697.6506419 [INFO] [launch.user]: Turtlesim started, spawning turtle
1772105697.6544344 [INFO] [call_service-2]: process started with pid [3459259]
1772105697.7248943 [turtlesim_node-1] [INFO] [1772105697.724506844] [turtlesim3.sim]: Starting turtlesim with node name /turtlesim3/sim
1772105697.7305982 [turtlesim_node-1] [INFO] [1772105697.730273144] [turtlesim3.sim]: Spawning turtle [turtle1] at x=[5.544445], y=[5.544445], theta=[0.000000]
1772105697.9790106 [call_service-2] [INFO] [1772105697.978677180] [turtlesim3.call_service_aJGqqvzn]: Checking service /turtlesim3/spawn
1772105700.1290376 [call_service-2] [INFO] [1772105700.128813691] [turtlesim3.call_service_aJGqqvzn]: Calling service /turtlesim3/spawn
1772105700.1694114 [turtlesim_node-1] [INFO] [1772105700.169055206] [turtlesim3.sim]: Spawning turtle [turtle2] at x=[2.000000], y=[2.000000], theta=[0.200000]
1772105700.1703796 [call_service-2] [INFO] [1772105700.170095903] [turtlesim3.call_service_aJGqqvzn]: /turtlesim3/spawn response is {'name': 'turtle2'}
1772105700.1752408 [call_service-2] /turtlesim3/spawn response is {'name': 'turtle2'}
1772105700.1758497 [INFO] [launch.user]: Spawn request says "/turtlesim3/spawn response is {'name': 'turtle2'}"
1772105700.3246629 [INFO] [call_service-2]: process has finished cleanly [pid 3459259]
1772105700.3254416 [INFO] [launch.user]: Spawn finished
1772105700.3298693 [INFO] [set_parameters-3]: process started with pid [3459383]
1772105700.7767828 [INFO] [set_parameters-3]: process has finished cleanly [pid 3459383]
1772105702.3370051 [INFO] [set_parameters-4]: process started with pid [3459438]
1772105702.7928996 [INFO] [set_parameters-4]: process has finished cleanly [pid 3459438]
1772105705.0938308 [INFO] [turtlesim_node-1]: process has finished cleanly [pid 3459257]
1772105705.0970383 [ERROR] [launch]: Caught exception in launch (see debug for traceback): environment variable 'USER' does not exist
1772105705.0977468 [INFO] [launch.user]: Launch was asked to shutdown: Caught exception in launch (see debug for traceback): environment variable 'USER' does not exist

note 2 second delay:
1772105700.3298693 [INFO] [set_parameters-3]: process started with pid [3459383]
1772105702.3370051 [INFO] [set_parameters-4]: process started with pid [3459438]
```

Without bug fix:
```
$ cat /home/aurzenligl/.ros/log/2026-02-26-11-39-01-628859-legion5-docker-3464336/launch.log
1772105941.6298618 [INFO] [launch]: All log files can be found below /home/aurzenligl/.ros/log/2026-02-26-11-39-01-628859-legion5-docker-3464336
1772105941.6299961 [INFO] [launch]: Default logging verbosity is set to INFO
1772105941.7089334 [INFO] [turtlesim_node-1]: process started with pid [3464343]
1772105941.7092974 [INFO] [launch.user]: Turtlesim started, spawning turtle
1772105941.7127531 [INFO] [call_service-2]: process started with pid [3464344]
1772105941.7951066 [turtlesim_node-1] [INFO] [1772105941.794773424] [turtlesim3.sim]: Starting turtlesim with node name /turtlesim3/sim
1772105941.8015523 [turtlesim_node-1] [INFO] [1772105941.801292195] [turtlesim3.sim]: Spawning turtle [turtle1] at x=[5.544445], y=[5.544445], theta=[0.000000]
1772105942.0371673 [call_service-2] [INFO] [1772105942.036903400] [turtlesim3.call_service_QAmqOobf]: Checking service /turtlesim3/spawn
1772105943.5138719 [call_service-2] [INFO] [1772105943.513512018] [turtlesim3.call_service_QAmqOobf]: Calling service /turtlesim3/spawn
1772105943.5505679 [turtlesim_node-1] [INFO] [1772105943.550323254] [turtlesim3.sim]: Spawning turtle [turtle2] at x=[2.000000], y=[2.000000], theta=[0.200000]
1772105943.5527923 [call_service-2] [INFO] [1772105943.552400008] [turtlesim3.call_service_QAmqOobf]: /turtlesim3/spawn response is {'name': 'turtle2'}
1772105943.5584710 [call_service-2] /turtlesim3/spawn response is {'name': 'turtle2'}
1772105943.5590067 [INFO] [launch.user]: Spawn request says "/turtlesim3/spawn response is {'name': 'turtle2'}"
1772105943.7037578 [INFO] [call_service-2]: process has finished cleanly [pid 3464344]
1772105943.7045486 [INFO] [launch.user]: Spawn finished
1772105943.7109196 [INFO] [set_parameters-3]: process started with pid [3464426]
1772105943.7112036 [INFO] [set_parameters-4]: process started with pid [3464427]
1772105944.4943454 [INFO] [set_parameters-4]: process has finished cleanly [pid 3464427]
1772105944.5103774 [INFO] [set_parameters-3]: process has finished cleanly [pid 3464426]
1772105946.9527469 [INFO] [turtlesim_node-1]: process has finished cleanly [pid 3464343]
1772105946.9553454 [ERROR] [launch]: Caught exception in launch (see debug for traceback): environment variable 'USER' does not exist
1772105946.9562895 [INFO] [launch.user]: Launch was asked to shutdown: Caught exception in launch (see debug for traceback): environment variable 'USER' does not exist

note 2 second delay:
1772105941.7089334 [INFO] [turtlesim_node-1]: process started with pid [3464343]
1772105943.7112036 [INFO] [set_parameters-4]: process started with pid [3464427]
```
