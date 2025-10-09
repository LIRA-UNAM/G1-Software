# G1-Software
Software for the biped robot Unitree G1

# Steps to initialize G1:

1. Turn on (Press turn on button .- (short-release-long-release)) G1
2. ssh to 192.168.123.164 if wired or 192.168.1.123 if wireless (connected to LIR_UNAM)
3. Set the robot to damping mode. To check if the robot is correctly set to damping mode, you can try to move any joint. You should feel a little bit of resistance to movement. 

```
ros2 topic pub --once /api/loco/request unitree_api/msg/Request "
header:
  identity: {id: 501, api_id: 7101}
  lease: {id: 0}
  policy: {priority: 5, noreply: false}
parameter: '{\"data\":1}'
binary: []"
```

4. Execute the stan-up routine. The robot will move to a predefined position. Remember the robot is not balancing, then you must keep an eye on the robot.

```
ros2 topic pub --once /api/loco/request unitree_api/msg/Request "
header:
  identity: {id: 502, api_id: 7101}
  lease: {id: 0}
  policy: {priority: 5, noreply: false}
parameter: '{\"data\":4}'
binary: []"
```

5. Ensure the robot is prepared to balance (not hanging and with enough free space)
6. Set balance mode.

```
ros2 topic pub --once /api/loco/request unitree_api/msg/Request "
header:
  identity: {id: 505, api_id: 7102}
  lease: {id: 0}
  policy: {priority: 5, noreply: false}
parameter: '{\"data\":1}'
binary: []"
```

7. Execute the start routine to prepare the robot to receive high level commands:

```
ros2 topic pub --once /api/loco/request unitree_api/msg/Request "
header:
  identity: {id: 506, api_id: 7101}
  lease: {id: 0}
  policy: {priority: 5, noreply: false}
parameter: '{\"data\":500}'
binary: []"
```

8. Now you can use any of the high-level commands:

* Twist command (set a velocity $(x,y,\omega)$ with a predefined time $t$). According to experience, the minimum linear speed is 0.2. If you publish a new topic before finishing the previous movement, the previous one will be interrupted and the new one will be executed. 

```
ros2 topic pub --once /api/loco/request unitree_api/msg/Request "
header:
  identity: {id: 58, api_id: 7105}
  lease: {id: 0}
  policy: {priority: 1, noreply: false}
parameter: '{\"velocity\":[0.0,0.0,-0.5],\"duration\":1.0}'
binary: []"
```

* Move the feet resembling a gait, but the robot remains in the same position (it actually moves due to uncertainties and misscalibration)

```
ros2 topic pub --once /api/loco/request unitree_api/msg/Request "
header:
  identity: {id: 51, api_id: 7102}
  lease: {id: 0}
  policy: {priority: 1, noreply: false}
parameter: '{\"data\":1}'
binary: []"
```

* Stop marching.

```
ros2 topic pub --once /api/loco/request unitree_api/msg/Request "
header:
  identity: {id: 90, api_id: 7102}
  lease: {id: 0}
  policy: {priority: 1, noreply: false}
parameter: '{\"data\":0}'
binary: []"
```

* Execute a greeting:

```
ros2 topic pub --once /api/loco/request unitree_api/msg/Request "
header: {identity: {id: 803, api_id: 7106}, lease: {id: 0}, policy: {priority: 1, noreply: false}}
parameter: '{\"data\":2}'
binary: []"
```

* Stop greeting:

```
ros2 topic pub --once /api/loco/request unitree_api/msg/Request "
header: {identity: {id: 804, api_id: 7106}, lease: {id: 0}, policy: {priority: 1, noreply: false}}
parameter: '{\"data\":3}'
binary: []"
```

* Turn waist and greet:

```
ros2 topic pub --once /api/loco/request unitree_api/msg/Request "
header: {identity: {id: 802, api_id: 7106}, lease: {id: 0}, policy: {priority: 1, noreply: false}}
parameter: '{\"data\":1}'
binary: []"
```

* Say something:

```
ros2 topic pub --once /api/voice/request unitree_api/msg/Request "
header:
  identity: {id: 6101, api_id: 1001}
  lease: {id: 0}
  policy: {priority: 1, noreply: false}
parameter: '{\"index\":1, \"speaker_id\":0, \"text\":\"Hello, how are you?\"}'
binary: []"
```