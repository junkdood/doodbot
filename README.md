# dobot_magician

### run
```
rosrun dobot DobotServer ttyUSB0
rosrun dobot DobotClient_PTP
rosrun dobot DobotClient_JOG
```
or
```
roslaunch dobot jog.launch
```

### some ros command

```
rqt_graph
rosbag
```

### problem solved

1. `rosrun dobot DobotServer ttyUSB0` -> `Invalid port name or Dobot is occupied by other application!`
   The problem happens when “ttyUSB0” doesn’t have authority.
   Just give an authority to “ttyUSB0”.
   ```
   sudo chmod a+rw /dev/ttyUSB0
   ```
   or just run as
   ```
   rosrun dobot DobotServer /dev/tty
   ```
