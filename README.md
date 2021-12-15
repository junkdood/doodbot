# dobot_magician

### run
```sh
rosrun dobot DobotServer ttyUSB0
rosrun dobot DobotClient_PTP
rosrun dobot DobotClient_JOG
```
or
```sh
roslaunch dobot jog.launch
```

### some ros command

```sh
rqt_graph
rosbag
```

### problem solved

1. `rosrun dobot DobotServer ttyUSB0` -> `Invalid port name or Dobot is occupied by other application!`
   The problem happens when “ttyUSB0” doesn’t have authority.
   Just give an authority to “ttyUSB0”.
   ```sh
   sudo chmod a+rw /dev/ttyUSB0
   ```
   or just run as
   ```sh
   rosrun dobot DobotServer /dev/tty
   ```
