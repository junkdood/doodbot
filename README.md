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

### control

| key | move |
| :---: | :----: |
|w|forward|
|a|left|
|s|backward|
|d|right|
|u|up|
|i|down|
|j|hand counterclockwise|
|k|hand clockwise|
|others|stop|


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

2. 因为机械臂关机之后关节不会锁定，重新开机后会有极大的误差，一定要跑归零
3. c++ 安装 casadi
   ```sh
   sudo apt-get install coinor-libipopt-dev
   sudo apt install gfortran
   sudo apt install swig
   git clone https://github.com/casadi/casadi.git -b master casadi 
   cd casadi
   mkdir build
   cd build
   cmake -DWITH_PYTHON=ON -DWITH_IPOPT=true ..
   make
   sudo make install
   ```