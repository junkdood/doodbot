### 记录

#### 2022-1-1
找到了靠谱的底层控制方式
dobot magician 提供了点到点的控制方式和直接控制关节转动的方式。鉴于点到点的控制方式应该是内嵌了类似PID的算法，封装程度较高，为了能体现个人控制算法的精度效果暂时先用直接控制关节转动的方式。

直接控制关节转动需要如下步骤：预设速度与角速度，然后直接发送转动命令，再发送停止命令实现一段位移。因此，需要自己预估时间。一开始为使用的是 ros::Duration 。但命令发送有较大的时延波动，按照速度乘时间来算位移会有几度的误差，无法接受。因此，采取的第二个方案是利用机械臂自带的人物队列功能，在转动命令和停止命令之间插入一条等待命令，再统一执行。这样，误差降低并稳定到+0.2度左右。这个数值是在固定参数下观测到的，仍须更多数据支撑，不过已经比 ros::Duration 好了。初步判断是机械臂任务队列指令切换延迟导致的。可以考虑借用论文方法消除误差。

下一任务：使用 casadi 求解库实现简单点到点规划。ROS 的 MoveIt! 插件是专门针对机械臂规划的，但是好像封装程度还是太高，暂时不用。


#### 2022-1-2
几何求解了该机械臂的运动学方程和对应逆解

直接用余弦定理求解x，y，z坐标与机械臂关节的关系，公式如下。需要注意机械臂关节角度能取到什么范围，好区分角度大于小于0的情况。

<img src="assets/arm_physics.png" alt="arm_physics" style="zoom:50%;" />

对应伪代码
```c++
void xyz_to_jointAngle(float x, float y, float z, float jointAngle[4]){
    double r_2 = x*x + y*y;
    double d_2 = r_2 + z*z;
    double d = sqrt(d_2);
    jointAngle[0]=asin(y/sqrt(r_2));
    jointAngle[1]=acos(z/d) - acos((d_2 + _l1_2 - _l2_2)/(2*_l1*d));
    jointAngle[2]=acos((d_2 + _l2_2 - _l1_2)/(2*_l2*d)) - asin(z/d);
    return;
}

void jointAngle_to_xyz(float jointAngle[4], float &x, float &y, float &z){
    double r = _l1*sin(j1) + _l2*cos(j2);
    x = r*cos(j0);
    y = r*sin(j0);
    z = _l1*cos(j1) - _l2*sin(j2);
    return;
}
```
测试了许多姿态下都能正确计算，暂时没有发现bug

下一任务：使用 casadi 求解库实现简单点到点规划。

#### 2022-1-3
安装casadi，还在考虑用c写还是python
决定再重读一下论文在继续写

下一任务：使用 casadi 求解库实现简单点到点规划。

#### 2022-1-7
开始写求解器的类

发现构建的运动学模型几乎是线性的，根本不需要用上复杂的用于求解非线性规划(nlp)的微分方程求解器。
不过先实现一个杀鸡用牛刀的版本吧。

下一任务：继续实现。