
## 1 底盘上位机运动测试

（1）关闭遥控器；或者打开遥控器，同时，使能拨杆拨往上方上位机模式，并且确保急停拨杆没有拨往下方的急停状态。

（2）进入ros的工作空间，运行如下命令,编译segway_msgs包。
```
cd catkin_ws
catkin_make -DCATKIN_WHITELIST_PACKAGES='segway_msgs'
```

（3）进入ros的工作空间，运行如下命令,编译segwayrmp包。

```
cd catkin_ws
catkin_make -DCATKIN_WHITELIST_PACKAGES='segwayrmp'
```

（4）执行测试，为避免受之前打开的terminal的影响，建议先关掉之前的terminal，打开三个terminal。

```
cd catkin_ws
roscore
```

------

```
cd catkin_ws
source devel/setup.bash
rosrun segwayrmp SmartCar
```

------

```
cd catkin_ws
source devel/setup.bash
rosrun segwayrmp ChassisResponseTest
```



------

## 2 主要文件路径说明

（1）segway_msgs：自定义消息package

​					segway_msgs/msg：自定义消息

（2）segwayrmp：SmartCar节点所在package

​					segwayrmp/include：节点所需头文件路径

​					segwayrmp/lib：ROS所需上位机动态库所在路径

​					segwayrmp/src：和底盘数据交互SmartCar节点源文件程序

​					segwayrmp/tools：运动测试ChassisResponseTest节点源程序

## 3 ROS节点串口设置

​		  串口设置的参数（param）名称固定为“segwaySmartCarSerial”，

​		  在测试案例节点“chassis_response_test”中，ChassisResponseTest.cpp中设置串口命令：n_.setParam("segwaySmartCarSerial", "ttyUSB0");

​		  n_									 ：ROS node；

​          segwaySmartCarSerial ：ROS param设置串口的固定名称；

​		  **ttyUSB0				        : 串口名称，根据实际需要修改此值。**

​		  可按需将此命令移植到其他节点。