# Franka Panda/Kinova Gen3 操作指南

## Panda篇

### 一、环境配置

本项目采用客户-服务器的方式来控制机械臂。服务器电脑和Franka Panda机械臂相连接，用来接受来自客户端的命令并控制机械臂。服务器环境配置详见本项目`Franka环境配置.md`。

### 二、操作流程

1. 开启FCI控制接口：服务器电脑和Franka Panda控制台用网线连接后，对服务器的网络IPv4进行设置，更改项：

   + IPv4 Method：Manual
   + IP: `172.16.0.1`
   + Netmask: `255.255.255.0`

   设置后，登录网址`172.16.0.2`. 对机械臂关节进行解锁，然后激活FCI。激活后，机械臂底座的灯由黄色变成蓝色，蓝色灯亮说明机械臂采用程序进行控制。在蓝灯亮时，可以上拉机械臂旁边的灰色按钮，这是底座灯会由蓝色变成白色。白色灯亮说明机械臂可以手动控制。下压灰色按钮可以让灯恢复为蓝色。

2. 服务器电脑开启服务：

   + 文件准备：在配置好环境之后，我们有一个工作空间目录。在机器人笔记本上，工作空间目录是`/home/panda_control_ws`。为了实现客户-服务器方式是控制机械臂，我们添加了`panda_server.py`文件，用来监听客户端的命令并控制机械臂。

   + 开启控制接口服务(服务1)：开启该接口服务后，才能通过程序来控制机械臂
     在`panda_control_ws`目录下，运行

     ```bash
     ./franka.sh master
     roslaunch franka_interface interface.launch
     ```

     当终端中出现`You can start planning now!`， 说明成功开启服务

   + 开启通信服务(服务2)：开启该接口服务后，才能接受客户端命令控制机械臂

     在`panda_control_ws`目录下，运行

     ```bash
     ./franka.sh master
     python3 panda_server.py
     ```

3. 客户端控制机械臂：

   服务器的各种命令接口在`panda_server.py`文件中，可以对这个文件进行修改从而适应自己的控制需求。目前，该文件仅控制二指末端执行器，多指需要进行修改。在客户端发送命令时，需要查询服务器电脑的ip，并在`panda_server.py`文件中找到设置的通信端口port。

   :star: 客户端获取机械臂信息的命令为：

   ```python
   import requests
   res = requests.get(f'http://ip:port/{order}')
   ```

   其中，order可以填入：

   + `joint_angles`: 获取机械臂七个关节的旋转角度
   + `pose`: 获取机械臂末端执行的的位姿，形式为(tx,ty,tz,qw,qx,qy,qz)
   + `open`: 打开夹爪
   + `close`: 关闭夹爪

   :star: 客户端发送执行命令为：

   ```python
   res = requests.pose(f'http://ip:port/{order}', parameter)
   ```

   其中，order可以填入：

   + `joint_angles`: 机械臂七个关节到达的旋转角度，parameter的形式为`{ 'joint1': angles[0], 'joint2': angles[1],'joint3': angles[2],'joint4': angles[3],'joint5': angles[4],'joint6': angles[5],'joint7': angles[6], }`
   + `pose`: 机械臂末端执行器到达的位姿，parameter的形式为`{"tx": pose[0],"ty": pose[1],"tz": pose[2],"qw": pose[3],"qx": pose[4],"qy": pose[5],"qz": pose[6],}`

## Kinova篇

### 一、环境配置

同样采用客户-服务器的方式来控制机械臂。服务器电脑和Kinova Gen3机械臂相连接，用来接受来自客户端的命令并控制机械臂。安装docker，拉取环境镜像

```
docker pull kwokho/ros_kortex:melodic-pose-socket
```

运行容器

```
docker run \
-itd \
-w /root/catkin_workspace \
--hostname $(hostname) \
--name kinova_container \
--network host \
kwokho/kortex_http:melodic
```

:star:将`/root/catkin_workspace/src/ros_kortex/kortex_app/src/basic_control`目录下的`server.py`文件替换为本项目`kinova_server.py`文件。

详细参考项目[kwokho/kortex_http - Docker Image | Docker Hub](https://hub.docker.com/r/kwokho/kortex_http)

### 二、操作流程

1. 服务器电脑开启服务：

```bash
# 运行驱动接口
bash kortex_driver.sh
# 开启服务
source /root/catkin_workspace/devel/setup.bash
cd /root/catkin_workspace/src/ros_kortex/kortex_app/src/basic_control
python3 kinova_server.py --host 0.0.0.0 --port 8888
```

2. 客户端控制机械臂：

   服务器的各种命令接口在`kinova_server.py`文件中，可以对这个文件进行修改从而适应自己的控制需求。目前，该文件仅控制二指末端执行器，多指需要进行修改。在客户端发送命令时，需要查询服务器电脑的ip，并在`kinova_server.py`文件中找到设置的通信端口port。

   :star: 客户端获取机械臂信息的命令为：

   ```python
   import requests
   res = requests.get(f'http://ip:port/{order}')
   ```

   其中，order可以填入：

   + `joint_angles`: 获取机械臂七个关节的旋转角度
   + `pose`: 获取机械臂末端执行的的位姿，形式为(tx,ty,tz,qw,qx,qy,qz)
   + `stop`: 停止机械臂
   + `gripper_width`: 获取夹爪宽度

   :star: 客户端发送执行命令为：

   ```python
   res = requests.pose(f'http://ip:port/{order}', parameter)
   ```

   其中，order可以填入：

   + `joint_angles`: 机械臂七个关节到达的旋转角度，parameter的形式为`{ 'joint1': angles[0], 'joint2': angles[1],'joint3': angles[2],'joint4': angles[3],'joint5': angles[4],'joint6': angles[5],'joint7': angles[6], }`
   + `pose`: 机械臂末端执行器到达的位姿，parameter的形式为`{"tx": pose[0],"ty": pose[1],"tz": pose[2],"rx": pose[3],"ry": pose[4],"rz": pose[5]}`
   + `gripper`: 控制夹爪宽度，实现夹爪开闭，parameter的形式为`{"width": width}`

## Realsense深度相机配置

### 一、环境配置

- 安装librealsense，从而使用realsense-viewer检测相机

```Bash
sudo mkdir -p /etc/apt/keyrings
curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null

echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | \
sudo tee /etc/apt/sources.list.d/librealsense.list
sudo apt-get update

sudo apt-get install librealsense2-dkms
sudo apt-get install librealsense2-utils
sudo apt-get install librealsense2-dev
sudo apt-get install librealsense2-dbg
```

- 安装pyrealsense2

```Bash
pip install pyrealsense2
```

- 下载本项目`realsense.py`和`realsense_server.py`文件

### 二、操作流程

- 实用工具：realsense-viewer, 运行之后可以检测相机时候正常，根据可视化界面摆好相机。realsense-viewer运行时，控制程序没能运行。注意程序控制时要关闭。
- 运行服务

```
python3 realsense_server.py
```

- 客户端同样通过ip和端口获取信息，包括`depth, color, intrinsic_color, intrinsic_depth`

## 参考项目

根据下面两个项目建议仔细阅读，从而在`kinova/panda_server.py`文件中实现更多的功能

- [panda_robot.](https://github.com/justagist/panda_robot)
- [Kinovarobotics/ros_kortex](https://github.com/Kinovarobotics/ros_kortex/tree/noetic-devel)

