# Franka Panda 操作指南

### 一、环境配置

本项目采用客户-服务器的方式来控制机械臂。服务器电脑和Franka Panda机械臂相连接，用来接受来自客户端的命令并控制机械臂。服务器环境配置详见 https://github.com/justagist/franka_ros_interface

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