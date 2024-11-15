# Franka环境配置

## 目录

- [安装Linux系统](#安装Linux系统)
- [编译Linux实时内核](#编译Linux实时内核)
- [安装ROS系统](#安装ROS系统)
- [安装libfranka,franka-ros,panda\_moveit\_config](#安装libfrankafranka-rospanda_moveit_config)
- [下载franka\_panda\_description](#下载franka_panda_description)
- [安装franka\_ros\_interface](#安装franka_ros_interface)
- [测试运行](#测试运行)

### 安装Linux系统

linux版本限制在Ubuntu 18.04 LTS，Ubuntu 20.04 LTS&#x20;

### 编译Linux实时内核

> 安装实时内核的目的是使用 `libfranka`.&#x20;

1. **安装依赖**

```bash
sudo apt-get install build-essential bc curl ca-certificates gnupg2 libssl-dev lsb-release libelf-dev bison flex dwarves zstd libncurses-dev
```

2. **根据系统的内核版本选择实时内核。系统内核版本查看** `uname -r`. [**实时patches**](https://www.kernel.org/pub/linux/kernel/projects/rt/ "实时patches")**要下载对应的版本。例如**

```bash
For Ubuntu 20.04 tested with the kernel version 5.9.1:

curl -SLO https://www.kernel.org/pub/linux/kernel/v5.x/linux-5.9.1.tar.xz
curl -SLO https://www.kernel.org/pub/linux/kernel/v5.x/linux-5.9.1.tar.sign
curl -SLO https://www.kernel.org/pub/linux/kernel/projects/rt/5.9/patch-5.9.1-rt20.patch.xz
curl -SLO https://www.kernel.org/pub/linux/kernel/projects/rt/5.9/patch-5.9.1-rt20.patch.sign
```

3. **解压缩**

```bash
xz -d *.xz 
```

4. **sign文件验证**

```bash
gpg2 --verify linux-*.tar.sign
gpg2 --verify patch-*.patch.sign
```

```bash
$ gpg2 --verify linux-*.tar.sign
gpg: assuming signed data in 'linux-4.14.12.tar'
gpg: Signature made Fr 05 Jan 2018 06:49:11 PST using RSA key ID 6092693E
gpg: Can't check signature: No public key 错误输出
```

```bash
$ gpg2 --verify linux-*.tar.sign
gpg: assuming signed data in 'linux-4.14.12.tar'
gpg: Signature made Fr 05 Jan 2018 06:49:11 PST using RSA key ID 6092693E
gpg: Good signature from "Greg Kroah-Hartman <gregkh@linuxfoundation.org>" [unknown]
gpg:                 aka "Greg Kroah-Hartman <gregkh@kernel.org>" [unknown]
gpg:                 aka "Greg Kroah-Hartman (Linux kernel stable release signing key) <greg@kroah.com>" [unknown]
gpg: WARNING: This key is not certified with a trusted signature!
gpg:          There is no indication that the signature belongs to the owner.
Primary key fingerprint: 647F 2865 4894 E3BD 4571  99BE 38DB BDC8 6092 693E 正确输出
```

如果出现错误输出, 根据错误输出中的 `ID 6092693E`, 从密钥服务器获取它。ID就是 `recv-keys`的参数

```bash
gpg2  --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys 6092693E 错误解决
```

5. **内核配置**

```bash
tar xf linux-*.tar
cd linux-*/
patch -p1 < ../patch-*.patch

cp -v /boot/config-$(uname -r) .config ## 将当前启动的内核配置复制为新实时内核的默认配置
make olddefconfig
make menuconfig
```

`make menuconfig`命令会打开一个终端界面，在其中对内核进一步修改，实际上就是修改`.config`：

- 使用箭头键导航至`General Setup` > *`Preemption Model`*>*`Fully Preemptible Kernel (Real-Time)`*
- 回到主目录，导航至 *`Cryptographic API`* > *`Certificates for signature checking`* > *`Provide system-wide ring of trusted keys`* > *`Additional X.509 keys for default system keyring`*, 删除`debian/canonical-certs.pem`, 确认并保存。
- 退出终端界面，一直exit

6. **编译并安装内核**

```bash
make -j8 deb-pkg     # 编译
sudo dpkg -i ../linux-headers-*.deb ../linux-image-*.deb   # 安装
```

7. **校验结果**

```bash
# 当前内核查看
cd /boot
ls | grep rt
## 输出
## config-5.9.1-rt20
## initrd.img-5.9.1-rt20
## System.map-5.9.1-rt20
## vmlinuz-5.9.1-rt20
```

```bash
# 进入实时内核查看
reboot
uname -r
# 输出
5.9.1-rt20  # 有rt
```

8. **设置用户权限以使用RT实时调度**

```bash
sudo groupadd realtime                   # 建立实时用户组
sudo usermod -aG realtime $(whoami)      # 将当前用户加入实时用户组
```

为了能够以用户权限调度线程（驱动程序将执行此操作），更改`/etc/security/limits.conf`文件来修改用户的限制

```bash
sudo vim /etc/security/limits.conf
# 写入文件的内容
@realtime soft rtprio 99
@realtime soft priority 99
@realtime soft memlock 204800
@realtime hard rtprio 99
@realtime hard priority 99
@realtime hard memlock 204800
```

其中，`rtprio`是实时调度的优先级，`memlock`是内存锁定的大小，单位为`KB`，防止应用程序的内存被交换到磁盘（即锁定内存），从而保证高性能.

9. **测试实时性**

- 重启电脑。在boot菜单中有Advance选项，进去选项中，能够找到新安装的内核。选择该内核启动电脑，在终端中运行 `uname -a`, 输出内核版本中带有 `PREEMPT RT`字符串。
- 使用 `ulimit -r` 命令来验证当前用户的实时优先级限制，这个命令会返回用户的最大实时优先级。正常情况下，输出应该是 99。如果不是，说明步骤8配置没有正确应用
- 安装rt\_test测试工具

```bash
sudo apt-get install rt-tests 
# 运行测试 5个线程，线程优先级99，以ms单位显示时间
sudo cyclictest -t 5 -p 99 -N -m
```

[【linux打实时补丁 | ubuntu20.04安装实时内核补丁PREEMPT\_RT及正确启用RT】\_ csdn ubuntu20.04安装实时内核-CSDN博客](https://blog.csdn.net/qq_46387453/article/details/143108869 "【linux打实时补丁 | ubuntu20.04安装实时内核补丁PREEMPT_RT及正确启用RT】_csdn ubuntu20.04安装实时内核-CSDN博客")

[  https://frankaemika.github.io/docs/installation\_linux.html](https://frankaemika.github.io/docs/installation_linux.html "  https://frankaemika.github.io/docs/installation_linux.html")

### 安装ROS系统

下面的各类安装在普通内核安装即可

- ROS版本对应关系
  | ros版本   | ubuntu版本  | python版本 | Franka ROS Branch                                                                                 |
  | ------- | --------- | -------- | ------------------------------------------------------------------------------------------------- |
  | Melodic | 18.04 LTS | 2.7+     | \[melodic-devel]\(<https://github.com/frankaemika/franka_ros/tree/melodic-devel> "melodic-devel") |
  | Noetic  | 20.04 LTS | 3.6+     | \[noetic-devel]\(<https://github.com/frankaemika/franka_ros/tree/noetic-devel> "noetic-devel")    |

检查系统是否存在ros系统

```bash
ls /opt/ros       # 查看是否有ros文件夹以及文件夹下面是否存在对应的ros版本
rosversion -d     # 如果存在ros且设置好环境变量，该命令会输出ros版本
# 为ros设置环境变量
vim ~/.bashrc
source /opt/ros/noetic/setup.bash
```

[ 安装教程](https://zhuanlan.zhihu.com/p/515361781 "   https://zhuanlan.zhihu.com/p/515361781")

### 安装libfranka,franka-ros,panda\_moveit\_config

- 采用命令行安装

```bash
sudo apt install ros-$ROS_DISTRO-libfranka
sudo apt install ros-$ROS_DISTRO-franka-ros
sudo apt install ros-$ROS_DISTRO-panda-moveit-config
```

- 采用源代码安装

[libfranka](https://frankaemika.github.io/docs/installation_linux.html#building-from-source "libfranka")，[franka\_ros](https://github.com/frankaemika/franka_ros "franka_ros")，[panda\_moveit\_config](https://github.com/moveit/panda_moveit_config "panda_moveit_config").&#x20;

注意选择与ROS版本对应的代码分支

### 下载[franka\_panda\_description](https://github.com/justagist/franka_panda_description "franka_panda_description")

在主目录创建目录 `panda_ws`作为工作空间，在该目录下创建 `src` 文件夹。在 `src`文件夹下下载franka\_panda\_description软件包

```bash
git clone https://github.com/justagist/franka_panda_description.git
```

### 安装franka\_ros\_interface

```bash
cd panda_ws
git clone -b v0.7.1-dev https://github.com/justagist/franka_ros_interface src/franka_ros_interface
catkin build # or catkin_make (catkin build is recommended)
source devel/setup.bash
cp src/franka_ros_interface/franka.sh ./
# 根据franka.sh文件中的提示修改文件,包括设置ip，修改ros版本
vim franka.sh

```

```bash
# ----- EDIT THIS TO MATCH THE IP OF THE FRANKA ROBOT CONTROLLER IN THE NETWORK
FRANKA_ROBOT_IP="172.16.0.2"   # 修改

# ----- EDIT THIS TO MATCH YOUR IP IN THE NETWORK
your_ip="172.16.0.1"     # 修改

# ----- EDIT THIS TO MATCH THE IP OF THE MAIN ROS MASTER PC (CONNECTED TO THE FRANKA CONTROLLER),
# ----- ON WHICH THE Franka ROS Interface 'driver' NODES WILL BE RUNNING (SEE "USAGE EXPLAINED" BELOW).
# ----- THIS VALUE IS NEEDED ONLY IF YOU WILL USE THE 'remote' ENVIRONMENT ON THIS MACHINE.
ROS_MASTER_IP=""

ros_version="noetic"    # 修改 franka.sh文件
```

### 测试运行

```bash
./franka.sh master
roslaunch franka_interface interface_launch
 运行franka接口
```

- fake\_execution参数错误，到报错的文件中修改，将fake\_execution\_type修改为fake\_execution
- `Exception while loading planner 'ompl_interface/OMPLPlanner': According to ...`

```bash
sudo apt-get install ros-noetic-moveit   # 根据ros版本修改
```

- `import quaternion`报错找不到模块

```bash
python -m pip install --no-deps numpy-quaternion
```
