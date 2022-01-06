# opti_track

大隔间高速运动捕捉系统OptiTrack的ROS节点包

# 环境配置

安装virtualenv：
```
# for ubuntu 16.04
sudo apt-get install virtualenv
# for ubuntu 14.04
sudo apt-get install python-virtualenv
```

初始化ROS仓库及编译节点代码：
```
mkdir -p ~/opti_ws/src && cd ~/opti_ws/src
git clone git@gitlab.com:sysulion/lionbrain-plugins/opit_track.git
cd ..
catkin_make
```

在虚拟环境下安装对应依赖：
```
virtualenv -p /usr/bin/python3 ~/opti_ws/venv
source ~/opti_ws/venv/bin/activate
pip install catkin_pkg 
pip install catkin_tools 
pip install empy 
pip install rospkg 
pip install pyyaml
pip install numpy
chmod +x ~/opti_ws/src/bebop_odom/scripts/opti_odom.py
```

# 修改opti主机对应的IP
将bebop_odom文件夹里script里的opti_odom.py的main函数里的这行代码，把ip地址改为opti主机的无线网络IP（系统完成初始化后，可以在主机软件上查询和设置IP地址）

```py
streamingClient = NatNetClient("196.168.0.112") 
```

# 运行节点
```
source ~/opti_ws/venv/bin/activate (不要写进.bashrc)
source ~/opti_ws/devel/setup.bash 
rosrun bebop_odom opti_odom.py

此时如果出现其他终端无法运行ros命令，出现syntax error报错，
说明python版本有问题，需要执行以下命令：
```
