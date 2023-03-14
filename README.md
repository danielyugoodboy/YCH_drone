# YCH_drone
This is a side project about Px4 on Gazebo
當前版本：1.0.0

* 連結:

    * < 主要安裝參考 > 無人機自動駕駛軟件系列 E01：OFFBOARD控制以及Gazebo仿真

        https://gaas.gitbook.io/guide/software-realization-build-your-own-autonomous-drone/wu-ren-ji-zi-dong-jia-shi-xi-lie-offboard-kong-zhi-yi-ji-gazebo-fang-zhen

    * < 官方安裝教學 > PX4 ROS
    
        https://docs.px4.io/main/zh/ros/ros1.html

    * < PX4 飛控介紹 >
    
        https://docs.px4.io/v1.12/zh/getting_started/px4_basic_concepts.html

    * 參考文件：https://github.com/danielyugoodboy/NCRL-AIDrone-Platform/tree/master/src
    * 飛行模式：https://docs.px4.io/main/zh/getting_started/flight_modes.html
    * MAVROS Basics: http://edu.gaitech.hk/gapter/mavros-basics.html
    * MAVROS_Tutorial: https://masoudir.github.io/mavros_tutorial/


## 0. 電腦配置
* Ubuntu 18.04
* ROS melodic
* python=3.9 (推薦)


## 1. 安裝教學
### 1-1. 安裝ROS

* 請直接參考 http://wiki.ros.org/ROS/Tutorials 中，1.Core ROS Tutorials -> 1.1 Beginner Level -> 1. Installing and Configuring Your ROS Environment 的安裝介紹安裝melodic的版本。

### 1-2. 安裝之後步驟所需之依賴

* 安裝 python 工具
    ```
    $ sudo apt install -y \
        ninja-build \
        exiftool \
        python-argparse \
        python-empy \
        python-toml \
        python-numpy \
        python-yaml \
        python-dev \
        python-pip \
        ninja-build \
        protobuf-compiler \
        libeigen3-dev \
        genromfs 
    ```

    ```
    $ pip install \
        pandas \
        jinja2 \
        pyserial \
        cerberus \
        pyulog \
        numpy \
        toml \
        pyquaternion
    ```

    ```
    $ pip install empy
    $ pip3 install empy
    ```

* 安裝 catkin-tools & rosinstall
    ```
    $ sudo apt-get install python-catkin-tools python-rosinstall-generator -y
    ```

### 1-3. 創建工作環境

* 建立workspace
    ```
    $ mkdir -p ~/drone_ws/src
    $ cd ~/drone_ws/
    $ catkin init && wstool init src
    ```

* 安裝 mavlink mavros
    ```
    $ rosinstall_generator --rosdistro melodic mavlink | tee /tmp/mavros.rosinstall
    $ rosinstall_generator --upstream mavros | tee -a /tmp/mavros.rosinstall
    $ wstool merge -t src /tmp/mavros.rosinstall
    $ wstool update -t src -j4
    $ rosdep install --from-paths src --ignore-src -y
    ```

* 安装geographic lib :
    ```
    $ sudo ./src/mavros/mavros/scripts/install_geographiclib_datasets.sh
    $ sudo apt install ros-melodic-catkin python-catkin-tools
    ```

* 編譯
    ```
    $ catkin build
    ```

### 1-4. 安裝 PX4 韌體

* 安裝
    ```
    # 下載px4
    $ cd ~/drone_ws/src
    $ git clone https://github.com/PX4/Firmware.git

    # 編譯
    $ cd Firmware
    $ git checkout v1.8.0
    $ make posix_sitl_default gazebo
    ```
* 編譯完成後會出現以下畫面，點x關閉即可
    ![](https://i.imgur.com/p8MeOlW.png)

* 增加環境變數
    ```
    $ nano ~/.bashrc
    ```

* 將以下內容貼到最下面，並且存檔退出
    ```
    source ~/drone_ws/devel/setup.bash
    source ~/drone_ws/src/Firmware/Tools/setup_gazebo.bash ~/drone_ws/src/Firmware/ ~/drone_ws/src/Firmware/build/posix_sitl_default
    export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/drone_ws/src/Firmware
    export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/drone_ws/src/Firmware/Tools/sitl_gazebo
    ```

* source 環境
    ```
    $ source ~/.bashrc
    ```

### 1-5. 建立YCH_drone環境包

#### 從github下載

* 下載 (下載下來的預設 branch 是 master 請更改 branch)
    ```
    $ cd ~/drone_ws/src/
    $ git clone https://github.com/danielyugoodboy/YCH_drone.git
    $ cd ~/drone_ws/
    $ catkin build YCH_drone --no-deps
    ```

### 1-6 執行
* 有一些額外的東西需要安裝

    ```
    # 1. Install openCV
    $ pip install opencv-python

    # 2. Install ros_numpy
    $ cd
    $ git clone https://github.com/eric-wieser/ros_numpy.git
    $ cd ros_numpy/
    $ python3 setup.py install
    ```

* 修改無人機中的相機角度與位置(自行選擇要不要修改)

    設定檔案在： drone_ws/src/Firmware/Tools/sitl_gazebo/models/iris_fpv_cam/iris_fpv_cam.sdf

    修改如下：
    ```
    <?xml version='1.0'?>
    <sdf version='1.5'>
    <model name='iris_fpv_cam'>

        <include>
        <uri>model://iris</uri>
        </include>

        <include>
        <uri>model://fpv_cam</uri>
        <pose>0 0 0 0 0.2 0</pose>
        </include>
        <joint name="fpv_cam_joint" type="fixed">
        <child>fpv_cam::link</child>
        <parent>iris::base_link</parent>
        <axis>
            <xyz>0 0 1</xyz>
            <limit>
            <upper>0</upper>
            <lower>0</lower>
            </limit>
        </axis>
        </joint>

    </model>
    </sdf>
    ```
#### A. Single Drone

* 開啟一個終端 - 開啟gazebo環境

    ```
    $ roslaunch YCH_drone start_Enviroment_1_single_drone.launch
    ```
    ![](https://i.imgur.com/1pJ1erm.jpg)

* 另外開啟一個終端 - 啟動執行檔

    a. KeyBoard control
    ```
    $ cd ~/drone_ws/src/YCH_drone/src/python
    $ python pj01_keyboard_control.py
    ```
    ![](https://i.imgur.com/t0bUpsa.png)

    b. Gym Envirom
    ```
    $ cd ~/drone_ws/src/YCH_drone/src/python
    $ python pj02_gym_control.py
    ```
    ![](https://i.imgur.com/6Zf0mCG.jpg)

* 另外開啟一個終端 - 確認是否有連接上mavros, 非必要僅用來檢查狀態

    ```
    $ rostopic echo /mavros/state
    ```
    ![](https://i.imgur.com/rRV9C8N.png)

#### B. Multi Drone

* 開啟一個終端 - 開啟gazebo環境

    ```
    $ roslaunch YCH_drone start_Enviroment_2_multi_drone.launch
    ```
    ![](https://i.imgur.com/BhTaIAc.png)

* 另外開啟一個終端 - 啟動執行檔

    ```
    $ cd ~/drone_ws/src/YCH_drone/src/python
    $ python pj04_multi_gym_control.py
    ```
    ![](https://i.imgur.com/Q5fVbZj.png)
    
    
#### C. VTOL
* 開啟編譯

    ```
    $ cd ~/drone_ws/src/Firmware
    $ make posix_sitl_default gazebo_standard_vtol
    ```

* 操控

    ```
    commander takeoff
    commander transition
    commander land
    ```

    ![](https://i.imgur.com/q32SAn3.png)


## 2. 實體無人機
### 2-1. 嵌入式系統開發版:

* Raspberry Pi 3
![](https://i.imgur.com/y9KLzOJ.jpg)

* Raspberry Pi 4
![](https://i.imgur.com/vf79geM.png)

![](https://i.imgur.com/4aslia4.png)


* UP board:
https://24h.pchome.com.tw/prod/DSAJ2B-A900F5ZK6
![](https://i.imgur.com/5BeHRIE.png)


* UP 4000:
https://up-board.org/up-4000/
![](https://i.imgur.com/beuXcGP.png)

* JETSON XAVIER NX
![](https://i.imgur.com/qfK3Q6q.png)


### 2-2. 實機操作
* 所有設備
    ![](https://i.imgur.com/njTRLiv.jpg)

* 裝槳順序
    ![](https://i.imgur.com/g8KeS3O.jpg)

    ![](https://i.imgur.com/AI2nA8p.jpg)

* 電池充電
    * 充電器沒有接上插座之前，不要把電池接上充電器，不然會回充
    * 一定要接上分壓板，不然離電池的電壓會不穩定
    * 遙控器電池充電的電流為：0.5A~1A
    * 無人機電池充電的電流為：3A~5A
    ![](https://i.imgur.com/XjBekdc.jpg)



## 3. Git 筆記

* 1. 安裝Git

* 2. 本地端註冊
    ```
    $ git config --global user.name "YuChiaHao"
    $ git config --global user.email "danielyugoodboy@gmail.com"
    ```

* 3. 初始化
    ```
    # 移動到要進行git管理的資料夾底下
    $ git init
    ```

* 4. 查看git管理狀態
    ```
    $ git status
    ```

* 5. 將更動過得檔案加入git索引
    ```
    $ git add .
    ```

* 6. 將加入到索引的檔案提交到本地端數據庫
    ```
    $ git commit -m "2022/11/15 Initial file"
    ```

* 7. 下載 GitKraken https://www.gitkraken.com/



## 4. Ros 筆記
教學 ： http://wiki.ros.org/ROS/Tutorials
建議閱讀章節:
```
1. Installing and Configuring Your ROS Environment
2. Navigating the ROS Filesystem
3. Creating a ROS Package
4. Building a ROS Package
5. Understanding ROS Nodes
6. Understanding ROS Topics
8. Using rqt_console and roslaunch
11. Writing a Simple Publisher and Subscriber (C++)
12. Writing a Simple Publisher and Subscriber (Python)
13. Examining the Simple Publisher and Subscriber
```
![](https://i.imgur.com/vgXhZYf.png)


1. rospack
rospack allows you to get information about packages. In this tutorial, we are only going to cover the find option, which returns the path to package.
```
$ rospack find [package_name]
```

2. roscd
roscd is part of the rosbash suite. It allows you to change directory (cd) directly to a package or a stack.
```
$ roscd <package-or-stack>[/subdir]
```

3. 建立package需要注意：
Building a catkin workspace and sourcing the setup file

4. Quick Overview of Graph Concepts
* Nodes: A node is an executable that uses ROS to communicate with other nodes.

* Messages: ROS data type used when subscribing or publishing to a topic.

* Topics: Nodes can publish messages to a topic as well as subscribe to a topic to receive messages.

* Master: Name service for ROS (i.e. helps nodes find each other)

* rosout: ROS equivalent of stdout/stderr

* roscore: Master + rosout + parameter server (parameter server will be introduced later)

5. 建立python node的時候,記得要把檔案改成可執行,不然rosrun會找不到
```
chmod +x talker.py
```