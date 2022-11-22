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
* python=3.8 (推薦)


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

#### 1-5-A 從github下載

* 下載
    ```
    $ cd ~/drone_ws/src/
    $ git clone https://github.com/danielyugoodboy/YCH_drone.git
    $ cd ~/drone_ws/
    $ catkin build YCH_drone --no-deps
    ```
* 接下來可以直接跳到1-6

#### 1-5-B 從頭建立

* 建立package
    ```
    $ cd ~/drone_ws/src/
    $ catkin_create_pkg YCH_drone rospy
    $ cd ~/drone_ws/
    $ catkin build YCH_drone --no-deps
    ```

* 建立環境執行檔
    ```
    $ cd ~/drone_ws/src/YCH_drone/
    $ mkdir launch
    $ cd launch/
    ```
    在 launch 資料夾中建立一個名為 ```start_offb.launch``` 的檔案
    並且將以下內容貼到此檔案中

    ```launch
    <?xml version="1.0"?>
    <launch>
        <!-- Include the MAVROS node with SITL and Gazebo -->
        <include file="$(find px4)/launch/mavros_posix_sitl.launch">
        </include>

    </launch>
    ```

    如果想嘗試不同地圖的話，可以改成下面這種
    ```launch
    <?xml version="1.0"?>
    <launch>
        <!-- Include the MAVROS node with SITL and Gazebo -->
        <include file="$(find px4)/launch/mavros_posix_sitl.launch">
            <arg name="world" default="$(find mavlink_sitl_gazebo)/worlds/warehouse.world"/>
        </include>

    </launch>
    ```

* 建立操控執行檔
    ```
    $ cd ~/drone_ws/src/YCH_drone/src
    ```
* 在 src/ 資料夾中建立一個名為 ```test_drone.py``` 的檔案並且將以下內容貼到此檔案中
    ```python
    #! /usr/bin/env python

    import rospy
    from geometry_msgs.msg import PoseStamped
    from mavros_msgs.msg import State
    from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

    current_state = State()

    def state_cb(msg):
        global current_state
        current_state = msg


    if __name__ == "__main__":
        rospy.init_node("offb_node_py")

        state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb)

        local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)

        rospy.wait_for_service("/mavros/cmd/arming")
        arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)    

        rospy.wait_for_service("/mavros/set_mode")
        set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)


        # Setpoint publishing MUST be faster than 2Hz
        rate = rospy.Rate(20)

        # Wait for Flight Controller connection
        while(not rospy.is_shutdown() and not current_state.connected):
            rate.sleep()

        pose = PoseStamped()

        pose.pose.position.x = 0
        pose.pose.position.y = 0
        pose.pose.position.z = 2

        # Send a few setpoints before starting
        for i in range(100):   
            if(rospy.is_shutdown()):
                break

            local_pos_pub.publish(pose)
            rate.sleep()

        offb_set_mode = SetModeRequest()
        offb_set_mode.custom_mode = 'OFFBOARD'

        arm_cmd = CommandBoolRequest()
        arm_cmd.value = True

        last_req = rospy.Time.now()

        while(not rospy.is_shutdown()):
            if(current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                if(set_mode_client.call(offb_set_mode).mode_sent == True):
                    rospy.loginfo("OFFBOARD enabled")

                last_req = rospy.Time.now()
            else:
                if(not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                    if(arming_client.call(arm_cmd).success == True):
                        rospy.loginfo("Vehicle armed")

                    last_req = rospy.Time.now()

            local_pos_pub.publish(pose)

            rate.sleep()
    ```

* build YCH_drone
    ```
    $ cd ~/drone_ws/
    $ catkin build YCH_drone --no-deps
    ```

### 1-7 執行

* 開啟一個終端 (開啟gazebo環境)
    ```
    $ roslaunch YCH_drone start_offb.launch 
    ```
    ![](https://i.imgur.com/1pJ1erm.jpg)

* 另外開啟一個終端 (確認是否有連接上mavros, 非必要緊用來檢查狀態)
    ```
    $ rostopic echo /mavros/state
    ```
    ![](https://i.imgur.com/rRV9C8N.png)

* 另外開啟一個終端 (啟動執行檔)
    ```
    $ cd ~/drone_ws/src/YCH_drone/src/python  # 或是 $ roscd YCH_drone/src/python
    $ python pj02_gym_control.py
    ```
    ![](https://i.imgur.com/6Zf0mCG.jpg)


## 2. Git 筆記

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