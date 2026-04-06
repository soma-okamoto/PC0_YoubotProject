

##################################################################################################################
IRT-MRO-Project手順
時刻設定    
sudo ntpdate -u ccntp.meijo-u.ac.jp

export ROS_IP=192.168.11.14
export ROS_MASTER_URI=http://192.168.11.14:11311

sudo iptables -P FORWARD ACCEPT

sudo iptables -F FORWARD
roslaunch rosbridge_server rosbridge_websocket.launch


---------------------------memo
hostname -I

ros不安定
rosparam set /use_sim_time false


rosservice call /dynamixel_workbench/execution "command: open"
rosservice call /dynamixel_workbench/execution "command: close"

--------------------youbot setting-------------------------------------------------------
ssh youbot@10.42.0.240

# Yobot 上で一度だけ実行（10.42.0.1 がホストのテザリング側 IP）
sudo ip route add 192.168.11.0/24 via 10.42.0.1 dev enp0s25



unset ROS_HOSTNAME
export ROS_IP=10.42.0.240
export ROS_MASTER_URI=http://192.168.11.14:11311
sudo iptables -F


###arm初期位置
roslaunch youbot_driver_suzuki youbot_driver_dual_arm.launch


###lIDER設定
sudo chmod 666 /dev/ttyACM1
sudo chmod 666 /dev/ttyACM2
roslaunch luh_laser_watchdog front.launch



---------------------pc setting---------------------------------------------------------



cd realsense_ws
source devel/setup.bash
export ROS_IP=192.168.11.14
export ROS_HOSTNAME=192.168.11.14
sudo iptables -F

roslaunch realsense2_camera cubeslam_camera.launch



cd catkin_ws
source devel/setup.bash
export ROS_IP=192.168.11.14
export ROS_HOSTNAME=192.168.11.14
sudo iptables -F


rosrun esaki_youbot_project_gradient gripper.py 

roslaunch esaki_slam youbot_move_base.launch 
roslaunch slam_toolbox online_async.launch 


roslaunch esaki_youbot_project_gradient IRM_youbot_base_move.launch

# rosrun esaki_youbot_project_gradient IRM_youbot_baseMove.py
# rosrun esaki_youbot_project_gradient Origin_move_pub.py 
# rosrun esaki_youbot_project_gradient Origin_move_pub_slam.py 


# cd src/yolov5_strongsort/Yolov5_StrongSORT_OSNet/
# ##map基準
# rosrun Yolov5_StrongSORT track_1.py
# ###base基準
# rosrun Yolov5_StrongSORT track.py


ros不安定
rosparam set /use_sim_time false


find . -name "*.py" -exec chmod +x {} \;
