roslaunch ur_modern_driver ur5_bringup_joint_limited.launch robot_ip:=163.221.44.227
roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch 
rosrun sliding_motion touching_body_pcl 
roslaunch kinect2_bridge kinect2_bridge.launch publish_tf:=true
roslaunch ur5_moveit_config moveit_rviz_miyamoto.launch config:=true

To do 

カメラのキャリブレーション