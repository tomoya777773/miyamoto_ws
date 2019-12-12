roslaunch ur_modern_driver ur5_bringup_joint_limited.launch robot_ip:=163.221.44.227
roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch 
python object_pcl_publisher.py
roslaunch kinect2_bridge kinect2_bridge.launch publish_tf:=true
roslaunch ur5_moveit_config moveit_rviz_miyamoto.launch config:=true

To do 

/make test data
/parameter
/plot surf? scatter?

/gpis実装完成させる
/mtgp