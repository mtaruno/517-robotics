
# when building the dev vontainer dont forget to install the fetch ws sh script again
roscore # this enables the master node

roslaunch fetch_gazebo playground.launch

rosrun applications joint_reader_demo.py
rosrun applications head_demo.py look_at base_link 1 0 0.3
rosrun applications head_demo.py pan_tilt 0 0
rosrun applications keyboard_teleop.py
rosrun rviz rviz
rosservice info /global_localization
rosservice call /global_localization

roslaunch fetch_moveit_config move_group.launch # check if moveit server is running
rosrun applications check_cart_pose.py plan 0.5 0 1
rosrun applications check_cart_pose.py plan 1 0 1
rosrun applications check_cart_pose.py ik 0.5 0 1