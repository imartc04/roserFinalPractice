How to use:

- Execute launch file : roslaunch simple_navigation_ros marsiTurtlebotNavigation.launch

- Set aproximation of initial position of the robot. To do it
select a position in the rviz that corresponds to the position of the robot in gazebo

- Give marsi_nav node way points file : rosservice call /marsi_nav_set_waypoints "pointMapPath: '<path to housePoints.yaml'"

- Order robot to move : rosservice call /marsi_nav_move "mode: 0"

mode parmeter is one of the following 0:Sequential between way points, 1:Random between way points, 2:Cancel movement


Video demos :
https://drive.google.com/drive/folders/1luuHtvInq9yioKXQ9o_aDTu__nHb3SXM?usp=share_link
